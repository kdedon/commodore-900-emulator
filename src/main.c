/* main.c — entry point + host console glue + CPU self-test.
 *
 * Usage (see usage() / --help for the full list):
 *   c900 [--firmware=DIR] [--disk=FILE] [--floppy=FILE] [--trace] [--max=N] [--input="..."]
 *        [--break=SEG:OFF] [--dump=SPEC]
 *   c900 --selftest
 *   c900 --help
 * Value options accept both "--opt=VALUE" and "--opt VALUE".
 *
 * --firmware defaults to ../rom and --disk to ../disk/hdd.bin (both relative to
 * the working directory), so running the binary from the bin/ directory with no
 * arguments finds the ROMs and disk image in their sibling directories.
 * --floppy is optional; when given it attaches a floppy image to the floppy
 * drive (/dev/fd1 under the shipped Coherent image).
 *
 * The SCC channel B serial console is redirected to this process's stdin/
 * stdout, so the C900 boot ROM banner and any OS console appear directly in
 * the terminal, and keystrokes are delivered to the machine.
 *
 * The three console_* functions below, and wire.c's endpoint for SCC channel A,
 * are the ONLY host-OS-specific code in the emulator; bus.c calls them to move
 * bytes between the SCC and the terminal (or the wire). */
#include "emu.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>

#ifdef _WIN32
/* Windows console: conio gives us a non-blocking key poll directly. The one
 * thing to configure is ENABLE_PROCESSED_INPUT: with it on (the default), the
 * console host turns Ctrl-C into a CTRL_C_EVENT at keypress time, so the 0x03
 * never lands in the input buffer and the guest can't see it. Turn it off so
 * Ctrl-C is an ordinary key event; the saved mode is restored on shutdown. */
#include <conio.h>
#include <io.h>
#include <windows.h>
static HANDLE g_hin;
static DWORD  g_oldmode;
static bool   g_havemode;
int console_poll_char(void){ return _kbhit() ? _getch() : -1; }   /* -1 = no key ready */
void console_init(void){
    g_hin = GetStdHandle(STD_INPUT_HANDLE);
    if (GetConsoleMode(g_hin, &g_oldmode)) {
        g_havemode = true;
        SetConsoleMode(g_hin, g_oldmode & ~ENABLE_PROCESSED_INPUT);
    }
}
void console_shutdown(void){ if (g_havemode) SetConsoleMode(g_hin, g_oldmode); }
#else
/* POSIX terminal: put stdin into raw, non-blocking mode so the emulator sees
 * each keystroke immediately (no line buffering) and the host doesn't echo it
 * — the guest OS does its own echo over the serial line. The original termios
 * settings are saved and restored on shutdown so the user's shell is left sane. */
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
static struct termios g_old;   /* saved terminal settings, restored on exit */
void console_init(void){
    struct termios t; tcgetattr(0,&g_old); t=g_old;
    t.c_lflag &= ~(ICANON|ECHO|ISIG);       /* raw: no line editing, no local echo,
                                             * no Ctrl-C→SIGINT (guest gets the 0x03) */
    t.c_cc[VMIN]=0; t.c_cc[VTIME]=0;        /* read() returns immediately if no byte */
    tcsetattr(0,TCSANOW,&t);
    int fl=fcntl(0,F_GETFL); fcntl(0,F_SETFL,fl|O_NONBLOCK);
}
void console_shutdown(void){ tcsetattr(0,TCSANOW,&g_old); }
int console_poll_char(void){ unsigned char b; int n=read(0,&b,1); return n==1 ? b : -1; }
#endif

/* Emit one byte the machine transmitted on the console; flush so output is not
 * held back while the guest waits (interactively) for our reply. */
void console_put_char(int ch){ putchar(ch & 0xFF); fflush(stdout); }

/* SIGINT handler: keyboard Ctrl-C is delivered to the guest (see console_init),
 * so this only fires for an external signal (kill, or Ctrl-Break on Windows) —
 * ask the run loop to stop cleanly so the terminal gets restored. The
 * interactive way to quit is Ctrl-] (handled in the run loop's console poll). */
static Machine *g_m;
static void on_sigint(int s){ (void)s; if (g_m) g_m->stop = true; }

/* ── minimal CPU self-test: exercises the interpreter without the ROM ──
 * Loads a tiny hand-assembled Z8001 program into RAM, single-steps it, and
 * checks register results. Verifies decode + ALU + branch + memory paths. */
int lout_layout_selftest(void);   /* uexec.c */

/* ── The serial ports (run by --selftest) ───────────────────────────────────
 * Three properties, none of which any other test in the tree can see:
 *
 *   1. the six ports of emu.h's SERIAL PORT TABLE are ADDRESSABLE and
 *      INDEPENDENT -- a byte delivered to one appears at one address and at
 *      no other, and a vector written to one chip does not reach another;
 *   2. --wire still means port 1 and behaves as it always has;
 *   3. --rx-overrun loses a character, and its absence does not.
 *
 * Property 3 goes through scc_wire_service(), which IS the run loop's receive
 * gate, so what passes here is the code that runs in a real boot rather than a
 * restatement of it. */

/* An SCC register's I/O address: chan bit is address bit 5, register bits 4:1
 * (see scc_read).  Chip bases are the table's: 0x0100, 0x0300, 0x0380. */
static uint16_t sccaddr(int port, int reg){
    static const uint16_t chipbase[SCC_CHIPS] = { 0x0100, 0x0300, 0x0380 };
    return (uint16_t)(chipbase[port / 2] + ((port & 1) << 5) + (reg << 1));
}
static uint8_t sccrd(Machine *m, int port, int reg){
    return (uint8_t)(io_read(m, sccaddr(port, reg), false, false) >> 8);
}
static void sccwr(Machine *m, int port, int reg, uint8_t v){
    io_write(m, sccaddr(port, reg), (uint16_t)v << 8, false, false);
}

static int serial_selftest(void){
    Machine *m = machine_new();
    int ok = 1;
    #define SFAIL(...) do { printf("serial FAIL: " __VA_ARGS__); ok = 0; } while (0)

    /* (a) With no LR board fitted -- the default -- 0x0300 and 0x0380 are not
     * decoded at all, and read back as the floating 0 they always did. */
    if (m->lr_scc) SFAIL("LR SCCs are fitted by default\n");
    if (io_read(m, 0x0300, false, false) != 0 || io_read(m, 0x0380, false, false) != 0)
        SFAIL("0x0300/0x0380 answer with no LR board fitted\n");
    if (scc_port(m, 2) || scc_port(m, 5)) SFAIL("LR ports exist with no LR board\n");

    /* (b) Fitted, all six ports exist, are distinct, and port 1 is still the
     * channel A that --wire has always attached. */
    m->lr_scc = true;
    for (int p = 0; p < SCC_PORTS; p++)
        if (!scc_port(m, p)) SFAIL("port %d missing with the LR board fitted\n", p);
    if (scc_port(m, 0) != &m->scc_b) SFAIL("port 0 is not the console channel\n");
    if (scc_port(m, 1) != &m->scc_a) SFAIL("port 1 is not channel A (--wire's port)\n");
    for (int a = 0; a < SCC_PORTS; a++)
        for (int b = a + 1; b < SCC_PORTS; b++)
            if (scc_port(m, a) == scc_port(m, b)) SFAIL("ports %d and %d alias\n", a, b);

    /* (c) Per-chip registers.  al.c gives each SCC its own interrupt vector
     * (VECTOR 0x10, +16 per chip), so a WR2 write must reach both channels of
     * ITS chip and neither channel of any other. */
    sccwr(m, 0, 2, 0x10); sccwr(m, 2, 2, 0x20); sccwr(m, 4, 2, 0x30);
    if (scc_port(m,1)->wr[2] != 0x10) SFAIL("WR2 did not reach chip 0's other channel\n");
    if (scc_port(m,3)->wr[2] != 0x20) SFAIL("WR2 did not reach chip 1's other channel\n");
    if (scc_port(m,0)->wr[2] != 0x10 || scc_port(m,2)->wr[2] != 0x20 ||
        scc_port(m,4)->wr[2] != 0x30) SFAIL("a WR2 write leaked between chips\n");

    /* (d) Receiver independence: one byte per port, each seen only at its own
     * address.  RR0 D0 is Rx Character Available; RR8 is the data. */
    for (int p = 0; p < SCC_PORTS; p++) scc_rx_port(m, p, (uint8_t)('A' + p));
    for (int p = 0; p < SCC_PORTS; p++)
        if (!(sccrd(m, p, 0) & 0x01)) SFAIL("port %d shows no character waiting\n", p);
    for (int p = 0; p < SCC_PORTS; p++) {
        uint8_t got = sccrd(m, p, 8);
        if (got != (uint8_t)('A' + p))
            SFAIL("port %d read '%c', wanted '%c'\n", p, got, 'A' + p);
    }
    for (int p = 0; p < SCC_PORTS; p++)
        if (sccrd(m, p, 0) & 0x01) SFAIL("port %d still shows a character after RR8\n", p);

    /* (e) Only the addressed port receives.  Give port 3 a byte and require
     * every other port to stay empty -- the property verify-conN needs. */
    scc_rx_port(m, 3, 'q');
    for (int p = 0; p < SCC_PORTS; p++) {
        bool avail = (sccrd(m, p, 0) & 0x01) != 0;
        if (avail != (p == 3)) SFAIL("port %d avail=%d after a byte to port 3\n", p, avail);
    }
    (void)sccrd(m, 3, 8);

    /* (f) The wires, end to end through the real receive gate.  Ports 1, 3
     * and 5 -- one on each chip -- get a host endpoint apiece. */
    /* Guarded because the host-side end of the pair is read and written with
     * POSIX read()/write(), which this file only has a declaration for off
     * Windows -- where wire_test_pair would refuse anyway. */
#ifndef _WIN32
    int h1 = -1, h3 = -1, h5 = -1;
    if (wire_test_pair(1, &h1) == 0 && wire_test_pair(3, &h3) == 0 &&
        wire_test_pair(5, &h5) == 0) {
        m->wire_port_on[1] = m->wire_port_on[3] = m->wire_port_on[5] = true;
        m->any_wire = true;
        if (write(h1, "a", 1) != 1 || write(h3, "c", 1) != 1 || write(h5, "e", 1) != 1)
            SFAIL("could not write to the test wires\n");
        scc_wire_service(m);
        if (scc_port(m,1)->rx_data != 'a' || !scc_port(m,1)->rx_avail) SFAIL("port 1 wire byte lost\n");
        if (scc_port(m,3)->rx_data != 'c') SFAIL("port 3 got '%c'\n", scc_port(m,3)->rx_data);
        if (scc_port(m,5)->rx_data != 'e') SFAIL("port 5 got '%c'\n", scc_port(m,5)->rx_data);
        if (scc_port(m,2)->rx_avail || scc_port(m,4)->rx_avail)
            SFAIL("an unwired port received someone else's traffic\n");

        /* Transmit goes out its own wire and no other. */
        sccwr(m, 3, 8, 'Z');
        char rb[4];
        if (read(h3, rb, 1) != 1 || rb[0] != 'Z') SFAIL("port 3 transmit did not reach its wire\n");
        if (read(h1, rb, 1) > 0 || read(h5, rb, 1) > 0) SFAIL("a transmit crossed to another wire\n");

        /* OVERRUN OFF (the default): a second byte arrives while the first is
         * unread.  The gate refuses it, the endpoint keeps it, nothing is
         * lost -- which is exactly why a receive ring has been untestable. */
        if (m->rx_overrun_on) SFAIL("--rx-overrun is on by default\n");
        if (write(h1, "b", 1) != 1) SFAIL("could not write the second byte\n");
        scc_wire_service(m);
        if (scc_port(m,1)->rx_data != 'a') SFAIL("overrun off: the unread byte was overwritten\n");
        if (scc_port(m,1)->rx_lost != 0) SFAIL("overrun off: a character was counted lost\n");
        if (sccrd(m, 1, 1) != 0x01) SFAIL("overrun off: RR1 is not 0x01\n");

        /* OVERRUN ON: the same second byte now lands, 'a' is gone, and RR1 D5
         * latches until an Error Reset (WR0 command 6 = 0x30). */
        m->rx_overrun_on = true;
        scc_wire_service(m);
        if (scc_port(m,1)->rx_data != 'b') SFAIL("overrun on: the byte did not arrive\n");
        if (scc_port(m,1)->rx_lost != 1) SFAIL("overrun on: %llu characters lost, wanted 1\n",
                                               (unsigned long long)scc_port(m,1)->rx_lost);
        if (!(sccrd(m, 1, 1) & 0x20)) SFAIL("overrun on: RR1 D5 did not latch\n");
        if (scc_port(m,3)->rx_overrun) SFAIL("an overrun latched on the wrong port\n");
        sccwr(m, 1, 0, 0x30);                       /* WR0: Error Reset */
        if (sccrd(m, 1, 1) & 0x20) SFAIL("Error Reset did not clear RR1 D5\n");

        close(h1); close(h3); close(h5);
        if (ok) printf("serial ports PASSED (6 addressable, independent; overrun on/off)\n");
    } else {
        printf("serial ports PASSED (no AF_UNIX here; wire checks skipped)\n");
    }
#else
    if (ok) printf("serial ports PASSED (6 addressable, independent; wires need AF_UNIX)\n");
#endif
    #undef SFAIL
    wire_close();
    free(m->ram); free(m);
    return ok;
}

static int selftest(void){
    Machine *m = machine_new();
    CPU *c = &m->cpu;
    /* Put PSA reset vector into ROM image directly (we bypass the file). */
    /* Program at physical 0x000100 in RAM segment 0? RAM starts at seg 1.
     * Use segment 1 (phys 0x010000) for code+data. */
    #define PUT16(a,v) do{ m->ram[(a)]=(uint8_t)((v)>>8); m->ram[(a)+1]=(uint8_t)(v);}while(0)
    uint32_t base = 0x080000;   /* segment 8 — populated DRAM */
    int p = base;
    /* LD R1, #0x1234        21 01 12 34 */
    PUT16(p, 0x2101); PUT16(p+2, 0x1234); p+=4;
    /* LD R2, #0x0001        21 02 00 01 */
    PUT16(p, 0x2102); PUT16(p+2, 0x0001); p+=4;
    /* ADD R1, R2            81 21 */
    PUT16(p, 0x8121); p+=2;
    /* LD R3, #0x00FF        21 03 00 FF */
    PUT16(p, 0x2103); PUT16(p+2, 0x00FF); p+=4;
    /* AND R1, R3            87 31 */
    PUT16(p, 0x8731); p+=2;
    /* SUB R1, R2 (R1=0x34)  83 21 */
    PUT16(p, 0x8321); p+=2;
    /* the stop doorbell: LD R1,#0xC900 / OUT 0x0FFE,R1  (21 01 C9 00, 3B 16 0F FE) */
    PUT16(p, 0x2101); PUT16(p+2, 0xC900); p+=4;
    PUT16(p, 0x3B16); PUT16(p+2, 0x0FFE); p+=4;

    /* set CPU state: segmented+system, PC = seg1:offset0 */
    memset(c->R,0,sizeof c->R);
    c->fcw = FCW_SEG | FCW_SN;
    c->pc = 0x080000;
    c->psap = 0;
    c->halted = false; c->irq_req=0;

    for (int i=0;i<6;i++) cpu_step(c);

    int ok = 1;
    if (c->R[1] != 0x0034) { printf("selftest FAIL: R1=%04X expected 0034\n", c->R[1]); ok=0; }
    if (c->R[2] != 0x0001) { printf("selftest FAIL: R2=%04X expected 0001\n", c->R[2]); ok=0; }
    if (ok) printf("CPU self-test PASSED (R1=%04X R2=%04X R3=%04X)\n", c->R[1],c->R[2],c->R[3]);

    /* ALU spot checks against the expected Z8000 semantics */
    AluResult r = alu_exec(OpAdd, 0x7FFF, 0x0001, false, WWord);
    if (r.value != 0x8000 || !(r.flags & F_PV) || !(r.flags & F_S)) { printf("ALU FAIL add overflow\n"); ok=0; }
    r = alu_exec(OpSub, 0x0000, 0x0001, false, WWord);
    if (r.value != 0xFFFF || !(r.flags & F_C)) { printf("ALU FAIL sub borrow\n"); ok=0; }
    r = alu_exec(OpSll, 0x0001, 4, false, WWord);
    if (r.value != 0x0010) { printf("ALU FAIL sll\n"); ok=0; }
    if (ok) printf("ALU checks PASSED\n");

    /* Stop doorbell (--stop-port): the two instructions appended above are the
     * whole guest side of it, so stepping them is a real end-to-end check of
     * the port write reaching io_write and ending the run. */
    m->stop_port = 0x0FFE;
    cpu_step(c); cpu_step(c);
    if (!m->stop || !m->stop_why) { printf("stop-doorbell FAIL: port write did not stop the run\n"); ok=0; }
    else printf("stop doorbell PASSED (%s)\n", m->stop_why);

    /* The l.out loader: which segment each section group lands in.  It is
     * pure header arithmetic, so it is checked here rather than by running a
     * program -- a wrong answer produces a program that runs and addresses
     * nothing, which is the hardest kind of failure to see from outside. */
    if (!lout_layout_selftest()) ok = 0;

    /* The serial ports: the SERIAL PORT TABLE, the wires and the overrun. */
    if (!serial_selftest()) ok = 0;

    free(m->ram); free(m);
    return ok ? 0 : 1;
}

/* Print command-line usage. */
static void usage(FILE *out, const char *prog){
    fprintf(out,
        "Usage: %s [options]\n"
        "  --firmware=DIR   ROM directory with bios_h.bin + bios_l.bin (default ../rom)\n"
        "  --disk=FILE      raw hard-disk image (default ../disk/hdd.bin; required)\n"
        "  --floppy=FILE    raw floppy image for the floppy drive (optional)\n"
        "  --trace          print periodic PC/FCW progress to stderr\n"
        "  --max=N          stop after N instructions (0 = run until Ctrl-])\n"
        "  --input=\"...\"    scripted console keystrokes (\\r \\n \\t \\\\ escapes).\n"
        "                   Bytes are PACED: each one waits until the guest looks\n"
        "                   ready for it (a prompt printed, the console quiet, or\n"
        "                   the guest spinning on the receiver), because a byte\n"
        "                   handed over early is eaten by whatever read the guest\n"
        "                   is in.  Two escapes queue no byte and change that:\n"
        "                     \\g  from here on, stop waiting for a prompt (a\n"
        "                         full-screen program never prints one)\n"
        "                     \\i  the NEXT byte is TYPE-AHEAD: delivered as soon\n"
        "                         as the receiver is free, with no pacing at all,\n"
        "                         so it can reach a program that is printing --\n"
        "                         which is where a ^S/^Q/^C has to land.  One byte\n"
        "                         only; the rest of the script stays paced\n"
        "  --input-mark=TEXT  hold every \\i byte until the guest has PRINTED\n"
        "                   TEXT on the console, then release them.  This is\n"
        "                   how a scripted ^S is aimed: \"send it when that\n"
        "                   appears\", which is exact and does not move when the\n"
        "                   guest is rebuilt, unlike an instruction count.  The\n"
        "                   mark is a starting gun -- once seen it stays open,\n"
        "                   and paced bytes are not affected by it at all\n"
        "  --break=SEG:OFF[/N]  record CPU state at a guest PC, before the\n"
        "                   instruction there executes: PC, instruction count, FCW,\n"
        "                   mode and all sixteen registers, as one \"[dbg] brk\"\n"
        "                   key=value line on stderr.  Addresses are hex; SEG may\n"
        "                   be a segment NUMBER (32) or a Z8001 segment WORD\n"
        "                   (B200), so a PC out of a guest trap message pastes in\n"
        "                   unchanged.  /N (decimal, default 1) is how many hits\n"
        "                   to record; /0 records every one.  Repeatable\n"
        "  --dump=SPEC      a memory window, printed at every --break hit and once\n"
        "                   when the run ends, as a \"[dbg] mem\" line.  Two forms:\n"
        "                     SEG:OFF+LEN   a fixed logical address\n"
        "                     rrN:DISP+LEN  relative to the long pointer RRn --\n"
        "                                   rr14:0+32 is the stack frame, wherever\n"
        "                                   it is on this run\n"
        "                   Addresses and DISP hex (DISP may be negative), LEN\n"
        "                   decimal.  Read through the MMU exactly as the guest\n"
        "                   would, and leaving no trace in it.  Repeatable\n"
        "  --stop-on=LIST   which early-stop channels are armed, comma-separated:\n"
        "                   \"idle\", \"port\" (the default), \"break\", \"all\" or\n"
        "                   \"none\".\n"
        "                   idle: for test harnesses, OFF unless asked for.  With\n"
        "                     --input, stop once every scripted byte has been fed\n"
        "                     and the guest is idle at a prompt: a prompt character\n"
        "                     ('#' or '>') printed last, console quiet for --idle\n"
        "                     instructions, and the guest spinning on the receiver\n"
        "                   port: stop when the guest writes the word 0xC900 to the\n"
        "                     --stop-port I/O port -- an explicit \"I am finished\"\n"
        "                     from the guest, so it cannot fire on its own\n"
        "                   break: stop once a --break has recorded its last hit,\n"
        "                     with that instruction NOT yet executed\n"
        "  --idle=N         instructions of console silence the idle channel waits\n"
        "                   for, and arms it (default 40000000)\n"
        "  --stop-port=P    the port channel's I/O port (default 0x0FFE, which\n"
        "                   nothing on the machine answers)\n"
        "  --require-stop   exit 3 if the run ended by exhausting --max rather than\n"
        "                   by stopping, so a caller can assert that the session\n"
        "                   finished rather than merely that the emulator survived\n"
        "  --wire=PATH      attach SCC channel A (the guest's /dev/tty51) to the\n"
        "                   AF_UNIX stream socket PATH, so two emulators joined\n"
        "                   through one bridge share a serial link\n"
        "  --wire-trace     hexdump every byte crossing a wire to stderr\n"
        "  --wireN=PATH     attach serial port N the same way, for N in 1..5.\n"
        "                   The C900 has three Z8030 SCCs, six ports (the map is\n"
        "                   COHERENT al.c's own altty[] table):\n"
        "                     0  0x0100  motherboard chan B  the ROM console\n"
        "                     1  0x0120  motherboard chan A  /dev/tty51 (= --wire)\n"
        "                     2  0x0300  LR SCC #1 chan B    rear DB25 CN3\n"
        "                     3  0x0320  LR SCC #1 chan A    rear DB25 CN4\n"
        "                     4  0x0380  LR SCC #2 chan B    header CN5\n"
        "                     5  0x03A0  LR SCC #2 chan A    header CN6\n"
        "                   Port 0 is the console and is stdin/stdout, so there is\n"
        "                   no --wire0.  Wiring 2..5 fits the LR board implicitly.\n"
        "  --lr-scc         fit the LR board's two SCCs (ports 2-5) with no wire\n"
        "                   attached.  Without this, and without a --wireN for one\n"
        "                   of its ports, 0x0300-0x03FF is not decoded at all\n"
        "  --rx-overrun     let a receiver LOSE a character: a byte arriving on a\n"
        "                   wired port while the guest has not read the previous\n"
        "                   one overwrites it and latches RR1 D5 (Rx Overrun)\n"
        "                   until an Error Reset, as the real SCC does.  Off by\n"
        "                   default, where the endpoint holds the byte instead and\n"
        "                   nothing can ever be lost\n"
        "  --rtc=WHEN       seed the MSM58321 clock: \"host\" (default), \"none\"\n"
        "                   for a machine with no module fitted, or an explicit\n"
        "                   YYYY-MM-DDTHH:MM:SS (the module has no battery here)\n"
        "  --rtc-ips=N      instructions of emulated CPU time per RTC second\n"
        "                   (default 1500000; lower runs the crystal fast)\n"
        "  --selftest       run the built-in CPU/ALU regression and exit\n"
        "\n"
        "User-mode execution (no ROM, no disk, no machine): run a linked Z8001\n"
        "l.out as a process.  These must be the FIRST argument, because every\n"
        "argument after the program's path belongs to the guest:\n"
        "  --exec FILE [ARGS...]      run crt0+main against a COHERENT syscall\n"
        "                             shim serviced on the host filesystem\n"
        "                             (-runexec is the same mode)\n"
        "  -runobjint FILE [ARGS...]  CALL f(ARGS...) and print R1\n"
        "  -runobj FILE [A B WANT [BITS]]  as -runobjint with float operands\n"
        "  --help, -h       show this help and exit\n"
        "\n"
        "Exit status: 0 the run stopped (a stop channel, Ctrl-], a halted guest, or\n"
        "--max when --require-stop was not given), 1 start-up failure (ROM, disk,\n"
        "wire), 2 bad command line, 3 --max exhausted under --require-stop.\n"
        "\n"
        "Value options accept either \"--opt=VALUE\" or \"--opt VALUE\".\n"
        "Run from the bin/ directory so the ../rom and ../disk defaults resolve.\n",
        prog);
}

/* Match a value-taking option in either "--name=VALUE" or "--name VALUE" form.
 * Returns the value string, or NULL if argv[*i] is not this option. When the
 * value is a separate argument, *i is advanced past it. */
static const char *opt_value(char **argv, int argc, int *i, const char *name){
    size_t n = strlen(name);
    const char *a = argv[*i];
    if (strncmp(a, name, n) != 0) return NULL;
    if (a[n] == '=') return a + n + 1;                       /* --name=VALUE */
    if (a[n] == '\0' && *i + 1 < argc) return argv[++(*i)];  /* --name VALUE */
    return NULL;
}

/* User-mode execution (src/uexec.c): these run a linked l.out as a PROCESS and
 * share nothing with the machine path below -- no ROM, no disk, no console. */
int run_objint(const char *path, int argc, char **argv);
int run_floatobj(const char *path, uint64_t a, uint64_t b, uint64_t want, int retbits);
int run_exec(const char *path, int argc, char **argv);

/* uexec_dispatch handles the user-mode modes, which must be recognised before
 * the machine option parser sees the command line: everything after the
 * program's path belongs to the GUEST, not to the emulator, so `--exec prog -x'
 * must not have -x taken as an emulator option.  Returns -1 when argv is not a
 * user-mode invocation. */
static int uexec_dispatch(int argc, char **argv){
    if (argc < 3) return -1;
    const char *mode = argv[1];
    if (!strcmp(mode, "-runobjint"))
        return run_objint(argv[2], argc - 3, argv + 3);
    if (!strcmp(mode, "-runexec") || !strcmp(mode, "--exec"))
        return run_exec(argv[2], argc - 3, argv + 3);
    if (!strcmp(mode, "-runobj")) {
        /* -runobj <l.out> [hexA hexB hexWant [retbits]] -- the float entry
         * points; the defaults are 1.5, 2.5 and 4.0 as IEEE doubles. */
        uint64_t a = 0x3FF8000000000000ull, b = 0x4004000000000000ull,
                 want = 0x4010000000000000ull;
        int bits = 64;
        if (argc >= 6) {
            a = strtoull(argv[3], 0, 0); b = strtoull(argv[4], 0, 0);
            want = strtoull(argv[5], 0, 0);
        }
        if (argc >= 7) bits = (int)strtol(argv[6], 0, 0);
        return run_floatobj(argv[2], a, b, want, bits);
    }
    return -1;
}

int main(int argc, char **argv){
    {   int rc = uexec_dispatch(argc, argv);
        if (rc >= 0) return rc; }

    /* --- argument parsing (see usage() for the full option list) ------------
     * Value options accept both "--opt=VALUE" and "--opt VALUE". */
    const char *fw = "../rom", *disk = "../disk/hdd.bin", *floppy = NULL, *g_input = NULL;
    const char *rtcseed = "host", *wire = NULL, *g_inmark = NULL;
    const char *wireN[SCC_PORTS] = {0};   /* --wire1 .. --wire5, by port number */
    bool lr_scc = false, rx_overrun = false;
    unsigned long long rtc_ips = 0;
    bool trace = false, dosel = false, wtrace = false;
    unsigned long long g_max = 0;
    /* the same silence the scripted-input pacing uses for its longest wait */
    unsigned long long g_idle = 40000000ull;
    unsigned long g_stopport = 0x0FFE;   /* see bus.c io_write: unclaimed I/O space */
    /* Idle is NOT armed by default: an emulator left alone must still be
     * running when its owner returns, however long the guest stays silent.
     * The port channel is, since only the guest itself can fire it. */
    const char *g_stopon = "port";
    bool idle_asked = false;             /* --idle=N given: tuning it asks for it */
    bool require_stop = false;
    for (int i=1;i<argc;i++){
        const char *v;
        if      ((v = opt_value(argv,argc,&i,"--rtc-ips"))) rtc_ips = strtoull(v,0,0);
        else if ((v = opt_value(argv,argc,&i,"--rtc")))     rtcseed = v;
        else if ((v = opt_value(argv,argc,&i,"--firmware"))) fw = v;
        else if ((v = opt_value(argv,argc,&i,"--disk")))     disk = v;
        else if ((v = opt_value(argv,argc,&i,"--floppy")))   floppy = v;
        else if ((v = opt_value(argv,argc,&i,"--max")))      g_max = strtoull(v,0,0);
        else if ((v = opt_value(argv,argc,&i,"--input-mark"))) g_inmark = v;
        else if ((v = opt_value(argv,argc,&i,"--input")))    g_input = v;
        else if ((v = opt_value(argv,argc,&i,"--idle")))   { g_idle = strtoull(v,0,0); idle_asked = true; }
        else if ((v = opt_value(argv,argc,&i,"--stop-port"))) g_stopport = strtoul(v,0,0);
        else if ((v = opt_value(argv,argc,&i,"--stop-on")))  g_stopon = v;
        else if ((v = opt_value(argv,argc,&i,"--break"))) {
            if (dbg_add_break(v)) { fprintf(stderr,"--break: expected SEG:OFF[/N] in hex, e.g. 32:3EAC or B200:3EAC/2\n"); return 2; }
        }
        else if ((v = opt_value(argv,argc,&i,"--dump"))) {
            if (dbg_add_dump(v)) { fprintf(stderr,"--dump: expected SEG:OFF+LEN or rrN:DISP+LEN, e.g. 0F:EF80+32 or rr14:0+32\n"); return 2; }
        }
        else if (!strcmp(argv[i],"--require-stop")) require_stop = true;
        else if ((v = opt_value(argv,argc,&i,"--wire")))     wire = v;
        else if (!strcmp(argv[i],"--wire-trace")) wtrace = true;
        else if (!strcmp(argv[i],"--lr-scc")) lr_scc = true;
        else if (!strcmp(argv[i],"--rx-overrun")) rx_overrun = true;
        else if (!strcmp(argv[i],"--trace")) trace = true;
        else if (!strcmp(argv[i],"--selftest")) dosel = true;
        else if (!strcmp(argv[i],"--help") || !strcmp(argv[i],"-h")) { usage(stdout, argv[0]); return 0; }
        else {
            /* --wireN=PATH for N in 1..5 (the SERIAL PORT TABLE's port
             * numbers).  --wire1 is the same port as plain --wire; there is
             * no --wire0 because port 0 is the console, which is stdin/stdout
             * and cannot be handed to a socket. */
            int matched = 0;
            for (int p = SERPORT_MB_A; p < SCC_PORTS; p++) {
                char opt[10];
                snprintf(opt, sizeof opt, "--wire%d", p);
                if ((v = opt_value(argv,argc,&i,opt))) { wireN[p] = v; matched = 1; break; }
            }
            if (!matched) { fprintf(stderr,"unknown arg: %s\n\n", argv[i]); usage(stderr, argv[0]); return 2; }
        }
    }

    if (dosel) return selftest();

    Machine *m = machine_new();
    g_m = m;
    if (machine_load_rom(m, fw) != 0) {
        fprintf(stderr,"failed to load ROM from %s (need bios_h.bin + bios_l.bin)\n", fw);
        return 1;
    }
    if (machine_attach_disk(m, disk) != 0) {
        fprintf(stderr,"failed to open hard-disk image %s (the machine requires a hard disk)\n", disk);
        return 1;
    }
    if (floppy && machine_attach_floppy(m, floppy) != 0) {
        fprintf(stderr,"failed to open floppy image %s\n", floppy);
        return 1;
    }

    /* RTC: the module has no battery in here, so the start-up time comes from
     * the host clock unless --rtc names one (or says the module is absent). */
    if (rtc_ips) m->rtc.ips = rtc_ips;
    if (!strcmp(rtcseed, "none") || !strcmp(rtcseed, "off")) {
        m->rtc.present = false;
    } else if (!strcmp(rtcseed, "host")) {
        time_t now = time(NULL);
        struct tm *lt = localtime(&now);
        rtc_set_time(m, lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday,
                     lt->tm_hour, lt->tm_min, lt->tm_sec);
    } else {
        int y, mo, d, h, mi, s;
        char sep;
        if (sscanf(rtcseed, "%d-%d-%d%c%d:%d:%d", &y,&mo,&d,&sep,&h,&mi,&s) != 7) {
            fprintf(stderr,"--rtc: expected host, none, or YYYY-MM-DDTHH:MM:SS\n");
            return 2;
        }
        rtc_set_time(m, y, mo, d, h, mi, s);
    }

    m->trace = trace;
    m->insn_limit = g_max;
    /* --stop-on selects the channels; --idle and --stop-port only tune them.
     * Disarming a channel zeroes the field its run-loop test reads, so a
     * disarmed channel costs nothing and cannot fire. */
    bool on_idle, on_port, on_break;
    if (!strcmp(g_stopon,"none"))       on_idle = on_port = on_break = false;
    else if (!strcmp(g_stopon,"all"))   on_idle = on_port = on_break = true;
    else {
        on_idle = strstr(g_stopon,"idle") != NULL || idle_asked;
        on_port = strstr(g_stopon,"port") != NULL;
        on_break = strstr(g_stopon,"break") != NULL;
        if (!on_idle && !on_port && !on_break) {
            fprintf(stderr,"--stop-on: expected a comma-separated list of "
                           "idle, port and/or break, or all, or none\n");
            return 2;
        }
    }
    dbg_stop_at_break(on_break);
    m->idle_quiet = on_idle ? g_idle : 0;
    m->stop_port  = on_port ? (uint16_t)g_stopport : 0;
    m->inq_gateoff = -1;                 /* prompt gate applies throughout by default */
    /* An empty --input-mark is no mark: it would otherwise match instantly and
     * read as "wait for nothing" rather than as the mistake it is. */
    m->inq_mark = (g_inmark && *g_inmark) ? g_inmark : NULL;
    if (g_input) {                       /* queue scripted serial input (\r \n \t \\ escapes) */
        bool now = false;                /* \i marks the NEXT byte as type-ahead */
        for (const char *s=g_input; *s && m->inq_len < (int)sizeof m->inq; s++) {
            uint8_t c = (uint8_t)*s;
            if (c=='\\' && s[1]) {
                s++;
                /* \g queues no byte: it marks where the prompt gate stops
                 * applying, so a script can set a curses program up with the
                 * gate on and then feed it keystrokes with the gate off. */
                if (*s=='g') { m->inq_gateoff = m->inq_len; continue; }
                /* \i queues no byte either: it marks the byte AFTER it as
                 * type-ahead -- handed over the moment the receiver is free,
                 * with none of the pacing the other bytes get.  It applies to
                 * one byte, so a script can type ahead into a running program
                 * (a ^S at its output) and then go back to paced delivery for
                 * the command line that follows.  See bus.c's feeder. */
                if (*s=='i') { now = true; continue; }
                c = (*s=='r')?'\r':(*s=='n')?'\n':(*s=='t')?'\t':(uint8_t)*s;
            }
            m->inq_now[m->inq_len] = now; now = false;
            m->inq[m->inq_len++] = c;
        }
    }
    if (wire) {
        if (wire_open(wire, wtrace) != 0) return 1;
        m->wire_on = true;
        m->wire_port_on[SERPORT_MB_A] = true;
        fprintf(stderr,"[c900: SCC channel A (/dev/tty51) on %s]\n", wire);
    }
    /* The LR board is fitted if it was asked for, or implicitly if one of its
     * ports was wired -- wiring a port on a board that is not there would be
     * a silent no-op, and a silent no-op is the failure mode this whole task
     * exists to remove. */
    for (int p = SERPORT_LR_FIRST; p < SCC_PORTS; p++) if (wireN[p]) lr_scc = true;
    m->lr_scc = lr_scc;
    m->rx_overrun_on = rx_overrun;
    for (int p = SERPORT_MB_A; p < SCC_PORTS; p++) {
        if (!wireN[p]) continue;
        if (m->wire_port_on[p]) { fprintf(stderr,"--wire%d: port %d is already wired\n", p, p); return 2; }
        if (wire_open_n(p, wireN[p], wtrace) != 0) return 1;
        m->wire_port_on[p] = true;
        if (p == SERPORT_MB_A) m->wire_on = true;
        fprintf(stderr,"[c900: SCC port %d on %s]\n", p, wireN[p]);
    }
    for (int p = 0; p < SCC_PORTS; p++) if (m->wire_port_on[p]) m->any_wire = true;
    signal(SIGINT, on_sigint);
#ifdef _WIN32
    signal(SIGBREAK, on_sigint);   /* Ctrl-Break always signals regardless of console mode */
#endif
    fprintf(stderr,"[c900: running boot ROM; serial console below. Ctrl-] to quit (Ctrl-C goes to the guest)]\n");
    machine_run(m);
    wire_close();
    /* One line, whatever ended the run: an idle exit or a doorbell is a normal
     * completion and still exits 0, so the reason goes in the text rather than
     * in the status -- a caller that treats a nonzero status as "the emulator
     * died" (the CP/M suite's EMUOK does) must not see one for a run that
     * finished its work.  rtc_report() stays the LAST line on stderr; the
     * verify-rtc targets read it with tail -1. */
    fprintf(stderr,"\n[c900: stopped after %llu instructions%s%s]\n",
            (unsigned long long)m->cpu.insns,
            m->stop_why ? " -- " : "", m->stop_why ? m->stop_why : "");
    rtc_report(m, stderr);
    /* Exhausting --max is a normal exit unless the caller asked to be told
     * apart: with --require-stop it is status 3, so a test can assert that the
     * session ENDED rather than that the emulator merely survived to the
     * budget.  Without the flag it stays 0, because every existing caller
     * treats any nonzero status as "the emulator died". */
    return (require_stop && m->max_reached) ? 3 : 0;
}
