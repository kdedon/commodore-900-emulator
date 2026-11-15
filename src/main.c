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
/* ── scripted input (--input) ───────────────────────────────────────────────
 * The queue grows as the script is appended.  It was a fixed 8192 bytes, and a
 * longer script was silently cut off there: the run stopped mid-command and
 * nothing anywhere said why, which is days of a person's time. */
static bool inq_push(Machine *m, uint8_t c, bool now){
    if (m->inq_len == m->inq_cap) {
        int cap = m->inq_cap ? m->inq_cap * 2 : 8192;
        uint8_t *q = realloc(m->inq, (size_t)cap);
        bool *n = realloc(m->inq_now, (size_t)cap * sizeof *n);
        if (q) m->inq = q;
        if (n) m->inq_now = n;
        if (!q || !n) return false;
        m->inq_cap = cap;
    }
    m->inq_now[m->inq_len] = now;
    m->inq[m->inq_len++] = c;
    return true;
}

/* Queue a script, honouring the \r \n \t \\ \g \i escapes. */
static bool queue_input(Machine *m, const char *text){
    bool now = false;                    /* \i marks the NEXT byte as type-ahead */
    for (const char *s=text; *s; s++) {
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
        if (!inq_push(m, c, now)) return false;
        now = false;
    }
    return true;
}

/* The queue holds whatever it is given, past the old fixed size, and the
 * escapes that queue no byte are counted as the no bytes they are. */
static int inq_selftest(void){
    int ok = 1;
    Machine *m = machine_new();
    char *big = malloc(20001);
    memset(big, 'x', 20000); big[20000] = 0;
    if (!queue_input(m, big)) { printf("input-queue FAIL: 20000 bytes refused\n"); ok = 0; }
    else if (m->inq_len != 20000)
        { printf("input-queue FAIL: queued %d of 20000 bytes\n", m->inq_len); ok = 0; }
    else if (m->inq[19999] != 'x')
        { printf("input-queue FAIL: the last byte is not the one given\n"); ok = 0; }
    free(big);
    m->inq_len = 0; m->inq_gateoff = -1;
    if (!queue_input(m, "a\\r\\g\\ib\\\\")) ok = 0;
    if (ok && (m->inq_len != 4 || m->inq[1] != '\r' || m->inq[3] != '\\' ||
               m->inq_gateoff != 2 || !m->inq_now[2] || m->inq_now[3]))
        { printf("input-queue FAIL: escapes queued %d bytes\n", m->inq_len); ok = 0; }
    if (ok) printf("input queue PASSED (grows past 8192; escapes queue one byte or none)\n");
    free(m->inq); free(m->inq_now); free(m->ram); free(m);
    return ok;
}

int lout_layout_selftest(void);   /* uexec.c */
int park_selftest(void);          /* bus.c */
int stop_mark_selftest(void);     /* bus.c */
int prompt_gate_selftest(void);   /* bus.c */
int keypace_selftest(void);       /* bus.c */

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

/* ── the memory map (run by --selftest) ─────────────────────────────────────
 * What answers the bus is what the ROM's memory march finds, so the fitted
 * range and the video apertures are checked here rather than by booting.  The
 * aperture cases are the interesting ones: the LR board's text RAM is 4 KB
 * repeated through a 64 KB segment, so it answers a write and readback
 * anywhere in it and the march walks through the whole segment. */
static int mem_map_selftest(void){
    int ok = 1;
    #define MFAIL(...) do{ printf("memory-map FAIL: " __VA_ARGS__); ok=0; }while(0)
    Machine *m = machine_new();

    /* The default machine: 1 MB of DRAM, nothing above it but the stack RAM. */
    phys_write16(m, 0x080000, 0xAAAA);
    phys_write16(m, 0x170000, 0xAAAA);
    if (phys_read16(m, 0x080000) != 0xAAAA || phys_read16(m, 0x170000) != 0xAAAA)
        MFAIL("the 1 MB board does not answer\n");
    if (phys_read16(m, 0x180000) != 0xFFFF) MFAIL("segment 18 answers by default\n");
    if (phys_read16(m, 0x370000) != 0xFFFF) MFAIL("text RAM answers with no LR board\n");
    if (phys_read16(m, 0x3E0000) != 0xFFFF) MFAIL("the bitmap answers with no HR board\n");
    phys_write16(m, 0x3F0000, 0x1234);       /* motherboard stack RAM, board or no board */
    if (phys_read16(m, 0x3F0000) != 0x1234) MFAIL("the stack RAM does not answer\n");

    /* Fitted RAM is a range, and nothing above it answers. */
    m->ram_hi = 0x3F;
    phys_write16(m, 0x380000, 0x5555);
    if (phys_read16(m, 0x380000) != 0x5555) MFAIL("fitted segment 38 does not answer\n");
    if (phys_read16(m, 0x400000) != 0xFFFF) MFAIL("segment 40 answers, past the fitted RAM\n");

    /* The LR board's aperture, mirrored: it answers at the segment base, and a
     * write 4 KB further up lands on the same 4 KB of text RAM.  It decodes
     * its segment even where DRAM is configured behind it. */
    m->lr_scc = true;
    phys_write16(m, 0x370000, 0xAAAA);
    if (phys_read16(m, 0x370000) != 0xAAAA) MFAIL("the text RAM does not answer\n");
    phys_write16(m, 0x371000, 0x5555);
    if (phys_read16(m, 0x370000) != 0x5555) MFAIL("the text RAM does not mirror at 4 KB\n");
    phys_write8(m, 0x37F002, 0x42);
    if (phys_read8(m, 0x370002) != 0x42) MFAIL("the text RAM does not mirror to the top\n");
    if (phys_read16(m, 0x370FFE) == 0xFFFF) MFAIL("the last word of the text RAM floats\n");

    /* The HR board's bitmap: only with the board fitted. */
    Machine *h = machine_new();
    h->hr_board = true;
    phys_write16(h, 0x3E0000, 0xAAAA);
    if (phys_read16(h, 0x3E0000) != 0xAAAA) MFAIL("the HR bitmap does not answer\n");

    /* Its mouse block: the strap at 404H with or without a mouse. */
    h->hr_strap = 2;
    if (io_read(h, 0x0404, false, false) != 2) MFAIL("404H does not read the mono strap\n");
    if (io_read(h, 0x0406, false, false) != 0) MFAIL("406H strap bits are not 00\n");
    h->hr_strap = 1;
    if (io_read(h, 0x0404, false, false) != 1) MFAIL("404H does not read the colour strap\n");
    h->mouse_on = true; h->mouse_x = 0x2B7; h->mouse_y = 0x155; h->mouse_btn = 5;
    if (io_read(h, 0x0400, false, false) != 0xA2B7) MFAIL("400H is not X and buttons\n");
    if (io_read(h, 0x0402, false, false) != 0xA155) MFAIL("402H is not Y and buttons\n");
    if (io_read(h, 0x0404, false, false) != 0xA2B5) MFAIL("404H is not X with the strap\n");
    if (io_read(h, 0x0406, false, false) != 0xA154) MFAIL("406H is not Y with 00\n");
    if (ok) printf("memory map PASSED (fitted range, mirrored text RAM, HR bitmap, HR mouse block)\n");
    #undef MFAIL
    free(h->ram); free(h);
    free(m->ram); free(m);
    return ok;
}

/* ── static and dynamic shifts (run by --selftest) ──────────────────────────
 * Each case runs one shift on preset registers.  A static byte shift takes
 * its count from the low byte of the second word, so 00FF and FFFF both
 * shift right by one; word and long forms use the whole word. */
static int shift_selftest(void){
    /* dst is the word register holding the operand (the pair's first for a
     * long); r2 is the count register of the dynamic forms. */
    static const struct { uint16_t op, cnt; int dst; uint32_t init; uint16_t r2; uint32_t want; uint16_t fl; const char *what; } t[] = {
        { 0xB281, 0x00FF, 0, 0x00F8, 0, 0x007C, 0, "srlb rl0,$1" },
        { 0xB281, 0x00FF, 0, 0x0081, 0, 0x0040, F_C, "srlb rl0,$1 (carry)" },
        { 0xB281, 0xFFFF, 0, 0x0081, 0, 0x0040, F_C, "srlb rl0,$1 as FFFF" },
        { 0xB281, 0x0004, 0, 0x0018, 0, 0x0080, F_C|F_S, "sllb rl0,$4" },
        { 0xB281, 0x0008, 0, 0x00FF, 0, 0x0000, F_C|F_Z, "sllb rl0,$8" },
        { 0xB219, 0x00FE, 1, 0x3B00, 0, 0x0E00, F_C, "srab rh1,$2" },
        { 0xB299, 0x0002, 1, 0x0001, 0, 0x0004, 0, "slab rl1,$2" },
        { 0xB289, 0x0001, 0, 0x0040, 0, 0x0080, F_S|F_PV, "slab rl0,$1 (overflow)" },
        { 0xB289, 0x0007, 0, 0x0001, 0, 0x0080, F_S|F_PV, "slab rl0,$7" },
        { 0xB289, 0x00FF, 0, 0x0081, 0, 0x00C0, F_C|F_S, "srab rl0,$1" },
        { 0xB289, 0x00F9, 0, 0x0080, 0, 0x00FF, F_S, "srab rl0,$7" },
        { 0xB2C1, 0x00FC, 4, 0x00A5, 0, 0x000A, 0, "srlb rl4,$4" },
        { 0xB2C1, 0xFFFC, 4, 0x00A5, 0, 0x000A, 0, "srlb rl4,$4 as FFFC" },
        { 0xB2C1, 0x00F8, 4, 0x00A5, 0, 0x0000, F_C|F_Z, "srlb rl4,$8" },
        { 0xB283, 0x0200, 0, 0x0081, 0xFFFF, 0x0040, F_C, "sdlb rl0,r2 (-1)" },
        { 0xB28B, 0x0200, 0, 0x0040, 0x0001, 0x0080, F_S|F_PV, "sdab rl0,r2 (+1)" },
        { 0xB311, 0xFFFA, 1, 0x1111, 0, 0x0044, 0, "srl r1,$6" },
        { 0xB311, 0x0004, 1, 0x1111, 0, 0x1110, F_C, "sll r1,$4" },
        { 0xB36D, 0xFFFD, 6, 0x80000010, 0, 0xF0000002, F_S, "sral rr6,$3" },
        { 0xB365, 0x0010, 6, 0x12355678, 0, 0x56780000, F_C, "slll rr6,$16" },
    };
    int ok = 1;
    Machine *m = machine_new();
    CPU *c = &m->cpu;
    for (size_t i = 0; i < sizeof t / sizeof t[0]; i++) {
        m->ram[0x080000] = (uint8_t)(t[i].op >> 8);  m->ram[0x080001] = (uint8_t)t[i].op;
        m->ram[0x080002] = (uint8_t)(t[i].cnt >> 8); m->ram[0x080003] = (uint8_t)t[i].cnt;
        memset(c->R, 0, sizeof c->R);
        c->R[t[i].dst] = (uint16_t)t[i].init; c->R[2] = t[i].r2;
        if ((t[i].op & 0xFF04) == 0xB304) { c->R[t[i].dst] = (uint16_t)(t[i].init >> 16); c->R[t[i].dst+1] = (uint16_t)t[i].init; }
        c->fcw = FCW_SEG | FCW_SN; c->pc = 0x080000; c->halted = false; c->irq_req = 0;
        cpu_step(c);
        uint32_t got = ((t[i].op & 0xFF04) == 0xB304) ? (uint32_t)c->R[t[i].dst] << 16 | c->R[t[i].dst+1] : c->R[t[i].dst];
        uint16_t fl = c->fcw & (F_C | F_Z | F_S | F_PV);
        if (got != t[i].want || fl != t[i].fl) {
            printf("shift FAIL: %s gave %X flags %02X, expected %X flags %02X\n",
                   t[i].what, (unsigned)got, fl, (unsigned)t[i].want, t[i].fl);
            ok = 0;
        }
    }
    if (ok) printf("shifts PASSED (%d cases)\n", (int)(sizeof t / sizeof t[0]));
    free(m->ram); free(m);
    return ok;
}

/* ── the LC card (run by --selftest) ───────────────────────────────────────── */
static void lc_out(Machine *m, uint16_t port, uint8_t v){ io_write(m, port, (uint16_t)v << 8, true, false); }
static uint8_t lc_in(Machine *m, uint16_t port){ return (uint8_t)(io_read(m, port, true, false) >> 8); }
static void lc_reg(Machine *m, int r, uint8_t v){ lc_out(m, 0x0181, (uint8_t)r); lc_out(m, 0x0183, v); }
static uint8_t lc_get(Machine *m, int r){ lc_out(m, 0x0181, (uint8_t)r); return lc_in(m, 0x0183); }

static int lc_selftest(void){
    int ok = 1;
    #define LFAIL(...) do { printf("LC card FAIL: " __VA_ARGS__); ok = 0; } while (0)
    Machine *m = machine_new();
    if (lc_in(m, 0x01FF) != 0) LFAIL("0x01FF answers with no card\n");
    lc_out(m, 0x0181, 18);
    if (lc_in(m, 0x0181) != 0) LFAIL("0x0181 answers with no card\n");

    m->lc_card = true;
    if (lc_in(m, 0x01FF) != 0xC1) LFAIL("ID reads %02X\n", lc_in(m, 0x01FF));
    if (!(lc_in(m, 0x0181) & 0x80)) LFAIL("status not ready\n");
    lc_reg(m, 28, 0x10);
    if (lc_get(m, 28) != 0x1F) LFAIL("R28 reads %02X\n", lc_get(m, 28));
    if (lc_get(m, 40) != 0xFF) LFAIL("a missing register does not read FF\n");

    /* "HI" at 0x1000, then a fill of 78 blanks and a copy to the next row */
    lc_reg(m, 12, 0x10); lc_reg(m, 13, 0x00); lc_reg(m, 1, 80); lc_reg(m, 27, 0);
    lc_reg(m, 18, 0x10); lc_reg(m, 19, 0x00);
    lc_reg(m, 31, 'H'); lc_reg(m, 31, 'I');
    if (lc_get(m, 19) != 0x02) LFAIL("R31 does not advance the update address\n");
    lc_reg(m, 31, ' '); lc_reg(m, 24, 0x00); lc_reg(m, 30, 77);
    if (lc_get(m, 18) != 0x10 || lc_get(m, 19) != 80) LFAIL("fill ends at %02X%02X\n", lc_get(m, 18), lc_get(m, 19));
    lc_reg(m, 32, 0x10); lc_reg(m, 33, 0x00); lc_reg(m, 24, 0x80); lc_reg(m, 30, 80);
    if (lc_get(m, 32) != 0x10 || lc_get(m, 33) != 80) LFAIL("copy source ends at %02X%02X\n", lc_get(m, 32), lc_get(m, 33));
    lc_reg(m, 18, 0x10); lc_reg(m, 19, 80);
    if (lc_get(m, 31) != 'H' || lc_get(m, 31) != 'I' || lc_get(m, 31) != ' ') LFAIL("copy or read-back is wrong\n");

    char buf[LC_SCREEN_TEXT];
    lc_screen_text(m, buf);
    if (strncmp(buf, "HI\nHI\n\n", 7) != 0) LFAIL("screen dump starts \"%.12s\"\n", buf);

    /* a 16x4 bitmap at 0x2000 (two character rows of two scan lines), its
     * attributes at 0x3000 */
    static const uint8_t bits[] = { 0x80,0x01, 0xFF,0x00, 0x0F,0xF0, 0x00,0x81 };
    lc_reg(m, 1, 2); lc_reg(m, 6, 2); lc_reg(m, 9, 1);
    lc_reg(m, 12, 0x20); lc_reg(m, 13, 0x00); lc_reg(m, 18, 0x20); lc_reg(m, 19, 0x00);
    for (int i = 0; i < 8; i++) lc_reg(m, 31, bits[i]);
    lc_reg(m, 20, 0x30); lc_reg(m, 21, 0x00); lc_reg(m, 18, 0x30); lc_reg(m, 19, 0x00);
    lc_reg(m, 31, 0x0C); lc_reg(m, 31, 0x0F); lc_reg(m, 31, 0x02); lc_reg(m, 31, 0x08);
    lc_reg(m, 26, 0x01); lc_reg(m, 24, 0x00);
    unsigned char img[16 * 4 * 3 + 32];
    size_t n;
    FILE *f = tmpfile();
    lc_reg(m, 25, 0x00);
    if (f && lc_bitmap_dump(m, f)) LFAIL("bitmap dump in text mode\n");
    lc_reg(m, 25, 0x80);
    if (f && lc_bitmap_dump(m, f)) {
        rewind(f); n = fread(img, 1, sizeof img, f);
        if (n != 16 || memcmp(img, "P4\n16 4\n", 8) || memcmp(img + 8, bits, 8)) LFAIL("mono bitmap is wrong\n");
    } else LFAIL("no mono bitmap\n");
    if (f) fclose(f);
    f = tmpfile();
    lc_reg(m, 24, 0x40);
    if (f && lc_bitmap_dump(m, f)) {
        rewind(f); n = fread(img, 1, sizeof img, f);
        if (n != 16 || img[8] != 0x7F || img[15] != 0x7E) LFAIL("reverse bitmap is wrong\n");
    } else LFAIL("no reverse bitmap\n");
    if (f) fclose(f);
    f = tmpfile();
    lc_reg(m, 24, 0x00); lc_reg(m, 25, 0xC0);
    if (f && lc_bitmap_dump(m, f)) {
        static const uint8_t yel[3] = {0xAA,0xAA,0x00}, grey[3] = {0x55,0x55,0x55},
                             blu[3] = {0x00,0x00,0xAA}, red[3] = {0xAA,0x00,0x00};
        rewind(f); n = fread(img, 1, sizeof img, f);
        const uint8_t *p = img + 12;           /* past "P6\n16 4\n255\n" */
        if (n != 12 + 16 * 4 * 3 || memcmp(img, "P6\n16 4\n255\n", 12)
            || memcmp(p, yel, 3) || memcmp(p + 3, grey, 3)
            || memcmp(p + 15 * 3, (const uint8_t[]){0xFF,0xFF,0xFF}, 3)
            || memcmp(p + (3 * 16 + 8) * 3, red, 3) || memcmp(p + (2 * 16) * 3, grey, 3)
            || memcmp(p + (2 * 16 + 4) * 3, blu, 3))
            LFAIL("colour bitmap is wrong\n");
    } else LFAIL("no colour bitmap\n");
    if (f) fclose(f);
    if (ok) printf("LC card PASSED (ID, registers, R31, fill, copy, screen and bitmap dumps)\n");
    #undef LFAIL
    free(m->ram); free(m);
    return ok;
}

/* Run guest code at 08:0000 (and a Port A handler at 08:0100) against the
 * scripted keys, which must each land once in the buffer at 08:9000. */
static int kbd_run(const char *name, const uint16_t *prog, int nprog,
                   const uint16_t *isr, int nisr){
    static const uint8_t keys[] = { 0x14, 0x12, 0x1F, 0x14, 0x12, 0x13, 0x1C };   /* tester\r */
    static const uint16_t rom[] = {
        0x0000, 0xC000, 0x0800, 0x0000,                 /* reset: FCW, PC 08:0000 */
    };
    int n = (int)sizeof keys, ok = 1;
    Machine *m = machine_new();
    #define PUTW(b, a, v) do { (b)[(a)] = (uint8_t)((v) >> 8); (b)[(a) + 1] = (uint8_t)(v); } while (0)
    for (int i = 0; i < 4; i++) PUTW(m->rom, 2 * i, rom[i]);
    PUTW(m->rom, 0x3A, 0xC000);                         /* VI: FCW, interrupts off */
    PUTW(m->rom, 0x4C, 0x0800); PUTW(m->rom, 0x4E, 0x0100);   /* vector 8: 08:0100 */
    for (int i = 0; i < nprog; i++) PUTW(m->ram, 0x080000 + 2 * i, prog[i]);
    for (int i = 0; i < nisr; i++) PUTW(m->ram, 0x080100 + 2 * i, isr[i]);
    #undef PUTW
    m->hwev = calloc((size_t)n, sizeof *m->hwev);
    for (int i = 0; i < n; i++) m->hwev[i] = (struct hwev){ 2000ull * (i + 1), keys[i], 0, 0, 0 };
    m->nhwev = n; m->hw_armed = true;
    m->insn_limit = 2000ull * (n + 2);
    machine_run(m);
    int got = (m->ram[0x088F00] << 8) | m->ram[0x088F01];
    if (got != n) { printf("%s FAIL: %d keys delivered, %d typed\n", name, got, n); ok = 0; }
    for (int i = 0; i < n && i < got && ok; i++)
        if ((m->ram[0x089000 + i] & 0x7F) != keys[i]) {
            printf("%s FAIL: key %d read %02X, typed %02X\n", name, i, m->ram[0x089000 + i], keys[i]);
            ok = 0;
        }
    if (ok) printf("%s PASSED (%d scripted keys, each read once)\n", name, n);
    free(m->hwev); free(m->ram); free(m);
    return ok;
}

/* The handler, like Coherent's, takes the key and echoes it with interrupts
 * on before it clears IP and strobes PC3. */
static int kbd_selftest(void){
    static const uint16_t prog[] = {
        0x140E, 0x0800, 0x8000,     /* LDL RR14,#08:8000 */
        0xC880, 0x3A86, 0x0001,     /* MICR: MIE */
        0xC804, 0x3A86, 0x0003,     /* MCC: Port A enable */
        0xC808, 0x3A86, 0x0005,     /* Port A vector 8 */
        0xC8C0, 0x3A86, 0x0011,     /* Port A: set IE */
        0x7C05,                     /* EI VI */
        0xE8FF,                     /* JR $ */
    };
    static const uint16_t isr[] = {
        0x3A84, 0x001B,             /* INB RL0,PADATA */
        0x1402, 0x0800, 0x8F00,     /* LDL RR2,#count */
        0x2124,                     /* LD R4,@RR2 */
        0x2920,                     /* INC @RR2 */
        0x1406, 0x0800, 0x9000,     /* LDL RR6,#buffer */
        0x8147,                     /* ADD R7,R4 */
        0x2E68,                     /* LDB @RR6,RL0 */
        0x1408, 0x0800, 0x8F02,     /* LDL RR8,#busy */
        0x0D84,                     /* TEST @RR8 */
        0xEE07,                     /* JR NZ,done */
        0x2980,                     /* INC @RR8 */
        0x7C05,                     /* EI VI: echo with interrupts on */
        0x2105, 0x00C8,             /* LD R5,#200 */
        0xF581,                     /* DJNZ R5,$ */
        0x7C01,                     /* DI VI */
        0x2B80,                     /* DEC @RR8 */
        0xC820, 0x3A86, 0x0011,     /* done: Port A clear IP and IUS */
        0xC870, 0x3A86, 0x001F,     /* PC3 Low: clear the key */
        0xC878, 0x3A86, 0x001F,     /* PC3 High */
        0x7B00,                     /* IRET */
    };
    return kbd_run("keyboard", prog, (int)(sizeof prog / 2), isr, (int)(sizeof isr / 2));
}

/* A driver that polls DAV with interrupts off and never touches IP. */
static int kbd_poll_selftest(void){
    static const uint16_t prog[] = {
        0xC800, 0x3A86, 0x0001,     /* MICR: out of reset */
        0x3A84, 0x001B,             /* wait: INB RL0,PADATA */
        0x8C84,                     /* TESTB RL0 */
        0xEDFC,                     /* JR PL,wait: no DAV */
        0x1402, 0x0800, 0x8F00,     /* LDL RR2,#count */
        0x2124,                     /* LD R4,@RR2 */
        0x2920,                     /* INC @RR2 */
        0x1406, 0x0800, 0x9000,     /* LDL RR6,#buffer */
        0x8147,                     /* ADD R7,R4 */
        0x2E68,                     /* LDB @RR6,RL0 */
        0xC870, 0x3A86, 0x001F,     /* PC3 Low: clear the key */
        0xC878, 0x3A86, 0x001F,     /* PC3 High */
        0xE8EB,                     /* JR wait */
    };
    return kbd_run("polled keyboard", prog, (int)(sizeof prog / 2), NULL, 0);
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

    /* The park watch: a halted guest a periodic tick keeps waking is ended,
     * and a guest that is getting somewhere never is. */
    if (!park_selftest()) ok = 0;

    /* The stop mark: ends on the text the guest prints, and on nothing else. */
    if (!stop_mark_selftest()) ok = 0;
    if (!keypace_selftest()) ok = 0;

    /* The prompt gate: scripted input is released by a prompt, not by a '#'
     * or '>' anywhere in the guest's output. */
    if (!prompt_gate_selftest()) ok = 0;

    /* The scripted-input queue: no silent truncation. */
    if (!inq_selftest()) ok = 0;

    /* What answers the bus: fitted RAM and the video apertures. */
    if (!mem_map_selftest()) ok = 0;

    /* Shift counts, byte forms in particular. */
    if (!shift_selftest()) ok = 0;

    if (!lc_selftest()) ok = 0;

    if (!kbd_selftest()) ok = 0;
    if (!kbd_poll_selftest()) ok = 0;

    free(m->ram); free(m);
    return ok ? 0 : 1;
}

/* Print command-line usage. */
static void usage(FILE *out, const char *prog){
    fprintf(out,
        "Usage: %s [options]\n"
        "  --firmware=DIR   ROM directory with bios_h.bin + bios_l.bin (default ../rom)\n"
        "  --disk=FILE      raw hard-disk image (default ../disk/hdd.bin; required)\n"
        "  --disk1=FILE     second hard-disk image (optional)\n"
        "  --disk-geom=C,H,S, --disk1-geom=C,H,S\n"
        "                   a drive's own geometry: sectors sit by it, and the\n"
        "                   card's one WDSDP set must match it (default: flat)\n"
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
        "  --key-pace=N     one key at a time: hold each scripted byte until the\n"
        "                   guest has ANSWERED the byte before it on the console\n"
        "                   and the console has then been quiet for N\n"
        "                   instructions.  Off at 0 (the default), which is the\n"
        "                   pacing above: a byte on silence alone.  Silence cannot\n"
        "                   tell a program that is thinking from one that is\n"
        "                   waiting, so a full-screen program that takes a second\n"
        "                   of emulated time per command collects the whole script\n"
        "                   into its type-ahead and throws it away.  Its own output\n"
        "                   says it acted, and waiting for that costs exactly as\n"
        "                   long as the program takes -- a key needing a billion\n"
        "                   instructions and a key needing a thousand both go in as\n"
        "                   soon as they can, which no fixed count does.  The first\n"
        "                   byte has nothing to answer and \\i bytes have asked not\n"
        "                   to wait; neither is held\n"
        "  --key-react=N    console bytes that count as an answer (default 1), for\n"
        "                   a guest that echoes the key itself before acting on it\n"
        "  --key-deadline=N instructions an unanswered key waits before the next\n"
        "                   byte goes in on the quiet window alone (default 0:\n"
        "                   wait forever, so a lost reaction stalls the script\n"
        "                   rather than silently going back to losing keys)\n"
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
        "                   \"idle\", \"port\" and \"park\" (the default is\n"
        "                   \"port,park\"), \"break\", \"mark\", \"all\" or \"none\".\n"
        "                   idle: for test harnesses, OFF unless asked for.  With\n"
        "                     --input, stop once every scripted byte has been fed\n"
        "                     and the guest is idle at a prompt: a prompt character\n"
        "                     ('#' or '>') printed last, console quiet for --idle\n"
        "                     instructions, and the guest spinning on the receiver\n"
        "                   port: stop when the guest writes the word 0xC900 to the\n"
        "                     --stop-port I/O port -- an explicit \"I am finished\"\n"
        "                     from the guest, so it cannot fire on its own\n"
        "                   mark: stop when the guest PRINTS --stop-mark. Armed by\n"
        "                     giving that text; naming it here without one is an\n"
        "                     error rather than a channel that cannot fire\n"
        "                   break: stop once a --break has recorded its last hit,\n"
        "                     with that instruction NOT yet executed\n"
        "                   park: stop when the guest is halted, woken only by a\n"
        "                     periodic interrupt, and doing nothing when woken.\n"
        "                     An IDLE guest wears that signature too, so a script\n"
        "                     that sleeps for longer than the watch's budget is\n"
        "                     called parked while it is working: name your own\n"
        "                     channels to leave it off\n"
        "  --idle=N         instructions of console silence the idle channel waits\n"
        "                   for, and arms it (default 40000000)\n"
        "  --stop-port=P    the port channel's I/O port (default 0x0FFE, which\n"
        "                   nothing on the machine answers)\n"
        "  --stop-mark=TEXT end the run when the guest PRINTS TEXT on ANY serial\n"
        "                   channel, the console included (each channel matched on\n"
        "                   its own output alone -- two channels never combine).\n"
        "                   The guest picks the moment, as with the doorbell, so\n"
        "                   nothing is truncated and nothing is waited out -- but\n"
        "                   it announces it in output rather than on an I/O port,\n"
        "                   so it needs no program on the guest's disk and no\n"
        "                   privileged OUT. A harness that already echoes a\n"
        "                   finished marker has one for free\n"
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
        "  --ram=LO-HI      fitted motherboard DRAM, as inclusive 64 KB segment\n"
        "                   numbers in hex (default 08-17, the 1 MB board).  The\n"
        "                   ROM sizes memory by marching up from segment 08, so it\n"
        "                   reports whatever is fitted from there -- and walks into\n"
        "                   any video aperture that adjoins it, as the hardware does\n"
        "  --hr             fit the HR board, whose bitmap answers at segments\n"
        "                   3E-3F.  3F answers either way: with no HR board it is\n"
        "                   the motherboard's stack RAM\n"
        "  --hr-bitmap=FILE fit the HR board and write its 128 KB bitmap to FILE\n"
        "                   as a 1024x1024 PBM when the run ends (a 1 bit is white)\n"
        "  --lc             fit the LC card: ID 0xC1 at port 0x01FF and an 8563\n"
        "                   VDC at 0x0181/0x0183 with 64 KB of RAM, no picture\n"
        "  --lc-slots[=N]   the LC card, with each RAM access timed by the chip's\n"
        "                   free cycles; ready stays clear until it is done.  N is\n"
        "                   the CPU's instructions a second (default 1500000)\n"
        "  --lc-screen=FILE fit the LC card and write its 80x25 text screen to\n"
        "                   FILE when the run ends (- for stdout)\n"
        "  --key=T:HH       press keyboard scancode HH (hex; bit 7 = release) T loop\n"
        "                   ticks after --hw-mark is printed (after the start\n"
        "                   without one), once the last key has been taken\n"
        "  --mouse=T:DX:DY[:B]  move the mouse counters by DX,DY and set buttons\n"
        "                   B at tick T; the counters then answer at 0x0400/0x0402\n"
        "  --hw-mark=TEXT   console text that starts the --key/--mouse clock\n"
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
    const char *disk1 = NULL, *geom0 = NULL, *geom1 = NULL;
    const char *rtcseed = "host", *wire = NULL, *g_inmark = NULL;
    const char *wireN[SCC_PORTS] = {0};   /* --wire1 .. --wire5, by port number */
    bool lr_scc = false, rx_overrun = false, hr_board = false, lc_card = false, lc_slots = false;
    const char *lc_screen = NULL, *lc_bitmap = NULL, *hr_bitmap = NULL;
    uint8_t ram_lo = 0x08, ram_hi = 0x17;   /* the 1 MB motherboard, as fitted */
    unsigned long long rtc_ips = 0, slots_ips = 1500000;
    bool trace = false, dosel = false, wtrace = false;
    unsigned long long g_max = 0;
    /* the same silence the scripted-input pacing uses for its longest wait */
    unsigned long long g_idle = 40000000ull;
    unsigned long long g_keypace = 0, g_keydead = 0;   /* --key-pace, off unless asked for */
    unsigned long g_keyreact = 1;
    unsigned long g_stopport = 0x0FFE;   /* see bus.c io_write: unclaimed I/O space */
    /* Idle is NOT armed by default: an emulator left alone must still be
     * running when its owner returns, however long the guest stays silent.
     * The port channel is, since only the guest itself can fire it. */
    const char *g_stopon = "port,park";
    const char *g_stopmark = NULL;   /* --stop-mark: guest-announced end of run */
    bool idle_asked = false;             /* --idle=N given: tuning it asks for it */
    bool require_stop = false;
    const char *g_hwmark = NULL;
    struct hwev *hwev = NULL;
    int nhwev = 0;
    bool mouse_on = false;
    for (int i=1;i<argc;i++){
        const char *v;
        if      ((v = opt_value(argv,argc,&i,"--rtc-ips"))) rtc_ips = strtoull(v,0,0);
        else if ((v = opt_value(argv,argc,&i,"--rtc")))     rtcseed = v;
        else if ((v = opt_value(argv,argc,&i,"--firmware"))) fw = v;
        else if ((v = opt_value(argv,argc,&i,"--disk")))     disk = v;
        else if ((v = opt_value(argv,argc,&i,"--disk1")))    disk1 = v;
        else if ((v = opt_value(argv,argc,&i,"--disk-geom")))  geom0 = v;
        else if ((v = opt_value(argv,argc,&i,"--disk1-geom"))) geom1 = v;
        else if ((v = opt_value(argv,argc,&i,"--floppy")))   floppy = v;
        else if ((v = opt_value(argv,argc,&i,"--max")))      g_max = strtoull(v,0,0);
        else if ((v = opt_value(argv,argc,&i,"--input-mark"))) g_inmark = v;
        else if ((v = opt_value(argv,argc,&i,"--input")))    g_input = v;
        else if ((v = opt_value(argv,argc,&i,"--key-pace")))  g_keypace = strtoull(v,0,0);
        else if ((v = opt_value(argv,argc,&i,"--key-react"))) g_keyreact = strtoul(v,0,0);
        else if ((v = opt_value(argv,argc,&i,"--key-deadline"))) g_keydead = strtoull(v,0,0);
        else if ((v = opt_value(argv,argc,&i,"--idle")))   { g_idle = strtoull(v,0,0); idle_asked = true; }
        else if ((v = opt_value(argv,argc,&i,"--stop-port"))) g_stopport = strtoul(v,0,0);
        else if ((v = opt_value(argv,argc,&i,"--stop-on")))  g_stopon = v;
        else if ((v = opt_value(argv,argc,&i,"--stop-mark"))) g_stopmark = v;
        else if ((v = opt_value(argv,argc,&i,"--break"))) {
            if (dbg_add_break(v)) { fprintf(stderr,"--break: expected SEG:OFF[/N] in hex, e.g. 32:3EAC or B200:3EAC/2\n"); return 2; }
        }
        else if ((v = opt_value(argv,argc,&i,"--dump"))) {
            if (dbg_add_dump(v)) { fprintf(stderr,"--dump: expected SEG:OFF+LEN or rrN:DISP+LEN, e.g. 0F:EF80+32 or rr14:0+32\n"); return 2; }
        }
        else if (!strcmp(argv[i],"--require-stop")) require_stop = true;
        else if ((v = opt_value(argv,argc,&i,"--wire")))     wire = v;
        else if (!strcmp(argv[i],"--wire-trace")) wtrace = true;
        else if ((v = opt_value(argv,argc,&i,"--ram"))) {
            char *end;
            unsigned long lo = strtoul(v,&end,16), hi;
            if (*end != '-' || (hi = strtoul(end+1,&end,16), *end) || lo > hi || hi > 0xFF) {
                fprintf(stderr,"--ram: expected LO-HI as hex segment numbers, e.g. 08-3D\n");
                return 2;
            }
            ram_lo = (uint8_t)lo; ram_hi = (uint8_t)hi;
        }
        else if ((v = opt_value(argv,argc,&i,"--hw-mark"))) g_hwmark = v;
        else if ((v = opt_value(argv,argc,&i,"--key")) || (v = opt_value(argv,argc,&i,"--mouse"))) {
            struct hwev e = { 0, -1, 0, 0, 0 };
            bool bad = strchr(v, ':') == strrchr(v, ':')
                ? sscanf(v, "%llu:%x", &e.at, &e.key) != 2
                : sscanf(v, "%llu:%d:%d:%d", &e.at, &e.dx, &e.dy, &e.btn) < 3;
            if (bad) { fprintf(stderr,"--key=TICKS:HEXCODE or --mouse=TICKS:DX:DY[:BUTTONS]\n"); return 2; }
            if (e.key < 0) mouse_on = true;
            hwev = realloc(hwev, (size_t)(nhwev + 1) * sizeof *hwev);
            hwev[nhwev++] = e;
        }
        else if (!strcmp(argv[i],"--hr")) hr_board = true;
        else if ((v = opt_value(argv,argc,&i,"--hr-bitmap"))) { hr_bitmap = v; hr_board = true; }
        else if ((v = opt_value(argv,argc,&i,"--lc-screen"))) { lc_screen = v; lc_card = true; }
        else if ((v = opt_value(argv,argc,&i,"--lc-bitmap"))) { lc_bitmap = v; lc_card = true; }
        else if (!strcmp(argv[i],"--lc")) lc_card = true;
        else if (!strcmp(argv[i],"--lc-slots")) lc_card = lc_slots = true;
        else if ((v = opt_value(argv,argc,&i,"--lc-slots"))) { lc_card = lc_slots = true; slots_ips = strtoull(v,0,0); }
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
    if (disk1 && machine_attach_disk1(m, disk1) != 0) {
        fprintf(stderr,"failed to open hard-disk image %s\n", disk1);
        return 1;
    }
    if ((geom0 && disk_set_geom(&m->disk, geom0) != 0) ||
        (geom1 && disk_set_geom(&m->disk1, geom1) != 0)) {
        fprintf(stderr,"bad --disk-geom/--disk1-geom: want C,H,S\n");
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
    bool on_idle, on_port, on_break, on_mark, on_park;
    /* An empty --stop-mark is no mark: it would match instantly and end the run
     * before the guest had printed anything, which reads as "stop at once"
     * rather than as the mistake it is. */
    bool mark_asked = g_stopmark && *g_stopmark;
    if (!strcmp(g_stopon,"none"))       on_idle = on_port = on_break = on_mark = on_park = false;
    else if (!strcmp(g_stopon,"all"))   on_idle = on_port = on_break = on_mark = on_park = true;
    else {
        on_idle = strstr(g_stopon,"idle") != NULL || idle_asked;
        on_port = strstr(g_stopon,"port") != NULL;
        on_break = strstr(g_stopon,"break") != NULL;
        on_park = strstr(g_stopon,"park") != NULL;
        /* Naming a mark arms its channel, as --idle arms idle: a caller who
         * passed the text has said what it is for, and making them repeat it
         * in --stop-on only creates a way to pass a mark that cannot fire. */
        on_mark = strstr(g_stopon,"mark") != NULL || mark_asked;
        if (!on_idle && !on_port && !on_break && !on_mark && !on_park) {
            fprintf(stderr,"--stop-on: expected a comma-separated list of "
                           "idle, port, break, park and/or mark, or all, or none\n");
            return 2;
        }
    }
    if (on_mark && !mark_asked && strstr(g_stopon,"mark")) {
        fprintf(stderr,"--stop-on=mark: no --stop-mark text to watch for\n");
        return 2;
    }
    dbg_stop_at_break(on_break);
    m->idle_quiet = on_idle ? g_idle : 0;
    m->stop_port  = on_port ? (uint16_t)g_stopport : 0;
    m->stop_mark  = (on_mark && mark_asked) ? g_stopmark : NULL;
    m->park_watch = on_park;
    m->hwev = hwev; m->nhwev = nhwev; m->mouse_on = mouse_on;
    m->hw_mark = (g_hwmark && *g_hwmark) ? g_hwmark : NULL;
    m->hw_armed = m->hw_mark == NULL;
    m->key_quiet    = g_keypace;
    m->key_react    = g_keyreact ? (uint32_t)g_keyreact : 1;
    m->key_deadline = g_keydead;
    m->inq_gateoff = -1;                 /* prompt gate applies throughout by default */
    /* An empty --input-mark is no mark: it would otherwise match instantly and
     * read as "wait for nothing" rather than as the mistake it is. */
    m->inq_mark = (g_inmark && *g_inmark) ? g_inmark : NULL;
    if (g_input && !queue_input(m, g_input)) {
        fprintf(stderr,"--input: out of memory queueing the script\n");
        return 2;
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
    m->ram_lo = ram_lo; m->ram_hi = ram_hi;
    m->hr_board = hr_board;
    {   /* C900_HRCONF=1 fits the colour strap */
        const char *e = getenv("C900_HRCONF");
        m->hr_strap = e ? (uint8_t)(atoi(e) & 3) : 2;
    }
    m->lc_card = lc_card;
    m->vdc.slots = lc_slots;
    m->vdc.slots_ips = slots_ips ? slots_ips : 1500000;
    { extern unsigned cyc_mem, cyc_io;
      if (getenv("C900_CYC_MEM")) cyc_mem = (unsigned)atoi(getenv("C900_CYC_MEM"));
      if (getenv("C900_CYC_IO")) cyc_io = (unsigned)atoi(getenv("C900_CYC_IO")); }
    if (getenv("C900_CPU_HZ")) m->vdc.hz = strtoull(getenv("C900_CPU_HZ"), 0, 0);
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
    if (lc_slots) lc_slot_report(m, stderr);
    if (lc_screen) {
        FILE *f = strcmp(lc_screen, "-") ? fopen(lc_screen, "w") : stdout;
        if (f) { lc_screen_dump(m, f); if (f != stdout) fclose(f); }
        else fprintf(stderr, "--lc-screen: cannot write %s\n", lc_screen);
    }
    if (hr_bitmap) {
        FILE *f = strcmp(hr_bitmap, "-") ? fopen(hr_bitmap, "wb") : stdout;
        if (!f) fprintf(stderr, "--hr-bitmap: cannot write %s\n", hr_bitmap);
        else {
            fprintf(f, "P4\n1024 1024\n");
            for (uint32_t a = 0x3E0000; a < 0x400000; a += 2) {
                uint16_t w = (uint16_t)~phys_read16(m, a);
                putc(w >> 8, f);
                putc(w & 0xFF, f);
            }
            if (f != stdout) fclose(f);
        }
    }
    if (lc_bitmap) {
        FILE *f = strcmp(lc_bitmap, "-") ? fopen(lc_bitmap, "wb") : stdout;
        if (!f) fprintf(stderr, "--lc-bitmap: cannot write %s\n", lc_bitmap);
        else {
            if (!lc_bitmap_dump(m, f)) fprintf(stderr, "--lc-bitmap: the LC card is not in bitmap mode\n");
            if (f != stdout) fclose(f);
        }
    }
    /* One line, whatever ended the run: an idle exit or a doorbell is a normal
     * completion and still exits 0, so the reason goes in the text rather than
     * in the status -- a caller that treats a nonzero status as "the emulator
     * died" (the CP/M suite's EMUOK does) must not see one for a run that
     * finished its work.  rtc_report() stays the LAST line on stderr; the
     * verify-rtc targets read it with tail -1. */
    if (m->sdp_count)
        fprintf(stderr,"\n[hdc: %lu WDSDP]", m->sdp_count);
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
