/* bus.c — C900 memory map, I/O dispatch, and peripherals for the classic
 * emulator: ROM/RAM, Z8010 MMU (special I/O), Z8030 SCC serial console
 * (redirected to the host terminal), Z8036 CIO stubs + 100 Hz timer, the
 * PDMAC disk doorbell + WD2010 command-block processing (DMA memcpy), and
 * interrupt aggregation. */
#include "emu.h"
#include <stdlib.h>
#include <string.h>

/* ─────────────── physical memory ───────────────
 * Only the segments that actually have a responder on the C900 answer the
 * bus; everything else floats (open bus → reads 0xFF, writes ignored). This
 * matters for boot: the BIOS probes the video board's VRAM, and when no card
 * responds there it falls back to the serial console for I/O. Backing every
 * segment with RAM would make that probe falsely succeed.
 * On a machine with no graphics card only two things respond:
 *   0x00        boot ROM (read-only, 32 KB)
 *   0x08-0x17   1 MB motherboard DRAM (RAM1M)
 * Everything else has no responder — including the graphics-card address
 * space (segs 0x37/0x3A/0x3E/0x3F) and the expansion bus (0x01-0x07,
 * 0x18-0x36). Finding nothing at the video area, the BIOS uses the serial
 * port for console I/O. */
static bool seg_populated(uint32_t a){
    uint32_t seg = (a >> 16) & 0xFF;
    return seg >= 0x08 && seg <= 0x17;             /* 1 MB motherboard DRAM (segs 8-0x17) */
}
uint8_t phys_read8(Machine *m, uint32_t a){
    a &= 0x00FFFFFF;
    if (m->flat_mem) return ((a>>16) <= m->flat_maxseg) ? m->ram[a] : 0xFF;
    if (a < ROM_SIZE) return m->rom[a];
    if (seg_populated(a)) return m->ram[a];
    return 0xFF;                                    /* open bus */
}
/* Debug write-watch (CSIM_WATCH_PHYS="lo-hi" hex, physical): logs every
 * store into [lo,hi] with the writing instruction's PC. Parsed lazily on
 * first store; zero overhead when unset beyond one predictable branch. */
static uint32_t watch_lo, watch_hi; static int watch_on = -1;
static void watch_init(void){
    const char *w = getenv("CSIM_WATCH_PHYS");
    watch_on = 0;
    if (w && sscanf(w, "%x-%x", &watch_lo, &watch_hi) == 2) watch_on = 1;
}
static void watch_hit(Machine *m, uint32_t a, uint32_t v, int bits, const char *who){
    fprintf(stderr, "[watch] %s %06X <- %0*X pc=%02X:%04X op=%04X insns=%llu\n",
            who, a, bits/4, v,
            (m->cpu.instr_start>>16)&0x7F, m->cpu.instr_start&0xFFFF,
            m->cpu.opcode[0], (unsigned long long)m->cpu.insns);
}
void phys_write8(Machine *m, uint32_t a, uint8_t v){
    a &= 0x00FFFFFF;
    if (watch_on < 0) watch_init();
    if (watch_on && a >= watch_lo && a <= watch_hi) watch_hit(m, a, v, 8, "w8 ");
    if (m->flat_mem) { if ((a>>16) <= m->flat_maxseg) m->ram[a] = v; return; }
    if (a < ROM_SIZE) return;                       /* ROM read-only */
    if (seg_populated(a)) m->ram[a] = v;            /* else float — ignore */
}
uint16_t phys_read16(Machine *m, uint32_t a){
    a &= 0x00FFFFFF;
    if (m->flat_mem)
        return ((a>>16) <= m->flat_maxseg) ? (((uint16_t)m->ram[a]<<8) | m->ram[a+1]) : 0xFFFF;
    if (watch_on < 0) watch_init();
    if (watch_on && a+1 >= watch_lo && a <= watch_hi)
        fprintf(stderr, "[watch] r16 %06X -> %04X pc=%02X:%04X op=%04X insns=%llu\n",
                a, (a<ROM_SIZE-1)?(((uint16_t)m->rom[a]<<8)|m->rom[a+1]):(((uint16_t)m->ram[a]<<8)|m->ram[a+1]),
                (m->cpu.instr_start>>16)&0x7F, m->cpu.instr_start&0xFFFF,
                m->cpu.opcode[0], (unsigned long long)m->cpu.insns);
    if (a < ROM_SIZE-1) return ((uint16_t)m->rom[a]<<8) | m->rom[a+1];   /* ROM word */
    uint32_t seg = (a>>16)&0xFF;
    if (seg>=0x08 && seg<=0x17) return ((uint16_t)m->ram[a]<<8) | m->ram[a+1]; /* DRAM word */
    return ((uint16_t)phys_read8(m,a)<<8) | phys_read8(m,a+1);          /* edge / open bus */
}
void phys_write16(Machine *m, uint32_t a, uint16_t v){
    a &= 0x00FFFFFF;
    if (m->flat_mem) { if ((a>>16) <= m->flat_maxseg) { m->ram[a]=(uint8_t)(v>>8); m->ram[a+1]=(uint8_t)v; } return; }
    if (watch_on < 0) watch_init();
    if (watch_on && a+1 >= watch_lo && a <= watch_hi) watch_hit(m, a, v, 16, "w16");
    if (a < ROM_SIZE) return;                                           /* ROM read-only */
    uint32_t seg = (a>>16)&0xFF;
    if (seg>=0x08 && seg<=0x17){ m->ram[a]=(uint8_t)(v>>8); m->ram[a+1]=(uint8_t)v; return; }
    phys_write8(m,a,(uint8_t)(v>>8)); phys_write8(m,a+1,(uint8_t)v);    /* edge / open bus */
}

/* ─────────────── SCC (Z8030) — console on channel B ─────────────── */
/* register byte read/write. reg 0 = control (WR0/RR0), reg 8 = data. */
/* Receive interrupt pending on a channel: a character is waiting and that
 * channel has Rx interrupts enabled (WR1 D4:D3 != 0) with the master interrupt
 * enable set (WR9 D3 MIE, one physical register for both channels). */
/* The SCC channel a SERIAL PORT TABLE port number names.  Ports 2..5 are on
 * the LR board and answer only when that board is fitted; asking for one that
 * is not there gets NULL rather than a channel that quietly exists. */
SCCChan *scc_port(Machine *m, int port){
    switch (port) {
    case SERPORT_CONSOLE: return &m->scc_b;
    case SERPORT_MB_A:    return &m->scc_a;
    default:
        if (port < SERPORT_LR_FIRST || port >= SCC_PORTS) return NULL;
        if (!m->lr_scc) return NULL;
        return &m->lr_scc_chan[port - SERPORT_LR_FIRST];
    }
}

/* (chip, chan) -> channel, for the I/O decode.  chip 0 is the motherboard
 * U74, 1 and 2 are the LR board's U31 and U36; chan 1 is A (address bit 5
 * high), chan 0 is B.  Port numbers follow: chip 0 gives ports 0 and 1, chip
 * 1 ports 2 and 3, chip 2 ports 4 and 5. */
static SCCChan *scc_chan_of(Machine *m, int chip, int chan){
    return scc_port(m, chip * 2 + chan);
}

static bool scc_rx_int(SCCChan *ch){
    if (!(ch->wr[9] & 0x08)) return false;  /* WR9 D3 MIE */
    if (!ch->rx_avail)       return false;
    return (ch->wr[1] & 0x18) != 0;         /* WR1 D4:D3 Rx int mode != disabled */
}
static bool scc_rxb_int(Machine *m){ return scc_rx_int(&m->scc_b); }
static bool scc_rxa_int(Machine *m){ return scc_rx_int(&m->scc_a); }

/* Modified interrupt vector for an Rx-available source (SCC Tech Manual
 * Table 4-7): the source code replaces V3:V1 (status-low, WR9 D4=0) or V6:V4
 * (status-high).  Ch B Rx Available is 010, Ch A Rx Available is 110 — the
 * same order the al(4) driver's setivec table assumes (vec+4 = Ch B receive,
 * vec+12 = Ch A receive). */
static uint8_t scc_vector_chip(Machine *m, int chip, uint8_t src){
    SCCChan *b = scc_chan_of(m, chip, 0);
    if (!b) return 0;
    uint8_t base = b->wr[2];
    if (b->wr[9] & 0x10)                     /* WR9 D4: Status High */
        return (uint8_t)((base & ~0x70) | (src << 4));
    return (uint8_t)((base & ~0x0E) | (src << 1));
}
/* The motherboard chip, which is the only one the pre-existing callers meant. */
static uint8_t scc_vector_src(Machine *m, uint8_t src){
    return scc_vector_chip(m, 0, src);
}
/* The vector a Ch-B RR2 read returns: whichever source is currently pending,
 * or the bare base when none is. */
static uint8_t scc_vector_of(Machine *m, int chip){
    SCCChan *a = scc_chan_of(m, chip, 1), *b = scc_chan_of(m, chip, 0);
    if (!a || !b) return 0;
    if (scc_rx_int(a)) return scc_vector_chip(m, chip, 0x06);
    if (scc_rx_int(b)) return scc_vector_chip(m, chip, 0x02);
    return b->wr[2];
}

/* RR3 (read on channel A) exposes the interrupt-pending bits.  D5 = Ch A Rx IP,
 * D2 = Ch B Rx IP. */
static uint8_t scc_rr3_of(Machine *m, int chip){
    SCCChan *a = scc_chan_of(m, chip, 1), *b = scc_chan_of(m, chip, 0);
    if (!a || !b) return 0;
    return (uint8_t)((scc_rx_int(a) ? 0x20 : 0) | (scc_rx_int(b) ? 0x04 : 0));
}

/* The C900's SCC is a Z8030 (Z-Bus).  Register addressing is a hybrid: the
 * effective register is AD4:AD1 (passed here as addrReg), UNLESS the channel's
 * register pointer (ch->ptr, set via a WR0 write) is non-zero, in which case it
 * overrides for this one access and then auto-resets.  chan=1 → Channel A
 * (AD5 high, scc_a), chan=0 → Channel B (the console, scc_b). */
static uint8_t scc_read(Machine *m, int chip, int chan, int addrReg){
    SCCChan *ch = scc_chan_of(m, chip, chan);
    if (!ch) return 0;
    bool console = (chip == 0 && chan == 0);
    int r = ch->ptr ? ch->ptr : (addrReg & 0x0F);
    ch->ptr = 0;
    switch (r) {
    case 0: {                         /* RR0: Rx avail / Tx empty / DCD / CTS */
        uint8_t v = 0x04 | 0x40 | 0x08 | 0x20;   /* Tx empty, all sent, DCD, CTS */
        if (ch->rx_avail) v |= 0x01;
        /* A run of console RR0 polls finding the receiver empty, with no
         * transmit in between, means the guest is blocked reading the
         * console: a polling console-input loop reads RR0 continuously,
         * while an output path checks it at most once per printed
         * character.  Scripted input and the idle exit both use the streak
         * to tell the two apart. */
        if (console && !ch->rx_avail) {
            m->rx_poll_streak++;
            if (m->rx_poll_streak >= RX_BLOCKED_POLLS) m->guest_polls = true;
        }
        return v;
    }
    case 8:                           /* RR8 = Rx data (reading it clears rx) */
        ch->rx_avail = false;
        if (console) {
            m->rx_poll_streak = 0;
            m->last_tx_insn = m->cpu.insns;   /* console activity: restart the quiet clock */
        }
        return ch->rx_data;
    /* RR1: All Sent, plus the error bits.  D5 is Rx Overrun, latched by a
     * character shifted in over an unread one and cleared only by the Error
     * Reset command -- see scc_rx_deliver.  Without --rx-overrun the bit is
     * never set and this is the constant 0x01 it has always been. */
    case 1: return (uint8_t)(0x01 | (ch->rx_overrun ? 0x20 : 0));
    case 2: return chan == 0 ? scc_vector_of(m, chip) : ch->wr[2];  /* RR2 (Ch B): modified vector */
    case 3: return chan == 1 ? scc_rr3_of(m, chip) : 0;             /* RR3 (Ch A): IP bits */
    default: return ch->wr[r & 0x0F];
    }
}
/* One character of a console-output mark match, returning the new matched
 * length.  A mismatch does not simply restart: it falls back to the longest
 * prefix of the mark that is also a suffix of what has already matched, which
 * is what lets a mark whose own prefix repeats be found at all.  Matching
 * "aab" against "aaab" is the smallest case -- resetting to 0, or to 1 for a
 * character equal to mark[0], walks straight past the match that is there.
 * The fallback is computed from the MARK, so no history of the output has to
 * be kept: what has matched so far is by definition mark[0..pos-1].
 */
static int mark_advance(const char *mark, int pos, uint8_t v){
    for (;;) {
        if ((uint8_t)mark[pos] == v) return pos + 1;
        if (pos == 0) return 0;
        int k = pos - 1;
        while (k > 0 && memcmp(mark, mark + pos - k, (size_t)k) != 0) k--;
        pos = k;
    }
}

/* --stop-mark on one transmitted character.  Every SCC transmit path is
 * watched, not the console alone: the mark is the guest saying "I am done",
 * and a guest that says it on a wired port has said it just as plainly.  The
 * match state is per PORT -- the ports are independent streams, and a single
 * shared position would let a half-printed mark on one port be finished by
 * bytes from another and end the run on text nothing ever printed.  Called
 * AFTER the character has been handed on, so the transcript (or the wire)
 * contains the mark that ended the run. */
static void stop_mark_watch(Machine *m, int port, uint8_t v){
    if (!m->stop_mark) return;
    int *pos = &m->stop_mark_pos[port];
    *pos = mark_advance(m->stop_mark, *pos, v);
    if (m->stop_mark[*pos] == '\0') {
        m->stop = true;
        m->stop_why = "the guest printed the stop mark";
    }
}

static void scc_write(Machine *m, int chip, int chan, int addrReg, uint8_t v){
    SCCChan *ch = scc_chan_of(m, chip, chan);
    if (!ch) return;
    bool console = (chip == 0 && chan == 0);
    int port = chip * 2 + chan;
    int r = ch->ptr ? ch->ptr : (addrReg & 0x0F);
    ch->ptr = 0;
    if (r == 0) {                            /* WR0: command + register pointer */
        uint8_t ptr = v & 0x07;
        uint8_t cmd = (v >> 3) & 0x07;
        ch->ptr = (cmd == 1) ? (ptr | 0x08) : ptr;  /* cmd 1 = Point High (+8) */
        /* Command 6 is Error Reset, which unlatches the RR1 error bits -- the
         * only one of commands 2-7 with state here.  The rest (reset
         * ext/status, arm-rx-int, reset-Tx-IP, reset-highest-IUS) still have
         * none: we fire the Rx interrupt directly off rx_avail and don't
         * track IUS. */
        if (cmd == 6) ch->rx_overrun = false;
        return;
    }
    if (r == 8) {                            /* WR8 = Tx data */
        /* Every port but the console transmits onto its own host endpoint, if
         * one is attached; port 1 through a wired endpoint is byte-for-byte
         * what --wire has always done. */
        if (!console) {
            if (m->wire_port_on[port]) wire_put_char_n(port, v);
            /* A non-console port is watched for the stop mark whether or not a
             * host endpoint is attached: the guest has printed it either way. */
            stop_mark_watch(m, port, v);
        }
        if (console) {
            console_put_char(v);
            m->last_tx_insn = m->cpu.insns;
            m->rx_poll_streak = 0;
            /* --input-mark: watch the guest's own output for the text that
             * releases the scripted type-ahead bytes.  Matching on what is
             * PRINTED is the one synchronisation a test can state exactly --
             * "send this the moment that appears" is what a person at the
             * terminal does, and unlike an instruction count it does not move
             * when the guest is rebuilt.  On a mismatch the match restarts at
             * the longest prefix of the mark that is still matched, so a mark
             * whose own prefix repeats is found where it occurs -- see
             * mark_advance(). */
            if (m->inq_mark && !m->inq_mark_seen) {
                m->inq_mark_pos = mark_advance(m->inq_mark, m->inq_mark_pos, v);
                if (m->inq_mark[m->inq_mark_pos] == '\0') m->inq_mark_seen = true;
            }
            /* --stop-mark: the same match, ending the run instead of releasing
             * type-ahead.  A test knows when it is done and can say so on the
             * console it is already writing to, which is the deterministic end
             * the doorbell gives without costing a program on the guest's disk
             * or the privileged OUT a user-mode program cannot execute.  The
             * character is handed to the console FIRST, so the transcript
             * contains the mark that ended the run.  Unlike --input-mark this
             * is not console-only -- see stop_mark_watch(). */
            stop_mark_watch(m, port, v);
            /* Prompt characters gate scripted input: the Coherent shell
             * prompts with '#', the kboot menu with "boot> ", and CP/M's CCP
             * with "A>" -- so '#' or '>' latches input-ready and counts a
             * prompt, letting scripts pace one line per prompt across all
             * three.  Latch only once the power-on tests are well behind us so
             * input is fed to a real prompt, not some earlier boot-time read;
             * the diagnostics print neither character and the first prompt
             * appears past 2-3M instructions, so 1M clears the diagnostics. */
            if ((v == '#' || v == '>') && m->cpu.insns > 1000000) {
                m->shell_up = true; m->prompt_seq++;
                m->at_prompt = true;     /* the console fell silent AT a prompt (see --idle) */
            } else m->at_prompt = false;
        }
        return;
    }
    ch->wr[r] = v;
    /* WR2 (vector) and WR9 (master int control) are single physical registers
     * shared by both channels OF ONE CHIP — keep that chip's two structs
     * coherent, and only that chip's: al.c gives each SCC its own vector
     * (VECTOR 0x10, +16 per chip, al.c:238,251-262), so leaking a WR2 write
     * across chips would hand every board the same vector. */
    if (r == 2 || r == 9) {
        SCCChan *o = scc_chan_of(m, chip, chan ^ 1);
        if (o) o->wr[r] = v;
    }
}

/* Highest-priority pending Rx interrupt among the LR board's channels, if
 * any: within a chip channel A outranks channel B, and U31 outranks U36. */
static bool lr_scc_vi(Machine *m, uint16_t *vec){
    for (int chip = 1; chip <= 2; chip++) {
        SCCChan *a = scc_chan_of(m, chip, 1), *b = scc_chan_of(m, chip, 0);
        if (a && scc_rx_int(a)) { *vec = scc_vector_chip(m, chip, 0x06); return true; }
        if (b && scc_rx_int(b)) { *vec = scc_vector_chip(m, chip, 0x02); return true; }
    }
    return false;
}

/* Which SCC an I/O port lands in, or -1 for none.  Each chip is decoded on
 * its own 0x80 page and so mirrors once in the upper half, which is the shape
 * the motherboard SCC has always had here; A7 is what separates U31 from U36
 * (`planning/PLA_REVERSE_ENGINEERING.md:159-161`). */
static int scc_chip_at(Machine *m, uint16_t port){
    if ((port & 0xFF80) == 0x0100) return 0;          /* U74, motherboard */
    if (!m->lr_scc) return -1;
    if ((port & 0xFF80) == 0x0300) return 1;          /* U31, LR board */
    if ((port & 0xFF80) == 0x0380) return 2;          /* U36, LR board */
    return -1;
}

/* Hand one byte to a receiver.
 *
 * The real Z8030 shifts the character in whether or not the CPU has read the
 * last one; the old one is gone and RR1 D5 (Rx Overrun) latches until an
 * Error Reset.  That is what --rx-overrun models.  Without it the caller
 * never offers a byte to a full receiver in the first place (the run loop
 * gates on rx_avail), which is why this emulator has never been able to lose
 * a character -- and why an interrupt-driven receive ring has been
 * indistinguishable from a polled read. */
static void scc_rx_deliver(Machine *m, SCCChan *ch, uint8_t b){
    if (ch->rx_avail && m->rx_overrun_on) { ch->rx_overrun = true; ch->rx_lost++; }
    ch->rx_data = b; ch->rx_avail = true;
}

/* deliver a byte to the console channel B receiver */
void scc_rx_console(Machine *m, uint8_t b){
    scc_rx_deliver(m, &m->scc_b, b);
}

/* deliver a byte to the channel A receiver (the second RS-232 port, /dev/tty51) */
void scc_rx_wire(Machine *m, uint8_t b){
    scc_rx_deliver(m, &m->scc_a, b);
}

/* One pass of the wired receivers: offer each wired port its next host byte.
 * This is the whole receive gate, and it is a function rather than run-loop
 * text so that --selftest exercises the real thing and not a copy of it. */
void scc_wire_service(Machine *m){
    for (int port = SERPORT_MB_A; port < SCC_PORTS; port++) {
        if (!m->wire_port_on[port]) continue;
        SCCChan *ch = scc_port(m, port);
        if (!ch) continue;
        /* The gate.  Off (the default), a full receiver is left alone and the
         * host endpoint keeps the byte, so nothing is ever lost.  On, the byte
         * is taken anyway and the unread one is destroyed. */
        if (ch->rx_avail && !m->rx_overrun_on) continue;
        int wb = wire_poll_char_n(port);
        if (wb >= 0) scc_rx_deliver(m, ch, (uint8_t)wb);
    }
}

/* deliver a byte to any port's receiver, by SERIAL PORT TABLE number */
void scc_rx_port(Machine *m, int port, uint8_t b){
    SCCChan *ch = scc_port(m, port);
    if (ch) scc_rx_deliver(m, ch, b);
}

/* ─────────────── HDC/FDC command-block processing (doorbell) ───────────────
 * The disk-controller card carries both the WD2010 hard-disk MCU and the
 * Commodore floppy controller. They share one PDMAC doorbell (port 0x0500)
 * and one vectored interrupt (vector 0x80), but each owns a 16-byte SASI-style
 * command block: the hard disk at hdc_cmdblk (default 0x080000) and the floppy
 * immediately after it at hdc_cmdblk+0x10. A block is "armed" when its error
 * byte (offset 0x0C) holds 0xFF; the controller overwrites it with a completion
 * status and raises the interrupt. Both media address their data by linear
 * block number, so this is a flat-file seek/copy for either one. */
static void hdc_process(Machine *m, uint32_t cb, Disk *disk, bool is_floppy){
    if (phys_read8(m, cb + 0x0C) != 0xFF) return;    /* not armed for this block */
    uint8_t opcode = phys_read8(m, cb + 0x00);

    uint8_t lunhi = phys_read8(m, cb + 0x01);
    uint8_t mid   = phys_read8(m, cb + 0x02);
    uint8_t low   = phys_read8(m, cb + 0x03);
    uint8_t bcnt  = phys_read8(m, cb + 0x04);
    uint32_t dma  = ((uint32_t)phys_read8(m, cb+0x06)<<16) |
                    ((uint32_t)phys_read8(m, cb+0x07)<<8)  |
                     (uint32_t)phys_read8(m, cb+0x08);
    uint32_t lba  = ((uint32_t)(lunhi & 0x1F)<<16) | ((uint32_t)mid<<8) | low;

    uint8_t status;
    switch (opcode){
        case 0x00: /* TestDriveReady */
        case 0x0C: /* SetDriveParams */
        case 0x01: /* Restore */
        case 0x05: /* CheckTrackFormat */
        case 0x03: /* RequestStatus */
            /* 0x92 = drive not ready (hard disk or floppy). */
            status = disk->present ? 0x00 : 0x92;
            break;
        case 0x04: /* FormatDisk (floppy only) — no low-level format to model */
            status = disk->present ? 0x80 : 0x92;
            break;
        case 0x0F: /* ChangeCmdBlockAddr — new base in DMA fields (hard disk) */
            if (!is_floppy) m->hdc_cmdblk = dma;
            status = 0x00;
            break;
        case 0x08: /* Read */ {
            if (!disk->present)              { status = 0x92; break; }
            if (lba + bcnt > disk->sectors)  { status = 0x92; break; }
            uint32_t n = (uint32_t)bcnt * 512;
            fseek(disk->fp, (long)lba * 512, SEEK_SET);
            /* DMA target is always in DRAM (segs 0x08-0x17) — block-copy it.
             * A partial image (trailing sectors trimmed) reads short; zero-fill
             * the remainder so unwritten blocks read as zero, not stale RAM. */
            if (((dma>>16)&0xFF) >= 0x08 && (((dma+n-1)>>16)&0xFF) <= 0x17) {
                if (watch_on < 0) watch_init();
                if (watch_on && dma <= watch_hi && dma+n > watch_lo)
                    fprintf(stderr, "[watch] DMA %06X..%06X <- disk lba=%u insns=%llu\n",
                            dma, dma+n-1, lba, (unsigned long long)m->cpu.insns);
                size_t got = fread(m->ram + dma, 1, n, disk->fp);
                if (got < n) memset(m->ram + dma + got, 0, n - got);
            } else {
                uint8_t buf[512];
                for (uint32_t s=0; s<bcnt; s++){
                    if (fread(buf,1,512,disk->fp) != 512) memset(buf,0,512);
                    for (int i=0;i<512;i++) phys_write8(m, dma + s*512 + i, buf[i]);
                }
            }
            status = 0x80;
            break; }
        case 0x0A: /* Write */ {
            if (!disk->present)              { status = 0x92; break; }
            if (lba + bcnt > disk->sectors)  { status = 0x92; break; }
            uint32_t n = (uint32_t)bcnt * 512;
            fseek(disk->fp, (long)lba * 512, SEEK_SET);
            if (((dma>>16)&0xFF) >= 0x08 && (((dma+n-1)>>16)&0xFF) <= 0x17) {
                fwrite(m->ram + dma, 1, n, disk->fp);
            } else {
                uint8_t buf[512];
                for (uint32_t s=0; s<bcnt; s++){
                    for (int i=0;i<512;i++) buf[i] = phys_read8(m, dma + s*512 + i);
                    fwrite(buf,1,512,disk->fp);
                }
            }
            fflush(disk->fp);
            status = 0x80;
            break; }
        default:
            status = 0x81; /* bad opcode */
            break;
    }
    phys_write8(m, cb + 0x0C, status);       /* completion sentinel */
    phys_write8(m, cb + 0x0D, lunhi);
    phys_write8(m, cb + 0x0E, mid);
    /* raise the PDMAC disk-completion vectored interrupt (vector 0x80) */
    m->disk_vi = true;
}

/* Doorbell (out 0x0500,1): service whichever command block is armed. The hard
 * disk sits at hdc_cmdblk; the floppy controller's block follows at +0x10. Both
 * are always examined — the floppy controller is a fixed part of the card, so a
 * command issued with no image attached completes with a "no floppy" status
 * rather than leaving the driver waiting for an interrupt that never comes. */
static void hdc_doorbell(Machine *m){
    hdc_process(m, m->hdc_cmdblk, &m->disk, false);
    hdc_process(m, m->hdc_cmdblk + 0x10, &m->floppy, true);
}

/* ─────────────── OKI MSM58321 real-time clock (board U16) ───────────────
 *
 * The C900 carries an OKI MSM58321 clock/calendar module at U16
 * (~/git/C900/schematics/c900-chips.txt: "M58321 OKI Japan 4456"; datasheet
 * ~/git/C900/docs/RTC-58321.pdf).  It has a 4-bit bidirectional bus and three
 * strobes, all bit-banged through Z-CIO #1 — the same CIO the boot ROM uses
 * for the keyboard:
 *
 *	PB0..PB3  D0..D3       bidirectional; the chip drives during READ
 *	PB4       READ         active-high level: chip drives D0..D3
 *	PB5       WRITE        active-high pulse: latches D0..D3 into reg[addr]
 *	PB6       ADDR WRITE   active-high pulse: latches D0..D3 as the address
 *	PB7       STOP         active-high LEVEL: halts the oscillator
 *	PC1       /CS          active-low level; gates the three strobes
 *
 * From the datasheet, not from any particular driver:
 *
 *  • Register Table (p2).  Sixteen 4-bit registers; 0..0xC are the time and
 *    calendar digits, 0xD the post-stage reset, 0xE/0xF a standard-signal
 *    output.  No address auto-increment.
 *  • The "*" marks there, and the Supplement's "* mark: Writable.  Recognized
 *    as 0 while in read mode", give the read-back masks in rtc_rdmask[]: the
 *    unused high bits of S10, MI10, W and MO10 read 0, as does the whole reset
 *    register.  H10's 24/12 and PM/AM and D10's leap bits carry no "*" and are
 *    ordinary readable bits.
 *  • "PM/AM": "In 24 H mode, this will be 0", applied at read.
 *  • "Reset register": a WRITE strobe at address 0xD, resetting the divider
 *    after the 1/2^15 stage (and a BUSY circuit the C900 does not wire).
 *  • "D3 and D2 of 10 days digit": the leap-year selection, 00 for the leap
 *    year then 01, 10, 11 for a surplus of 3, 2, 1 -- a countdown.  February
 *    has 29 days exactly when the field is 00.
 *
 * Whether that field self-advances at a year rollover is not stated; it is
 * modelled as a 2-bit counter decrementing on each year carry.  Flagged as an
 * inference in docs/RTC-NOTES.md.
 *
 * There is no battery, so the host seeds the chip (rtc_set_time) and the
 * divider runs off emulated CPU time: `ips' instructions per second, default
 * 1.5M to match the CT3 tick above.  --rtc-ips is the equivalent of
 * overclocking the crystal, which is how a carry is made to land inside a
 * driver's register read.  The divider advances lazily at each CIO #1 access,
 * so it depends on the instruction count and never on host time.
 */
#define RTC_PB_D      0x0F
#define RTC_PB_READ   0x10
#define RTC_PB_WRITE  0x20
#define RTC_PB_ADWR   0x40
#define RTC_PB_STOP   0x80

/* CIO #1 register numbers (the I/O port is 2*reg+1) the module hangs off. */
#define CIO_PBDATA    0x0E    /* port 0x001D */
#define CIO_PCDATA    0x0F    /* port 0x001F */
#define CIO_PBDD      0x2B    /* port 0x0057, 1 = input */

/* Instructions of emulated CPU time per second of RTC time. */
#define RTC_DEFAULT_IPS 1500000u

/* Bits a read returns; 0 where the datasheet's "*" says "recognized as 0
 * while in read mode".  Index = register address. */
static const uint8_t rtc_rdmask[16] = {
    0x0F, 0x07, 0x0F, 0x07, 0x0F, 0x0F, 0x07, 0x0F,
    0x0F, 0x0F, 0x01, 0x0F, 0x0F, 0x00, 0x00, 0x00
};

/* /CS is PC1, active low.  Port C is a plain latch here; the C900 driver
 * makes PC1 an output before it ever asserts /CS. */
static bool rtc_cs(Machine *m){ return (m->cio1[CIO_PCDATA] & 0x02) == 0; }

static int rtc_mlen(int mo, bool leap){
    static const int d[13] = {31,31,28,31,30,31,30,31,31,30,31,30,31};
    if (mo < 1 || mo > 12) return 31;          /* garbage month: don't hang */
    return (mo == 2 && leap) ? 29 : d[mo];
}

/* Day-of-week for a calendar date, 0 = Sunday (seeding only; the chip's W
 * register free-counts mod 7 thereafter, exactly as the hardware does). */
static int rtc_wday(int y, int m, int d){
    static const int t[12] = {0,3,2,5,0,3,5,1,4,6,2,4};
    if (m < 3) y -= 1;
    return (y + y/4 - y/100 + y/400 + t[(m-1) & 15] + d) % 7;
}

static void rtc_year_tick(RTC *r){
    int y = (r->regs[0x0C] & 0x0F) * 10 + (r->regs[0x0B] & 0x0F);
    y = (y + 1) % 100;
    r->regs[0x0B] = (uint8_t)(y % 10);
    r->regs[0x0C] = (uint8_t)(y / 10);
    /* Leap-year selection counts down: its code 00,11,10,01 corresponds to a
     * surplus of 0,1,2,3, so one year later is one code lower. */
    r->regs[0x08] = (uint8_t)((r->regs[0x08] & 0x03)
                  | ((((r->regs[0x08] >> 2) + 3) & 3) << 2));
}

static void rtc_day_tick(RTC *r){
    int d  = (r->regs[0x08] & 0x03) * 10 + (r->regs[0x07] & 0x0F);
    int mo = (r->regs[0x0A] & 0x01) * 10 + (r->regs[0x09] & 0x0F);
    bool leap = ((r->regs[0x08] >> 2) & 3) == 0;

    r->regs[0x06] = (uint8_t)((r->regs[0x06] + 1) % 7);   /* W, free-running */
    if (++d > rtc_mlen(mo, leap)) {
        d = 1;
        if (++mo > 12) { mo = 1; rtc_year_tick(r); }
        r->regs[0x09] = (uint8_t)(mo % 10);
        r->regs[0x0A] = (uint8_t)((r->regs[0x0A] & 0x0E) | (mo / 10));
    }
    r->regs[0x07] = (uint8_t)(d % 10);
    r->regs[0x08] = (uint8_t)((r->regs[0x08] & 0x0C) | (d / 10));
}

static void rtc_hour_tick(RTC *r){
    uint8_t h10 = r->regs[0x05];
    int h = (h10 & 0x03) * 10 + (r->regs[0x04] & 0x0F);

    if (h10 & 0x08) {                          /* 24-hour mode */
        if (++h > 23) { h = 0; rtc_day_tick(r); }
        r->regs[0x04] = (uint8_t)(h % 10);
        r->regs[0x05] = (uint8_t)(0x08 | (h / 10));   /* PM/AM reads 0 in 24H */
    } else {                                   /* 12-hour mode, PM/AM in D2 */
        uint8_t pm = h10 & 0x04;
        if (++h > 12) h = 1;
        else if (h == 12) {                    /* 11 → 12 flips the half-day; */
            pm ^= 0x04;                        /* 11 PM → 12 AM carries a day */
            if (!pm) rtc_day_tick(r);
        }
        r->regs[0x04] = (uint8_t)(h % 10);
        r->regs[0x05] = (uint8_t)(pm | (h / 10));
    }
}

/* One second: the BCD digit counters, each with its own modulus, carrying
 * into the next exactly as the divider chain does. */
static void rtc_tick(RTC *r){
    r->n_tick++;
    if (++r->regs[0x00] > 9) {                 /* S1  */
        r->regs[0x00] = 0;
        if (++r->regs[0x01] > 5) {             /* S10 */
            r->regs[0x01] = 0;
            if (++r->regs[0x02] > 9) {         /* MI1 */
                r->regs[0x02] = 0;
                if (++r->regs[0x03] > 5) {     /* MI10 */
                    r->regs[0x03] = 0;
                    rtc_hour_tick(r);
                }
            }
        }
    }
}

/* Advance the divider to the current instruction count.  Frozen by STOP. */
static void rtc_advance(Machine *m){
    RTC *r = &m->rtc;
    if (!r->present || r->stopped) return;
    uint64_t now = m->cpu.insns;
    while (now - r->base >= r->ips) {
        r->base += r->ips;                     /* = when this carry happened */
        rtc_tick(r);
        /* Did the carry land inside a transaction, i.e. after the driver
         * asserted /CS and before it released it?  That is the race a clock
         * driver has to survive, and counting it is how a test can prove it
         * actually happened rather than hoping. */
        if (rtc_cs(m) && r->base >= r->cs_since) r->n_tick_selected++;
    }
}

static void rtc_set_stop(Machine *m, bool stop){
    RTC *r = &m->rtc;
    if (stop == r->stopped) return;
    if (stop) { rtc_advance(m); r->held = m->cpu.insns - r->base; r->stopped = true; }
    else      { r->base = m->cpu.insns - r->held; r->stopped = false; }
}

/* Port B write: strobe edge detection.  ADDR-WRITE and WRITE act only while
 * /CS is asserted; STOP is wired straight to the pin and is not gated. */
static void rtc_pb_write(Machine *m, uint8_t v){
    RTC *r = &m->rtc;
    if (!r->present) return;
    /* Note the /CS edge BEFORE catching the divider up: the accesses that
     * assert /CS are often the first CIO traffic in a long while, and the
     * seconds they discover elapsed before the transaction began. */
    if (rtc_cs(m) && !r->prev_cs) r->cs_since = m->cpu.insns;
    r->prev_cs = rtc_cs(m);
    rtc_advance(m);
    uint8_t old = r->prev_pb;
    if (rtc_cs(m)) {
        if ((v & RTC_PB_ADWR) && !(old & RTC_PB_ADWR))
            r->addr = v & RTC_PB_D;
        if ((v & RTC_PB_WRITE) && !(old & RTC_PB_WRITE)) {
            r->n_write++;
            if (r->addr == 0x0D) {             /* post-stage reset */
                r->base = m->cpu.insns;
                r->held = 0;
            } else {
                r->regs[r->addr] = v & RTC_PB_D;
            }
        }
    }
    r->prev_pb = v;
    rtc_set_stop(m, (v & RTC_PB_STOP) != 0);
}

/* Port B read.  The chip drives D0..D3 while /CS is asserted and READ is
 * high; the CIO returns the pin for the bits it has programmed as inputs and
 * its own output latch for the rest.
 *
 * With the module absent — or with PB0..3 still outputs — the caller sees the
 * latch instead.  That is a property of the CIO and of an unpopulated socket,
 * NOT of the chip: the block diagram drives D0..D3 through a tri-state stage
 * enabled by CS + READ, and the address latch is write-only, so a fitted
 * module can never echo the address back.  What a bare board's floating pins
 * really read is unknown; the latch is modelled here because it is the one
 * level the emulator can justify. */
static uint8_t rtc_pb_read(Machine *m, uint8_t latch){
    RTC *r = &m->rtc;
    if (!r->present) return latch;
    rtc_advance(m);
    if (!rtc_cs(m) || !(latch & RTC_PB_READ)) return latch;
    uint8_t din = m->cio1[CIO_PBDD] & RTC_PB_D;      /* 1 = CIO input */
    if (!din) return latch;
    uint8_t v = r->regs[r->addr] & rtc_rdmask[r->addr];
    if (r->addr == 0x05 && (r->regs[0x05] & 0x08)) v &= (uint8_t)~0x04;
    r->n_read++;
    return (uint8_t)((latch & ~din) | (v & din));
}

/* Seed the register file from a wall-clock time, in 24-hour mode.  This
 * stands in for the battery-backed contents real silicon would hold. */
void rtc_set_time(Machine *m, int y, int mo, int d, int h, int mi, int s){
    RTC *r = &m->rtc;
    r->regs[0x00] = (uint8_t)(s % 10);   r->regs[0x01] = (uint8_t)(s / 10);
    r->regs[0x02] = (uint8_t)(mi % 10);  r->regs[0x03] = (uint8_t)(mi / 10);
    r->regs[0x04] = (uint8_t)(h % 10);   r->regs[0x05] = (uint8_t)(0x08 | (h / 10));
    r->regs[0x06] = (uint8_t)rtc_wday(y, mo, d);
    r->regs[0x07] = (uint8_t)(d % 10);
    /* D10: leap-year selection (datasheet code) in D3:D2, day tens in D1:D0 */
    r->regs[0x08] = (uint8_t)((((4 - (y & 3)) & 3) << 2) | (d / 10));
    r->regs[0x09] = (uint8_t)(mo % 10);  r->regs[0x0A] = (uint8_t)(mo / 10);
    r->regs[0x0B] = (uint8_t)((y % 100) % 10);
    r->regs[0x0C] = (uint8_t)((y % 100) / 10);
    r->addr = 0; r->prev_pb = 0; r->stopped = false;
    r->prev_cs = false; r->cs_since = 0;
    r->base = m->cpu.insns; r->held = 0;
    r->present = true;
}

/* What the module saw during the run.  n_tick_selected is the count of
 * one-second carries that landed while /CS was asserted — i.e. inside a
 * driver's transaction, the race a clock driver has to survive.  The leap
 * field is D10's leap-year selection as it now stands: the datasheet's
 * countdown code, 00 for the leap year itself and 01/10/11 for a surplus of
 * 3/2/1.  It is reported because a guest that sets the clock writes it and
 * nothing else can see what it wrote — within one year the code and the raw
 * surplus behave identically. */
void rtc_report(Machine *m, FILE *out){
    RTC *r = &m->rtc;
    if (!r->present) { fprintf(out, "[rtc: no module fitted]\n"); return; }
    rtc_advance(m);
    fprintf(out,
        "[rtc: %d%d/%d%d/%d%d %d%d:%d%d:%d%d, leap %d, %llu reads, "
        "%llu writes, %llu ticks (%llu inside a /CS transaction), "
        "%llu insn/s]\n",
        r->regs[0x0A] & 1, r->regs[0x09] & 0x0F,      /* MM */
        r->regs[0x08] & 3, r->regs[0x07] & 0x0F,      /* DD */
        r->regs[0x0C] & 0x0F, r->regs[0x0B] & 0x0F,   /* YY */
        r->regs[0x05] & 3, r->regs[0x04] & 0x0F,      /* HH */
        r->regs[0x03] & 0x0F, r->regs[0x02] & 0x0F,   /* MM */
        r->regs[0x01] & 0x0F, r->regs[0x00] & 0x0F,   /* SS */
        (r->regs[0x08] >> 2) & 3,                     /* leap-year sel */
        (unsigned long long)r->n_read, (unsigned long long)r->n_write,
        (unsigned long long)r->n_tick, (unsigned long long)r->n_tick_selected,
        (unsigned long long)r->ips);
}

/* ─────────────── I/O dispatch ─────────────── */
uint16_t io_read(Machine *m, uint16_t port, bool is_byte, bool special){
    (void)is_byte;
    if (special) {
        uint8_t low = port & 0xFF;
        if (low == 0xFC || low == 0xF8) {
            return mmu_cmd_read(&m->mmu, (uint8_t)(port>>8));
        }
        return 0;
    }
    /* SCC, on D15:D8.  Motherboard U74 at 0x0100-0x017F; the LR board's U31
     * at 0x0300-0x037F and U36 at 0x0380-0x03FF, present only when the board
     * is fitted (see emu.h's SERIAL PORT TABLE).  With no LR board these
     * addresses fall through to the floating 0 they always returned. */
    {
        int chip = scc_chip_at(m, port);
        if (chip >= 0) {
            int reg = (port >> 1) & 0x0F;
            int chan = (port >> 5) & 1;       /* 1→A, 0→B */
            return (uint16_t)scc_read(m, chip, chan, reg) << 8;
        }
    }
    /* CIO #1 0x0000-0x007F — plain register file (read back what was written).
     * In reset all regs read 0 except MICR. */
    if ((port & 0xFF80) == 0x0000) {
        int reg = (port >> 1) & 0x3F;
        if (m->cio1_reset && reg != 0) return 0;
        uint8_t v = m->cio1[reg];
        /* Port B: the RTC module drives D0..D3 back at us during a READ. */
        if (reg == CIO_PBDATA) v = rtc_pb_read(m, v);
        return (uint16_t)v << 8;
    }
    /* CIO #2 0x0080-0x00FF (inert) */
    if ((port & 0xFF80) == 0x0080) {
        int reg = (port >> 1) & 0x3F;
        if (m->cio2_reset && reg != 0) return 0;
        return (uint16_t)m->cio2[reg] << 8;
    }
    return 0;
}

void io_write(Machine *m, uint16_t port, uint16_t data, bool is_byte, bool special){
    (void)is_byte;
    if (special) {
        uint8_t low = port & 0xFF;
        if (low == 0xFC || low == 0xF8) mmu_cmd_write(&m->mmu, (uint8_t)(port>>8), (uint8_t)data);
        return;
    }
    {                                         /* SCC (D15:D8) */
        int chip = scc_chip_at(m, port);
        if (chip >= 0) {
            int reg = (port >> 1) & 0x0F;
            int chan = (port >> 5) & 1;
            scc_write(m, chip, chan, reg, (uint8_t)(data >> 8));
            return;
        }
    }
    if ((port & 0xFF80) == 0x0000) {          /* CIO #1 */
        int reg = (port >> 1) & 0x3F;
        uint8_t v = (uint8_t)(data >> 8);
        if (reg == 0x00) {                    /* MICR: bit0=reset */
            if (!(v & 0x01)) m->cio1_reset = false;
        }
        if (reg == 0x0C) {                    /* CT3 Command & Status */
            /* Apply the command (D7:D5) to the IP/IE/IUS bits rather than
             * storing the byte raw. */
            uint8_t cmd = (v >> 5) & 0x07;
            uint8_t cs = m->cio1[0x0C];
            switch (cmd){
                case 1: cs &= ~(0x20|0x80); break;      /* clear IP + IUS */
                case 2: cs |= 0x80; break;              /* set IUS */
                case 3: cs &= ~0x80; break;             /* clear IUS */
                case 4: cs |= 0x20; break;              /* set IP */
                case 5: cs &= ~0x20; break;             /* clear IP */
                case 6: cs |= 0x40; break;              /* set IE */
                case 7: cs &= ~0x40; break;             /* clear IE */
            }
            cs = (cs & ~0x04) | (v & 0x04);             /* GCB (gate) R/W */
            if (v & 0x02) {                             /* TCB: trigger/start */
                if (!m->ct3_running || (m->cio1[0x1E] & 0x04)) { /* REB */
                    m->ct3_running = true;
                    m->ct3_accum = 0;
                }
            }
            m->cio1[0x0C] = cs;
            return;
        }
        m->cio1[reg] = v;
        /* Port B carries the RTC's data nibble and its three strobes; Port C
         * bit 1 is its /CS, so both writes are handed to the chip model.  The
         * Port C write is passed through rtc_pb_write with the Port B level
         * unchanged so that only the /CS level moves — a strobe that was
         * already high when /CS falls is not seen as an edge. */
        if (reg == CIO_PBDATA) rtc_pb_write(m, v);
        else if (reg == CIO_PCDATA) rtc_pb_write(m, m->rtc.prev_pb);
        return;
    }
    if ((port & 0xFF80) == 0x0080) {          /* CIO #2 */
        int reg = (port >> 1) & 0x3F;
        uint8_t v = (uint8_t)(data >> 8);
        if (reg == 0x00 && !(v & 0x01)) m->cio2_reset = false;
        m->cio2[reg] = v;
        return;
    }
    /* Stop doorbell (--stop-port): an EXPLICIT end-of-session signal for guest
     * code that is free to make one.  A word write of 0xC900 to port 0x0FFE
     * ends the run with status 0, exactly as the idle rule does; any other
     * value is ignored, so a wild store into unclaimed I/O space cannot ring
     * it.  Nothing in the machine answers 0x0FFE -- it is above every port the
     * board decodes (CIO 0x0000-0x00FF, SCC 0x0100-0x017F, system latch
     * 0x0200-0x02FF, PDMAC 0x0500-0x05FF) -- and an I/O port rather than a
     * memory address on purpose: Z8000 I/O space needs no MMU mapping, so a
     * guest rings it with the outb() it already has, while a physical address
     * would need a segment set up first and would sit in the graphics-card
     * space (0x37/0x3A/0x3E/0x3F) that our own cursor code writes to.
     *
     * This is the deterministic mechanism and it truncates nothing, because
     * the guest picks the moment.  It costs a program on the guest's disk,
     * which is why it is not what the CP/M verify suite uses -- see --idle. */
    if (port == m->stop_port && m->stop_port) {
        if (data == 0xC900) {                 /* outb(port,0xC9) lands here too:
                                               * a byte OUT drives D15:D8 */
            m->stop = true;
            m->stop_why = "the guest rang the stop doorbell";
        }
        return;
    }
    if ((port & 0xFF00) == 0x0200) return;    /* system latch (write-only, ignore) */
    if (port >= 0x0500 && port <= 0x05FF) {   /* PDMAC disk doorbell */
        if (data != 0) hdc_doorbell(m);
        return;
    }
    /* everything else: ignore */
}

/* ─────────────── machine setup / run ─────────────── */
Machine *machine_new(void){
    decode_init();
    Machine *m = calloc(1, sizeof *m);
    /* +2 guard bytes: the flat (--exec) word accessors touch a and a+1 without
     * a decode check, and a is allowed to be the last address of the space. */
    m->ram = calloc(PHYS_SIZE + 2, 1);
    m->cpu.m = m;
    m->hdc_cmdblk = 0x080000;
    m->cio1_reset = true;
    m->cio2_reset = true;
    /* The module is fitted on a real board, so it is fitted by default.  One
     * second = RTC_DEFAULT_IPS instructions, the same rate the CT3 tick
     * approximation above implies (TC 30000 → ~100 Hz).  main() seeds the
     * time; without a seed the chip starts at 1978-01-01 00:00:00. */
    m->rtc.ips = RTC_DEFAULT_IPS;
    rtc_set_time(m, 1978, 1, 1, 0, 0, 0);
    mmu_reset(&m->mmu);
    return m;
}

int machine_load_rom(Machine *m, const char *dir){
    char hp[512], lp[512];
    snprintf(hp, sizeof hp, "%s/bios_h.bin", dir);
    snprintf(lp, sizeof lp, "%s/bios_l.bin", dir);
    FILE *fh = fopen(hp, "rb"), *fl = fopen(lp, "rb");
    if (!fh || !fl) { if(fh)fclose(fh); if(fl)fclose(fl); return -1; }
    uint8_t hi[ROM_SIZE/2], lo[ROM_SIZE/2];
    size_t nh = fread(hi, 1, sizeof hi, fh);
    size_t nl = fread(lo, 1, sizeof lo, fl);
    fclose(fh); fclose(fl);
    if (nh != nl) return -2;
    for (size_t i=0;i<nh;i++){ m->rom[2*i] = hi[i]; m->rom[2*i+1] = lo[i]; }
    return 0;   /* stock ROM — the full power-on memory test runs unmodified */
}

int machine_attach_disk(Machine *m, const char *path){
    FILE *f = fopen(path, "r+b");
    if (!f) f = fopen(path, "rb");
    if (!f) return -1;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    m->disk.fp = f;
    m->disk.sectors = (uint32_t)(sz / 512);
    /* MiniScribe 20MB geometry (612×4×17) — matches the shipped images */
    m->disk.cyls = 612; m->disk.heads = 4; m->disk.spt = 17;
    m->disk.present = true;
    return 0;
}

int machine_attach_floppy(Machine *m, const char *path){
    FILE *f = fopen(path, "r+b");
    if (!f) f = fopen(path, "rb");
    if (!f) return -1;
    m->floppy.fp = f;
    /* One fixed C900 floppy format: NFBLK linearly-addressed blocks. The image
     * may be shorter (trailing zero sectors trimmed); reads past EOF zero-fill,
     * so present the full block count regardless of file length. */
    m->floppy.sectors = FLOPPY_BLOCKS;
    m->floppy.cyls = 80; m->floppy.heads = 2; m->floppy.spt = 16;
    m->floppy.present = true;
    return 0;
}

/* ── THE PARK WATCH: a halted guest that a periodic interrupt keeps waking ──
 *
 * The check at the bottom of the run loop ("the guest halted with no interrupt
 * pending") sees a CPU that has stopped and has nothing left to restart it.  It
 * is defeated by any periodic source at all.  With the CT3 tick running, a
 * guest parked in kboot's
 *
 *      hang:   halt
 *              jr      hang
 *
 * -- where crt.s leaves the machine when bmain() returns after an unrecoverable
 * error, a bootinfo version refusal being the one that bit -- wakes on every
 * tick, runs the interrupt path, falls back into `halt', and does that forever.
 * `idle' is reset by each tick, so that check never fires; and because only a
 * couple of instructions retire per tick, the instruction budget does not
 * advance either.  Measured on this machine: ~24 M instructions/s while the
 * loader was working, under 250/s once parked, so the suite's --max=600000000
 * is about four weeks away.  An emulator sat in exactly this state for 16 h
 * 47 min at 99% CPU before a person noticed.
 *
 * WHY THIS IS NOT A TIMEOUT.  A clock ends a run because it has been going a
 * long time, which is why a default one was refused: a test that legitimately
 * needs the emulator for hours is not doing anything wrong.  This ends a run
 * because the guest is DEMONSTRABLY IN A CYCLE.  At every halt we take a
 * signature of everything the guest could have changed in a way that matters --
 * the whole register file, the PC, the flag/control word, the console
 * activity stamp, how much scripted input has been handed over, how many
 * prompts have been printed -- and the counter advances only while consecutive
 * halts are BIT-FOR-BIT IDENTICAL.  A guest that prints one character, reads
 * one byte, touches one register or halts anywhere else resets it and gets the
 * full budget again, however long it has already been running.  So no run that
 * is making progress can reach this bound, no matter how long it takes; a run
 * that reaches it has provably been repeating itself.
 *
 * Nor is it a gate.  It ENDS a run, with a plain reason, exactly as the sibling
 * check above it does -- it sets no failure, changes no exit status, and needs
 * no flag.  A caller that judges a session by its transcript (the CP/M suite's
 * tests/bootgate.sh does) gets the same verdict it gets today, and gets it in
 * seconds instead of never.
 *
 * And it moves no instruments: it only READS state that already exists, on a
 * path the guest cannot observe.  A run retires exactly the instructions it
 * retired before. */

/* Iterations of an unbroken, identical park before the run is called over.
 * The sibling check uses 2,000,000 for a CPU with no wakeup at all, which is
 * unambiguous the moment it is seen.  This wants more room, so it is 250x
 * that: about twenty seconds of parked spinning on a current host, and
 * unreachable for a guest whose registers or console traffic ever change. */
#define PARK_LOOPS 500000000ull
/* Consecutive non-halted iterations that count as the guest having WOKEN UP
 * AND DONE SOMETHING, which throws the signature away rather than merely
 * resetting the count.  A tick interrupt path is tens of instructions; this is
 * three orders of magnitude more, so no service routine reaches it and no real
 * stretch of work fails to. */
#define PARK_AWAKE 100000ull

typedef struct {
    uint64_t budget;        /* PARK_LOOPS for a real run; the selftest uses a small one */
    uint64_t loops;         /* iterations parked in an unbroken identical halt */
    uint64_t awake;         /* consecutive iterations not halted */
    bool     was_halted;    /* to catch the moment it halts, which is when we look */
    bool     valid;         /* a signature has been taken */
    /* the signature */
    uint16_t R[16];
    uint32_t pc;
    uint16_t fcw;
    uint64_t tx;            /* m->last_tx_insn: any console byte in or out moves it */
    int      inq;           /* m->inq_pos: a scripted byte was handed over */
    uint32_t seq;           /* m->prompt_seq: a prompt was printed */
} ParkWatch;

/* Clears the count and the signature.  The budget is the watch's setting, not
 * part of its state, so it survives -- park_reset() is called mid-run. */
static void park_reset(ParkWatch *p){
    uint64_t b = p->budget; memset(p, 0, sizeof *p); p->budget = b;
}
static void park_init(ParkWatch *p, uint64_t budget){
    memset(p, 0, sizeof *p); p->budget = budget;
}

/* Called once per run-loop iteration, after cpu_step().  Returns true when the
 * run should end, having set m->stop_why. */
static bool park_step(ParkWatch *p, Machine *m){
    CPU *c = &m->cpu;

    if (!c->halted) {
        /* Woken.  A tick's interrupt path is a brief excursion and must not by
         * itself clear the park -- if it did, one periodic source would hide a
         * dead guest again, which is the whole bug.  A long stretch of running
         * is different: that is a guest doing work, and the signature it left
         * behind is stale. */
        if (++p->awake > PARK_AWAKE) { uint64_t a = p->awake; park_reset(p); p->awake = a; }
        p->was_halted = false;
        return false;
    }
    p->awake = 0;

    if (!p->was_halted) {                       /* the instant it (re)halted */
        bool same = p->valid
                 && c->pc  == p->pc
                 && c->fcw == p->fcw
                 && !memcmp(c->R, p->R, sizeof p->R)
                 && m->last_tx_insn == p->tx
                 && m->inq_pos      == p->inq
                 && m->prompt_seq   == p->seq;
        if (!same) {                            /* something moved: start over */
            p->loops = 0;
            p->valid = true;
            memcpy(p->R, c->R, sizeof p->R);
            p->pc = c->pc; p->fcw = c->fcw;
            p->tx = m->last_tx_insn; p->inq = m->inq_pos; p->seq = m->prompt_seq;
            if (getenv("C900_PARK_DEBUG"))
                fprintf(stderr, "[park reset at pc=%06X insns=%llu]\n",
                        c->pc, (unsigned long long)c->insns);
        }
    }
    p->was_halted = true;

    if (p->valid && ++p->loops > p->budget) {
        m->stop_why = "the guest is parked: halted, woken only by a periodic "
                      "interrupt, and doing nothing when woken";
        return true;
    }
    return false;
}

void machine_run(Machine *m){
    CPU *c = &m->cpu;
    mmu_reset(&m->mmu);
    cpu_reset(c);
    console_init();

    uint64_t idle = 0;
    ParkWatch park; park_init(&park, PARK_LOOPS);
    for (;;) {
        if (m->stop) break;
        if (m->insn_limit && c->insns >= m->insn_limit) {
            m->stop_why = "instruction budget (--max) exhausted";
            m->max_reached = true;       /* the one ending --require-stop rejects */
            break;
        }

        /* Idle exit (--idle, off unless armed): the scripted input is used up,
         * a prompt character was the last thing printed, the console has been
         * quiet for idle_quiet, and the guest is spinning on RR0.
         *
         * The poll streak is what separates a guest waiting from a guest
         * working -- silence does not, since a long computation prints nothing
         * either.  A guest that reads the console by interrupt never builds a
         * streak, so the channel never fires for one. */
        if (m->idle_quiet && m->inq_len && m->inq_pos >= m->inq_len &&
            m->at_prompt && m->rx_poll_streak >= RX_BLOCKED_POLLS &&
            (c->insns - m->last_tx_insn) > m->idle_quiet) {
            m->stop_why = "scripted input consumed, guest blocked at a prompt";
            break;
        }

        /* Poll host console only every 4096 instructions — _kbhit()/read()
         * is a syscall and doing it per-instruction dominates runtime. */
        if (!m->scc_b.rx_avail) {
            /* Coherent reads the console by Rx interrupt and does not poll RR0
             * while idle, so a byte goes in once the prompt is up and the
             * console has been briefly TX-quiet.  After a CR we wait for a
             * fresh '#' (prompt_seq to advance) before the next command's first
             * byte, so a still-running command cannot swallow it. */
            /* Past inq_gateoff the prompt requirement drops; the quiet-console
             * one still applies.  A curses program prints no `#', so without
             * this the CR ending its first input is the last byte it can ever
             * receive.  A position rather than a flag: the shell commands that
             * set the game up still need the gate. */
            bool past_gate = (m->inq_gateoff >= 0 &&
                              m->inq_pos >= m->inq_gateoff);
            /* The first byte past the gate waits much longer: a curses program
             * is noisy while it draws and only then goes quiet to read, and
             * that quiet is the only ready signal available. */
            uint64_t quiet = (past_gate && m->inq_pos == m->inq_gateoff)
                             ? 40000000ull : 300000ull;
            /* Past the gate a command's own output pauses look like silence,
             * and a byte fed into one is eaten by its output path (CP/M's BDOS
             * checks for ^S/^C there).  So wait until the guest is provably
             * reading: an RR0 poll streak, or the long silence for a guest that
             * never polls.  Per-byte, not sticky -- the boot ROM polls at its
             * menu, and a sticky flag would then pace an interrupt-driven guest
             * booted through it at 40M per keystroke. */
            if (past_gate && (m->inq_cr_wait || m->guest_polls) &&
                m->rx_poll_streak < RX_BLOCKED_POLLS)
                quiet = 40000000ull;
            /* TYPE-AHEAD (the \i escape marks the byte): deliver it the
             * moment the receiver is free, with no gate, no prompt and no
             * quiet wait.  Everything above answers the question "is the
             * guest ready for this byte yet?", and answers it conservatively
             * because a byte handed over too early is eaten by whatever read
             * the guest happens to be in -- a boot-ROM probe, or a running
             * program's own output path, where CP/M's BDOS polls for ^S/^Q/^C
             * between characters.  \i is the caller saying it WANTS that
             * reader: the ^S of a flow-control test is aimed at exactly the
             * output-path poll the pacing exists to dodge, and no amount of
             * waiting produces it, because a guest that is printing never
             * looks blocked and never falls silent.  So the choice is left
             * where it belongs, with whoever wrote the script, one byte at a
             * time: the gated pacing is unchanged for every byte not marked,
             * and a marked byte still waits for the receiver to be empty, so
             * the queue can never overrun the guest or reorder itself. */
            bool now = m->inq_pos < m->inq_len && m->inq_now[m->inq_pos] &&
                       (!m->inq_mark || m->inq_mark_seen);
            if (m->inq_pos < m->inq_len &&
                (now || (m->shell_up &&
                         (past_gate || m->prompt_seq >= m->inq_wait_seq) &&
                         (c->insns - m->last_tx_insn) > quiet))) {
                uint8_t b = m->inq[m->inq_pos++];
                scc_rx_console(m, b);
                if (getenv("C900_FEED_DEBUG"))
                    fprintf(stderr, "[feed %02x '%c' insns=%llu streak=%u polls=%d quiet=%s]\n",
                            b, (b>=32&&b<127)?b:'.', (unsigned long long)c->insns,
                            m->rx_poll_streak, m->guest_polls,
                            now ? "type-ahead" : "waited");
                m->guest_polls = false;
                m->inq_cr_wait = (b == '\r' || b == '\n');
                if (m->inq_cr_wait) m->inq_wait_seq = m->prompt_seq + 1;
            } else if ((m->tick_counter & 0x0FFF) == 0) {
                int ch = console_poll_char();
                if (ch == 0x1D) { m->stop = true; m->stop_why = "Ctrl-] at the console"; }
                                                               /* Ctrl-] quits the emulator;
                                                                * Ctrl-C (0x03) passes through */
                else if (ch >= 0) scc_rx_console(m, (uint8_t)ch);   /* serial input */
            }
        }

        /* Wired receivers.  By default, take the next byte from a host
         * endpoint only once the guest has read the previous one: the real
         * chip has a three-deep FIFO and a byte arriving faster than the
         * driver services the Rx interrupt is simply lost, but here the
         * endpoint holds it instead, so the wire cannot overrun the guest and
         * SLIP frames arrive whole.  --rx-overrun drops that gate for the
         * wires (never for the console, whose pacing is a separate machine
         * above) so a receive ring can be told from a polled read by whether
         * it loses characters.
         *
         * The loop is skipped entirely when nothing is wired, so a run with no
         * --wire executes exactly the instructions it did before. */
        if (m->any_wire && (m->tick_counter & 0x3F) == 0) scc_wire_service(m);

        m->tick_counter++;
        /* CIO #1 CT3 down-counter in continuous mode = the 100 Hz system tick.
         * We approximate the PCLK/2 period with an instruction accumulator
         * scaled from the programmed time constant, so the tick rate tracks
         * what the kernel set up (TC=30000 → ~100 Hz-equivalent cadence). */
        if (m->ct3_running && (m->cio1[0x1E] & 0x80)) {           /* CSC continuous */
            uint32_t tc = ((uint32_t)m->cio1[0x1A]<<8) | m->cio1[0x1B];
            uint32_t period = tc>>1; if (period<2000) period=2000; if (period>60000) period=60000;
            if (++m->ct3_accum >= period) { m->ct3_accum = 0; m->cio1[0x0C] |= 0x20; } /* set IP */
        }

        /* Vectored-interrupt aggregation, in daisy-chain priority order:
         * CIO #1 (CT3 timer) outranks the PDMAC disk completion. */
        bool ct3_int = (m->cio1[0x0C] & 0x20) && (m->cio1[0x0C] & 0x40)   /* CT3 IP & IE */
                    && (m->cio1[0x00] & 0x80);                            /* MICR MIE */
        /* Within the SCC, channel A outranks channel B (chip-internal daisy
         * chain), so a busy wire is serviced ahead of a console keystroke. */
        bool scc_a_int = scc_rxa_int(m);                                  /* Ch A Rx available */
        bool scc_int = scc_rxb_int(m);                                    /* Ch B Rx available */
        if (ct3_int) {
            uint8_t vec = m->cio1[0x04];                                  /* CTIV */
            if (m->cio1[0x00] & 0x04) vec &= ~0x07u;                      /* CTVIS: CT3 → ctNum 0 */
            c->vi_line = true; c->vi_vector = vec; m->last_vi_disk = false;
        } else if (scc_a_int) {
            c->vi_line = true; c->vi_vector = scc_vector_src(m, 0x06); m->last_vi_disk = false;
        } else if (scc_int) {
            c->vi_line = true; c->vi_vector = scc_vector_src(m, 0x02); m->last_vi_disk = false;
        } else if (m->lr_scc && lr_scc_vi(m, &c->vi_vector)) {
            /* The LR board's chips, after the motherboard's and before the
             * disk.  Where the LR board actually sits in the board-level daisy
             * chain is not established anywhere I could find, so this order is
             * a choice, not a finding -- but with no LR board fitted (the
             * default) the branch is never taken and nothing above it moves. */
            c->vi_line = true; m->last_vi_disk = false;
        } else if (m->disk_vi) {
            c->vi_line = true; c->vi_vector = 0x80; m->last_vi_disk = true;
        } else {
            c->vi_line = false;
        }

        cpu_step(c);

        /* The disk VI is a one-shot (cleared when serviced); the CT3 IP is
         * level and is cleared by the clock ISR writing the CS register. */
        if (c->serviced == IRQ_VI && m->last_vi_disk) m->disk_vi = false;

        if (c->halted && !c->vi_line && !m->disk_vi) {
            if (++idle > 2000000) {          /* stuck halted with no wakeup */
                m->stop_why = "the guest halted with no interrupt pending";
                break;
            }
        } else idle = 0;

        /* ...and the same guest with a periodic interrupt still ticking.  The
         * check above is defeated by ANY wakeup source: the tick clears
         * vi_line, `idle' resets, and a guest parked in kboot's
         * `hang: halt / jr hang' spins at 99% CPU forever.  park_step() is
         * that hole closed -- see its comment for why this is a bound on NO
         * PROGRESS and not a clock.  It is a channel of --stop-on like the
         * others, because an IDLE guest wears the same signature as a dead
         * one: a script that sleeps for longer than the budget is working,
         * and a run ended there is a false report of a hang. */
        if (m->park_watch && park_step(&park, m)) break;
    }
    dbg_finish(m);          /* --dump windows, once more, on whatever ended it */
    console_shutdown();
}

/* ── park_selftest (--selftest) ─────────────────────────────────────────────
 *
 * Both directions, because only one of them is the interesting one.  That a
 * parked guest is now caught is easy to show and was shown against a real
 * kboot; that a guest which is WORKING is never caught is the property the
 * bound has to have to be allowed to exist at all, and it is the one a
 * careless threshold quietly breaks.
 *
 * The watch is driven directly rather than through a boot: what is under test
 * is the decision, and a test that has to boot something can only ever try the
 * few shapes someone thought to build a disk for.  Each case drives park_step
 * over a small budget many times, so "never fires" means never over a span
 * many multiples of the budget -- which for the real PARK_LOOPS is hours. */
static bool park_case(const char *name, uint64_t budget, uint64_t iters,
                      void (*tick)(Machine *, uint64_t), bool want, int *ok){
    Machine *m = machine_new();
    CPU *c = &m->cpu;
    ParkWatch p; park_init(&p, budget);
    bool fired = false;
    uint64_t at = 0;
    for (uint64_t i = 0; i < iters && !fired; i++) {
        /* One iteration of the run loop's shape: the guest is halted, except
         * for a brief excursion every 64th iteration -- the tick's interrupt
         * path, exactly the thing that used to reset the old check. */
        bool waking = (i % 64) == 0;
        c->halted = !waking;
        if (waking) tick(m, i);
        if (park_step(&p, m)) { fired = true; at = i; }
    }
    if (fired != want) {
        printf("park FAIL: %s -- %s\n", name,
               want ? "the watch never fired" : "the watch fired");
        if (fired) printf("           it fired at iteration %llu of %llu\n",
                          (unsigned long long)at, (unsigned long long)iters);
        *ok = 0;
    } else {
        printf("park %s PASSED (%s)\n", name,
               want ? "ended" : "ran to the end, never ended");
    }
    free(m->ram); free(m);
    return fired;
}

/* (a) THE PARK.  kboot's `hang: halt / jr hang': the tick wakes it, it does
 * nothing at all, it halts again at the same PC with the same registers.
 * Nothing is printed, nothing is read, no scripted byte moves. */
static void park_guest_parked(Machine *m, uint64_t i){ (void)m; (void)i; }

/* (b) PROGRESS, the direction that protects a long legitimate run.  A guest
 * that halts waiting for its next tick and does REAL WORK on each one -- a
 * register moves.  It is halted for all but 1 iteration in 64, forever, and it
 * must never be called parked, however long it runs. */
static void park_guest_working(Machine *m, uint64_t i){ m->cpu.R[3] = (uint16_t)i; }

/* (c) The same, where the only thing that changes is CONSOLE OUTPUT: a guest
 * printing a progress dot every tick and otherwise idle.  The register file is
 * identical every time, so this is the case that would be missed if the
 * signature were registers alone. */
static void park_guest_printing(Machine *m, uint64_t i){ m->last_tx_insn = i; }

/* (d) A guest that is simply RUNNING -- never halted at all.  The cheapest
 * thing to get wrong is a counter that advances when the CPU is up. */
static void park_guest_running(Machine *m, uint64_t i){ (void)m; (void)i; }

/* ── stop_mark_selftest (--selftest) ────────────────────────────────────────
 *
 * The mark is matched on the guest's console transmit path, so the check
 * drives that path -- scc_write() of WR8 on chip 0 channel 0, which is what
 * the guest executes to print a character -- rather than calling the matcher
 * directly.  Three things have to hold: the run ends on the LAST character of
 * the mark and not before, text that merely resembles the mark does not end
 * it, and a restart mid-mark still matches (a mark may begin with the byte
 * that broke the previous attempt, as "aab" does in "aaab").
 */
static int mark_case_on(const char *name, int chip, int chan, const char *mark,
                        const char *feed, int expect_at, int *ok){
    Machine *m = machine_new();
    m->lr_scc = true;                /* so the LR board's channels answer too */
    m->stop_mark = mark;
    int fired_at = -1;
    for (int i = 0; feed[i] && fired_at < 0; i++) {
        scc_write(m, chip, chan, 8, (uint8_t)feed[i]);
        if (m->stop) fired_at = i;
    }
    /* The fed text has just been printed through the real console path, which
     * is what the case exercises; the verdict starts on its own line. */
    if (fired_at != expect_at) {
        printf("\nstop-mark FAIL: %s -- ended at %d, expected %d\n",
               name, fired_at, expect_at);
        *ok = 0;
    } else if (expect_at < 0)
        printf("\nstop mark %s PASSED (never ended)\n", name);
    else
        printf("\nstop mark %s PASSED (ended on the last character)\n", name);
    free(m->ram); free(m);
    return *ok;
}
/* The console is what nearly every case means, so it keeps the short name. */
static int mark_case(const char *name, const char *mark, const char *feed,
                     int expect_at, int *ok){
    return mark_case_on(name, 0, 0, mark, feed, expect_at, ok);
}

/* Two channels printing at once must not help each other: each carries half of
 * the mark, in an order that would complete it at once if the match position
 * were shared.  Nothing may end the run -- neither channel ever printed the
 * mark itself. */
static void mark_case_crosstalk(const char *mark, const char *feed,
                                int *ok){
    Machine *m = machine_new();
    m->lr_scc = true;
    m->stop_mark = mark;
    int fired_at = -1;
    for (int i = 0; feed[i] && fired_at < 0; i++) {
        /* alternate console (chip 0 chan 0) and the motherboard's other
         * channel (chip 0 chan 1), a character each */
        scc_write(m, 0, i & 1, 8, (uint8_t)feed[i]);
        if (m->stop) fired_at = i;
    }
    if (fired_at >= 0) {
        printf("\nstop-mark FAIL: two channels combined into a mark at %d\n",
               fired_at);
        *ok = 0;
    } else
        printf("\nstop mark crosstalk PASSED (channels never combined)\n");
    free(m->ram); free(m);
}

int stop_mark_selftest(void){
    int ok = 1;
    /* Ends on the mark's last character, with output before and after it. */
    mark_case("caught", "__DONE__", "# echo __DONE__\r\n", 14, &ok);
    /* A prefix of the mark is not the mark. */
    mark_case("prefix spared", "__DONE__", "__DONE and nothing more", -1, &ok);
    /* A restart mid-match: the second 'a' begins the match the first broke. */
    mark_case("restart", "aab", "aaab", 3, &ok);
    /* The same, printed on channels that are NOT the console: a guest that
     * reports on a serial port rather than its terminal ends its run too --
     * the motherboard's second channel, and one on the LR board. */
    mark_case_on("caught on port 1", 0, 1, "__DONE__", "# echo __DONE__\r\n", 14, &ok);
    mark_case_on("caught on the LR board", 1, 0, "__DONE__", "junk__DONE__", 11, &ok);
    mark_case_on("restart on port 1", 0, 1, "aab", "aaab", 3, &ok);
    mark_case_on("prefix spared on port 1", 0, 1, "__DONE__", "__DONE and no more", -1, &ok);
    /* Interleaved on two channels, alternating a character each: "__DONE__"
     * split as "_DN_" on the console and "_OE_" on port 1.  A shared match
     * position would see the mark in the merged stream and end the run. */
    mark_case_crosstalk("__DONE__", "__DONE__", &ok);
    /* Disarmed: no mark, no stop, however much the guest prints. */
    {
        Machine *m = machine_new();
        m->stop_mark = NULL;
        for (const char *s = "__DONE__"; *s; s++) scc_write(m, 0, 0, 8, (uint8_t)*s);
        if (m->stop) { printf("\nstop-mark FAIL: fired with no mark set\n"); ok = 0; }
        else printf("\nstop mark disarmed PASSED (never ended)\n");
        free(m->ram); free(m);
    }
    return ok;
}

int park_selftest(void){
    int ok = 1;
    uint64_t budget = 100000;          /* the real run uses PARK_LOOPS */

    /* Fires, and does not fire early: over one budget it must still be running. */
    park_case("park caught", budget, budget * 4, park_guest_parked, true, &ok);
    {
        Machine *m = machine_new();
        ParkWatch p; park_init(&p, budget);
        m->cpu.halted = true;
        bool early = false;
        for (uint64_t i = 0; i < budget && !early; i++)
            if (park_step(&p, m)) early = true;
        if (early) { printf("park FAIL: fired inside its own budget\n"); ok = 0; }
        else printf("park budget PASSED (silent for a whole budget)\n");
        free(m->ram); free(m);
    }

    /* ...and never fires on a guest that is getting somewhere.  50 budgets:
     * at the real PARK_LOOPS that is 25 billion iterations, hours of running. */
    park_case("working guest spared", budget, budget * 50, park_guest_working,  false, &ok);
    park_case("printing guest spared", budget, budget * 50, park_guest_printing, false, &ok);

    {   /* (d) never halted */
        Machine *m = machine_new();
        ParkWatch p; park_init(&p, budget);
        m->cpu.halted = false;
        bool fired = false;
        for (uint64_t i = 0; i < budget * 50 && !fired; i++) {
            park_guest_running(m, i);
            if (park_step(&p, m)) fired = true;
        }
        if (fired) { printf("park FAIL: fired on a guest that never halted\n"); ok = 0; }
        else printf("park running-guest PASSED (never ended)\n");
        free(m->ram); free(m);
    }
    return ok;
}
