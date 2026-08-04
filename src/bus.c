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
static uint8_t scc_vector_src(Machine *m, uint8_t src){
    uint8_t base = m->scc_b.wr[2];
    if (m->scc_b.wr[9] & 0x10)               /* WR9 D4: Status High */
        return (uint8_t)((base & ~0x70) | (src << 4));
    return (uint8_t)((base & ~0x0E) | (src << 1));
}
/* The vector a Ch-B RR2 read returns: whichever source is currently pending,
 * or the bare base when none is. */
static uint8_t scc_vector(Machine *m){
    if (scc_rxa_int(m)) return scc_vector_src(m, 0x06);
    if (scc_rxb_int(m)) return scc_vector_src(m, 0x02);
    return m->scc_b.wr[2];
}

/* RR3 (read on channel A) exposes the interrupt-pending bits.  D5 = Ch A Rx IP,
 * D2 = Ch B Rx IP. */
static uint8_t scc_rr3(Machine *m){
    return (uint8_t)((scc_rxa_int(m) ? 0x20 : 0) | (scc_rxb_int(m) ? 0x04 : 0));
}

/* The C900's SCC is a Z8030 (Z-Bus).  Register addressing is a hybrid: the
 * effective register is AD4:AD1 (passed here as addrReg), UNLESS the channel's
 * register pointer (ch->ptr, set via a WR0 write) is non-zero, in which case it
 * overrides for this one access and then auto-resets.  chan=1 → Channel A
 * (AD5 high, scc_a), chan=0 → Channel B (the console, scc_b). */
static uint8_t scc_read(Machine *m, int chan, int addrReg){
    SCCChan *ch = chan ? &m->scc_a : &m->scc_b;
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
        if (chan == 0 && !ch->rx_avail) {
            m->rx_poll_streak++;
            if (m->rx_poll_streak >= RX_BLOCKED_POLLS) m->guest_polls = true;
        }
        return v;
    }
    case 8:                           /* RR8 = Rx data (reading it clears rx) */
        ch->rx_avail = false;
        if (chan == 0) {
            m->rx_poll_streak = 0;
            m->last_tx_insn = m->cpu.insns;   /* console activity: restart the quiet clock */
        }
        return ch->rx_data;
    case 1: return 0x01;              /* RR1: all-sent, no errors */
    case 2: return chan == 0 ? scc_vector(m) : ch->wr[2];  /* RR2 (Ch B): modified vector */
    case 3: return chan == 1 ? scc_rr3(m) : 0;             /* RR3 (Ch A): IP bits */
    default: return ch->wr[r & 0x0F];
    }
}
static void scc_write(Machine *m, int chan, int addrReg, uint8_t v){
    SCCChan *ch = chan ? &m->scc_a : &m->scc_b;
    int r = ch->ptr ? ch->ptr : (addrReg & 0x0F);
    ch->ptr = 0;
    if (r == 0) {                            /* WR0: command + register pointer */
        uint8_t ptr = v & 0x07;
        uint8_t cmd = (v >> 3) & 0x07;
        ch->ptr = (cmd == 1) ? (ptr | 0x08) : ptr;  /* cmd 1 = Point High (+8) */
        /* Commands 2-7 (reset ext/status, arm-rx-int, reset-Tx-IP, error-reset,
         * reset-highest-IUS) have no state we model — we fire the Rx interrupt
         * directly off rx_avail and don't track IUS. */
        return;
    }
    if (r == 8) {                            /* WR8 = Tx data */
        if (chan == 1) wire_put_char(v);     /* channel A → the host endpoint */
        if (chan == 0) {
            console_put_char(v);
            m->last_tx_insn = m->cpu.insns;
            m->rx_poll_streak = 0;
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
     * shared by both channels — keep our two structs coherent. */
    if (r == 2 || r == 9) { m->scc_a.wr[r] = v; m->scc_b.wr[r] = v; }
}

/* deliver a byte to the console channel B receiver */
void scc_rx_console(Machine *m, uint8_t b){
    m->scc_b.rx_data = b; m->scc_b.rx_avail = true;
}

/* deliver a byte to the channel A receiver (the second RS-232 port, /dev/tty51) */
void scc_rx_wire(Machine *m, uint8_t b){
    m->scc_a.rx_data = b; m->scc_a.rx_avail = true;
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
    /* SCC 0x0100-0x017F, on D15:D8 */
    if ((port & 0xFF80) == 0x0100) {
        int reg = (port >> 1) & 0x0F;
        int chan = (port >> 5) & 1;           /* 1→A, 0→B */
        return (uint16_t)scc_read(m, chan, reg) << 8;
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
    if ((port & 0xFF80) == 0x0100) {          /* SCC (D15:D8) */
        int reg = (port >> 1) & 0x0F;
        int chan = (port >> 5) & 1;
        scc_write(m, chan, reg, (uint8_t)(data >> 8));
        return;
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

void machine_run(Machine *m){
    CPU *c = &m->cpu;
    mmu_reset(&m->mmu);
    cpu_reset(c);
    console_init();

    uint64_t idle = 0;
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
            if (m->inq_pos < m->inq_len && m->shell_up &&
                (past_gate || m->prompt_seq >= m->inq_wait_seq) &&
                (c->insns - m->last_tx_insn) > quiet) {
                uint8_t b = m->inq[m->inq_pos++];
                scc_rx_console(m, b);
                if (getenv("C900_FEED_DEBUG"))
                    fprintf(stderr, "[feed %02x '%c' insns=%llu streak=%u polls=%d quiet=%llu]\n",
                            b, (b>=32&&b<127)?b:'.', (unsigned long long)c->insns,
                            m->rx_poll_streak, m->guest_polls, (unsigned long long)quiet);
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

        /* Channel A receiver: take the next byte from the host endpoint only
         * once the guest has read the previous one.  The real chip has a
         * three-deep FIFO and a byte arriving faster than the driver services
         * the Rx interrupt is simply lost; here the endpoint holds it instead,
         * so the wire cannot overrun the guest and SLIP frames arrive whole. */
        if (m->wire_on && !m->scc_a.rx_avail && (m->tick_counter & 0x3F) == 0) {
            int wb = wire_poll_char();
            if (wb >= 0) scc_rx_wire(m, (uint8_t)wb);
        }

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
    }
    console_shutdown();
}
