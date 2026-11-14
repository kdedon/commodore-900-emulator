/* The LC colour card: an ID byte and a MOS 8563 VDC with 64 KB of RAM behind
 * it.  The CPU reaches the RAM only through the update address (R18/R19) and
 * the data register (R31).  No picture is drawn; the text screen or the
 * bitmap can be written out at the end of a run. */
#include "emu.h"

#define LC_PORT_ADDR 0x0181          /* write: register select; read: status */
#define LC_PORT_DATA 0x0183
#define LC_PORT_ID   0x01FF
#define LC_ID        0xC1            /* 64 KB card */

/* Bits each register keeps, and the bits that read back as 1. */
static const uint8_t keep[VDC_NREGS] = {
    0xFF,0xFF,0xFF,0xFF,0xFF,0x1F,0xFF,0xFF, 0x03,0x1F,0x7F,0x1F,0xFF,0xFF,0xFF,0xFF,
    0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x1F, 0xFF,0xFF,0xFF,0xFF,0xF0,0x1F,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0x0F,
};

/* The chip's RAM address lines: with R28 bit 4 clear (4416 parts) A14 is not
 * driven and A13:A8 move up one row bit. */
static uint16_t ram_addr(const VDC *v, uint16_t a){
    if (v->reg[28] & 0x10) return a;
    return (uint16_t)((a & 0x8000) | ((a & 0x3F00) << 1) | (a & 0x0100) | (a & 0x00FF));
}

static uint8_t vram_rd(const VDC *v, uint16_t a){ return v->vram[ram_addr(v, a)]; }
static void vram_wr(VDC *v, uint16_t a, uint8_t b){ v->vram[ram_addr(v, a)] = b; }

/* About 60 frames per emulated second, the last fifth of each in blanking. */
static bool in_vblank(Machine *m){
    uint64_t ips = m->rtc.ips ? m->rtc.ips : 1500000;
    uint64_t frame = ips / 60 ? ips / 60 : 1;
    return m->cpu.insns % frame >= frame - frame / 5;
}

/* Character clocks (2 MHz) since the start, at slots_ips instructions a
 * second of the chip's time. */
static uint64_t vdc_now(Machine *m){
    if (m->vdc.hz) return m->cpu.cyc * 2000000ull / m->vdc.hz;
    return m->cpu.insns * 2000000ull / m->vdc.slots_ips;
}

/* Whether the chip's RAM is free for the CPU at character clock t, after the
 * chip's own fetches: in each line the character fetches fill the displayed
 * columns, refresh takes R36+1 columns after them, two columns either side of
 * the line end are never used, and each row's screen and attribute fetches
 * take the first free columns of its lines. */
static bool vdc_free(const VDC *v, uint64_t t){
    unsigned ht = v->reg[0] + 1u, hd = v->reg[1], drr = v->reg[36] & 0x0F;
    unsigned sl = (v->reg[9] & 0x1F) + 1u, rows = v->reg[6];
    unsigned lines = (v->reg[4] + 1u) * sl + (v->reg[5] & 0x1F);
    bool bitmap = v->reg[25] & 0x80, attr = v->reg[25] & 0x40;
    if (hd + drr + 8 >= ht || rows * sl > lines) return true;  /* not set up */
    unsigned col = (unsigned)(t % ht), line = (unsigned)(t / ht % lines);
    if (col < 2 || col >= ht - 3) return false;
    if (col >= hd && col <= hd + drr) return false;
    if (line >= rows * sl) return true;
    if (col < hd) return false;
    unsigned per = ht - 3 - hd - drr - 1;            /* free columns a line */
    unsigned fetch = (bitmap ? 0 : hd) + (attr ? hd : 0);
    unsigned before = line % sl * per + (col - hd - drr - 1);
    return before >= fetch;
}

/* The CPU starts an access of n RAM cycles: it finishes n free columns after
 * the later of now and the end of the last one. */
static void vdc_access(Machine *m, unsigned n){
    VDC *v = &m->vdc;
    if (!v->slots) return;
    uint64_t t = vdc_now(m);
    if (t < v->done) t = v->done;
    v->nwait++;
    while (n) { t++; if (vdc_free(v, t)) n--; }
    v->done = t;
}

static uint8_t vdc_status(Machine *m){
    VDC *v = &m->vdc;
    uint8_t ready = 0x80;
    if (v->slots) {
        uint64_t t = vdc_now(m);
        v->nstat++;
        if (t < v->done) { ready = 0; v->nbusy++; v->pbusy++; v->busycc += v->done - t; }
    }
    return (uint8_t)(ready | (in_vblank(m) ? 0x20 : 0) | 0x01);
}

void lc_slot_report(Machine *m, FILE *out){
    VDC *v = &m->vdc;
    fprintf(out, "lc slots: %llu accesses, %llu status reads, %llu busy\n",
            (unsigned long long)v->nwait, (unsigned long long)v->nstat,
            (unsigned long long)v->nbusy);
}

static uint8_t vdc_read(VDC *v){
    switch (v->sel) {
    case 18: return (uint8_t)(v->ua >> 8);
    case 19: return (uint8_t)v->ua;
    case 31: {
        uint8_t b = v->da;
        v->ua++;
        v->da = vram_rd(v, v->ua);
        return b;
    }
    case 32: return (uint8_t)(v->ba >> 8);
    case 33: return (uint8_t)v->ba;
    }
    if (v->sel >= VDC_NREGS) return 0xFF;
    return (uint8_t)(v->reg[v->sel] | (uint8_t)~keep[v->sel]);
}

static void vdc_write(VDC *v, uint8_t b){
    unsigned n;
    switch (v->sel) {
    case 18: v->ua = (uint16_t)((v->ua & 0x00FF) | b << 8); v->da = vram_rd(v, v->ua); return;
    case 19: v->ua = (uint16_t)((v->ua & 0xFF00) | b);      v->da = vram_rd(v, v->ua); return;
    case 31:
        v->wda = b;
        vram_wr(v, v->ua, b);
        v->ua++;
        v->da = vram_rd(v, v->ua);
        return;
    case 30:                                     /* a count of 0 moves 256 */
        v->reg[30] = b;
        n = b ? b : 256;
        if (v->reg[24] & 0x80) {
            while (n--) vram_wr(v, v->ua++, vram_rd(v, v->ba++));
        } else {
            while (n--) vram_wr(v, v->ua++, v->wda);
        }
        return;
    case 32: v->ba = (uint16_t)((v->ba & 0x00FF) | b << 8); return;
    case 33: v->ba = (uint16_t)((v->ba & 0xFF00) | b);      return;
    }
    if (v->sel < VDC_NREGS) v->reg[v->sel] = b & keep[v->sel];
}

bool lc_io_read(Machine *m, uint16_t port, uint8_t *v){
    if (!m->lc_card) return false;
    switch (port) {
    case LC_PORT_ID:   *v = LC_ID; return true;
    case LC_PORT_ADDR: m->vdc.pstat++; *v = vdc_status(m); return true;
    case LC_PORT_DATA:
        *v = vdc_read(&m->vdc);
        if (m->vdc.sel == 31) vdc_access(m, 1);
        return true;
    }
    return false;
}

bool lc_io_write(Machine *m, uint16_t port, uint8_t v){
    if (!m->lc_card) return false;
    switch (port) {
    case LC_PORT_ADDR: m->vdc.psel++; m->vdc.sel = v & 0x3F; return true;
    case LC_PORT_DATA: {
        uint8_t s = m->vdc.sel;
        unsigned n = v ? v : 256;
        vdc_write(&m->vdc, v);
        if (s == 31) m->vdc.pwr++;
        else if (s == 18 || s == 19) m->vdc.pupd++;
        else if (s == 30) { m->vdc.pblk++; if (m->vdc.reg[24] & 0x80) m->vdc.pcopy += n; else m->vdc.pfill += n; }
        else m->vdc.preg++;
        if (s == 31) vdc_access(m, 2);
        else if (s == 18 || s == 19) vdc_access(m, 1);
        else if (s == 30) vdc_access(m, m->vdc.reg[24] & 0x80 ? 2 * n : n);
        return true;
    }
    case LC_PORT_ID:   return true;
    }
    return false;
}

/* 80x25 characters from the display start, each row R1 + R27 on from the
 * last; trailing blanks dropped, unprintable codes as '.'.  buf needs
 * LC_SCREEN_TEXT bytes. */
void lc_screen_text(const Machine *m, char *buf){
    const VDC *v = &m->vdc;
    uint16_t row = (uint16_t)(v->reg[12] << 8 | v->reg[13]);
    uint16_t stride = (uint16_t)(v->reg[1] + v->reg[27]);
    for (int r = 0; r < 25; r++, row += stride) {
        int len = 0;
        for (int c = 0; c < 80; c++) {
            uint8_t ch = vram_rd(v, (uint16_t)(row + c));
            buf[c] = (ch == 0 || ch == ' ') ? ' ' : (ch > ' ' && ch < 0x7F) ? (char)ch : '.';
            if (buf[c] != ' ') len = c + 1;
        }
        buf += len;
        *buf++ = '\n';
    }
    *buf = '\0';
}

void lc_screen_dump(const Machine *m, FILE *out){
    char buf[LC_SCREEN_TEXT];
    lc_screen_text(m, buf);
    fputs(buf, out);
}

/* RGBI: R, G, B at two thirds, I adds a third to each. */
static void rgbi(uint8_t c, uint8_t px[3]){
    uint8_t i = (c & 1) ? 0x55 : 0;
    px[0] = (uint8_t)(((c & 8) ? 0xAA : 0) + i);
    px[1] = (uint8_t)(((c & 4) ? 0xAA : 0) + i);
    px[2] = (uint8_t)(((c & 2) ? 0xAA : 0) + i);
}

/* The bitmap the chip would show: PBM in monochrome, PPM with attributes.
 * Returns false, writing nothing, when the chip is not in bitmap mode. */
bool lc_bitmap_dump(const Machine *m, FILE *out){
    const VDC *v = &m->vdc;
    if (!(v->reg[25] & 0x80)) return false;
    bool colour = v->reg[25] & 0x40, rev = v->reg[24] & 0x40;
    int cols = v->reg[1], scans = (v->reg[9] & 0x1F) + 1, rows = v->reg[6];
    uint16_t stride = (uint16_t)(v->reg[1] + v->reg[27]);
    uint16_t line = (uint16_t)(v->reg[12] << 8 | v->reg[13]);
    uint16_t attr = (uint16_t)(v->reg[20] << 8 | v->reg[21]);
    uint8_t bg[3], fg[3];
    rgbi(v->reg[26] & 0x0F, bg);
    fprintf(out, colour ? "P6\n%d %d\n255\n" : "P4\n%d %d\n", cols * 8, rows * scans);
    for (int r = 0; r < rows; r++, attr += stride) {
        for (int s = 0; s < scans; s++, line += stride) {
            for (int c = 0; c < cols; c++) {
                uint8_t b = vram_rd(v, (uint16_t)(line + c));
                if (rev) b = (uint8_t)~b;
                if (!colour) { fputc(b, out); continue; }
                rgbi(vram_rd(v, (uint16_t)(attr + c)) & 0x0F, fg);
                for (int x = 7; x >= 0; x--) fwrite(b >> x & 1 ? fg : bg, 1, 3, out);
            }
        }
    }
    return true;
}
