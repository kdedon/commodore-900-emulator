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

static uint8_t vdc_status(Machine *m){
    return (uint8_t)(0x80 | (in_vblank(m) ? 0x20 : 0) | 0x01);
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
    case LC_PORT_ADDR: *v = vdc_status(m); return true;
    case LC_PORT_DATA: *v = vdc_read(&m->vdc); return true;
    }
    return false;
}

bool lc_io_write(Machine *m, uint16_t port, uint8_t v){
    if (!m->lc_card) return false;
    switch (port) {
    case LC_PORT_ADDR: m->vdc.sel = v & 0x3F; return true;
    case LC_PORT_DATA: vdc_write(&m->vdc, v); return true;
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
