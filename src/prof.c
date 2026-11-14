/* Profiling, armed by C900_PROF: modelled cycles by system-mode PC in 16-byte
 * buckets and the LC card's traffic, written to stderr and cleared each time
 * the console prints C900_PROF_MARK. */
#include "emu.h"
#include <stdlib.h>
#include <string.h>

#define NB (1u << 19)
static uint32_t *pcc;
#define NC (1u << 22)
static uint32_t *pcc, *calls, *sites, *stgt, scn[256];
static uint64_t last, sys, usr, idle, c0, i0, n0, t0;
static uint32_t ustk = 0xFFFF;
static int on = -1;

/* Inclusive cycles by target: a frame per system-mode call, matched to its
 * return by the stack pointer.  A context switch inside a call loses or
 * misattributes that call's time. */
static struct { uint32_t sp, t; uint64_t c0; } fr[512];
static int nfr;
static uint64_t *incl;
static uint32_t spof(CPU *c){ return ((uint32_t)c->R[14] << 16) | c->R[15]; }
/* Calls by exact system-mode target, and system calls by number. */
void prof_call(CPU *c){
    if (on > 0 && (c->fcw & 0x4000)) {
        unsigned i = (c->instr_start & 0x7FFFFF) >> 1;
        calls[(c->pc & 0x7FFFFF) >> 1]++;
        sites[i]++; stgt[i] = c->pc & 0x7FFFFF;
        uint32_t sp = spof(c);
        while (nfr && fr[nfr-1].sp <= sp) nfr--;
        if (nfr < 512) { fr[nfr].sp = sp; fr[nfr].t = c->pc & 0x7FFFFF; fr[nfr].c0 = c->cyc; nfr++; }
    }
}
void prof_ret(CPU *c){
    if (on <= 0 || !(c->fcw & 0x4000)) return;
    uint32_t sp = spof(c);
    while (nfr && fr[nfr-1].sp < sp) nfr--;
    if (nfr && fr[nfr-1].sp == sp) { nfr--; incl[fr[nfr].t >> 1] += c->cyc - fr[nfr].c0; }
}
void prof_sc(CPU *c){
    if (on > 0) scn[c->opcode[0] & 0xFF]++;
}

/* Cycles by process: C900_PROF_PROC is the kernel's current-process pointer
 * (segmented, hex) and C900_PROF_PID the offset of the pid in PROC. */
static uint32_t procva, pidoff = 100;
static struct { uint32_t p; unsigned pid; uint64_t sys, usr; } pt[64];
static int npt;
static uint32_t lread32(Machine *m, uint32_t va){
    uint8_t sn = (va >> 24) & 0x7F; uint16_t off = va & 0xFFFF;
    MMU *u = &m->mmu;
    uint32_t a = ((uint32_t)sn << 16) | off;
    if ((u->mode & 0x80) && (u->mode & 0x40) && ((sn >> 6) & 1) == ((u->mode >> 5) & 1)) {
        MmuDesc *dp = &u->desc[sn & 0x3F];
        a = (((uint32_t)dp->base << 8) + off) & 0xFFFFFF;
    }
    return ((uint32_t)phys_read16(m, a) << 16) | phys_read16(m, a + 2);
}
/* C900_PROF_UPID: user-mode cycles of that pid by PC, in 16-byte buckets. */
static unsigned upid; static uint32_t *upc;
static void proc_charge(Machine *m, uint64_t d, int s){
    uint32_t p = lread32(m, procva);
    int i;
    for (i = 0; i < npt && pt[i].p != p; i++) ;
    if (i == npt) { if (npt == 64) return; npt++; pt[i].p = p; pt[i].sys = pt[i].usr = 0; pt[i].pid = 0; }
    if (!pt[i].pid) pt[i].pid = (unsigned)(lread32(m, p + pidoff) >> 16);
    if (s) pt[i].sys += d; else pt[i].usr += d;
    if (!s && upid && pt[i].pid == upid) upc[(m->cpu.instr_start & 0x7FFFFF) >> 4] += (uint32_t)d;
}

/* HR bitmap watch, C900_HRWATCH=x,y,w,h,period[;x,y,w,h...]: every period
 * cycles (default a 60 Hz frame at 6 MHz) each changed rectangle is appended
 * to C900_HRWATCH_OUT as a frame: "F cycle w h index" then rows of hex words. */
#define NHW 4
static int hw_on = -1, nhw, hx[NHW], hy[NHW], hwd[NHW], hht[NHW];
static uint64_t hper = 100000, hnext;
static uint16_t *hlast[NHW];
static FILE *hout;
void hrwatch_step(Machine *m){
    if (hw_on < 0) {
        const char *e = getenv("C900_HRWATCH"); int per = 0;
        hw_on = 0;
        while (e && nhw < NHW) {
            int k = nhw, n = sscanf(e, "%d,%d,%d,%d,%d", &hx[k], &hy[k], &hwd[k], &hht[k], &per);
            if (n < 4) break;
            if (n == 5 && per > 0) hper = (uint64_t)per;
            hx[k] &= ~15; hwd[k] = (hwd[k] + 15) / 16;
            hlast[k] = calloc((size_t)hwd[k] * hht[k], 2);
            nhw++;
            e = strchr(e, ';'); if (e) e++;
        }
        if (!nhw) return;
        hout = fopen(getenv("C900_HRWATCH_OUT") ? getenv("C900_HRWATCH_OUT") : "hrwatch.txt", "w");
        hw_on = hout != NULL;
    }
    if (!hw_on || m->cpu.cyc < hnext) return;
    hnext = m->cpu.cyc + hper;
    for (int k = 0; k < nhw; k++) {
        int ch = 0;
        for (int y = 0; y < hht[k]; y++)
            for (int x = 0; x < hwd[k]; x++) {
                uint16_t w = phys_read16(m, 0x3E0000 + (uint32_t)(hy[k] + y) * 128 + hx[k] / 8 + x * 2);
                if (w != hlast[k][y * hwd[k] + x]) { hlast[k][y * hwd[k] + x] = w; ch = 1; }
            }
        if (!ch) continue;
        fprintf(hout, "F %llu %d %d %d\n", (unsigned long long)m->cpu.cyc, hwd[k] * 16, hht[k], k);
        for (int y = 0; y < hht[k]; y++) {
            for (int x = 0; x < hwd[k]; x++) fprintf(hout, "%04x", hlast[k][y * hwd[k] + x]);
            fputc('\n', hout);
        }
    }
    fflush(hout);
}

void prof_step(Machine *m){
    if (on < 0) {
        on = getenv("C900_PROF") != NULL;
        if (on && getenv("C900_PROF_PROC")) procva = (uint32_t)strtoul(getenv("C900_PROF_PROC"), 0, 16);
        if (on && getenv("C900_PROF_UPID")) { upid = (unsigned)atoi(getenv("C900_PROF_UPID")); upc = calloc(NB, sizeof *upc); }
        if (on && getenv("C900_PROF_PID")) pidoff = (uint32_t)strtoul(getenv("C900_PROF_PID"), 0, 0);
        if (on) { pcc = calloc(NB, sizeof *pcc); calls = calloc(NC, sizeof *calls); sites = calloc(NC, sizeof *sites); stgt = calloc(NC, sizeof *stgt); incl = calloc(NC, sizeof *incl); }
    }
    if (!on) return;
    uint64_t d = m->cpu.cyc - last;
    last = m->cpu.cyc;
    if (!d) return;
    if (m->cpu.halted) idle += d;
    else if (procva) proc_charge(m, d, m->cpu.fcw & 0x4000);
    if (m->cpu.halted) ;
    else if (m->cpu.fcw & 0x4000) { sys += d; pcc[(m->cpu.instr_start & 0x7FFFFF) >> 4] += (uint32_t)d; }
    else { usr += d; if (m->cpu.R[15] && m->cpu.R[15] < ustk) ustk = m->cpu.R[15]; }
}

void prof_mark(Machine *m){
    VDC *v = &m->vdc;
    extern unsigned long long hr_acc;
    static unsigned long long hr0;
    if (on <= 0) return;
    fprintf(stderr, "[prof] hr bytes %llu\n", hr_acc - hr0);
    hr0 = hr_acc;
    {   /* C900_HRSNAP=PREFIX: the HR bitmap at each mark, raw, PREFIX.N */
        static int nsnap;
        const char *e = getenv("C900_HRSNAP");
        if (e) {
            char name[512];
            snprintf(name, sizeof(name), "%s.%d", e, nsnap++);
            FILE *f = fopen(name, "wb");
            if (f) {
                for (uint32_t a = 0x3E0000; a < 0x400000; a++) putc(m->ram[a], f);
                fclose(f);
            }
        }
    }
    fprintf(stderr, "[prof] cyc %llu sys %llu usr %llu insns %llu io %llu idle %llu ticks %llu ustk %u\n",
        (unsigned long long)(m->cpu.cyc - c0), (unsigned long long)sys,
        (unsigned long long)usr, (unsigned long long)(m->cpu.insns - i0),
        (unsigned long long)(m->cpu.nio - n0), (unsigned long long)idle,
        (unsigned long long)(m->ct3_ticks - t0), 0x10000u - ustk);
    fprintf(stderr, "[prof] lc sel %llu stat %llu busy %llu wr31 %llu upd %llu reg %llu blk %llu fill %llu copy %llu busycc %llu\n",
        (unsigned long long)v->psel, (unsigned long long)v->pstat, (unsigned long long)v->pbusy,
        (unsigned long long)v->pwr, (unsigned long long)v->pupd, (unsigned long long)v->preg,
        (unsigned long long)v->pblk, (unsigned long long)v->pfill, (unsigned long long)v->pcopy,
        (unsigned long long)v->busycc);
    for (int i = 0; i < npt; i++)
        if (pt[i].sys || pt[i].usr) fprintf(stderr, "[prof] proc %08x pid %u sys %llu usr %llu\n", pt[i].p, pt[i].pid,
            (unsigned long long)pt[i].sys, (unsigned long long)pt[i].usr);
    npt = 0;
    if (upc) {
        for (unsigned i = 0; i < NB; i++)
            if (upc[i]) fprintf(stderr, "[prof] upc %02x:%04x %u\n", i >> 12, (i << 4) & 0xFFFF, upc[i]);
        memset(upc, 0, NB * sizeof *upc);
    }
    for (unsigned i = 0; i < NB; i++)
        if (pcc[i]) fprintf(stderr, "[prof] pc %02x:%04x %u\n", i >> 12, (i << 4) & 0xFFFF, pcc[i]);
    for (unsigned i = 0; i < NC; i++)
        if (calls[i]) fprintf(stderr, "[prof] call %02x:%04x %u %llu\n", i >> 15, (i << 1) & 0xFFFF, calls[i], (unsigned long long)incl[i]);
    for (unsigned i = 0; i < NC; i++)
        if (sites[i]) fprintf(stderr, "[prof] site %02x:%04x %02x:%04x %u\n", i >> 15, (i << 1) & 0xFFFF, stgt[i] >> 16, stgt[i] & 0xFFFF, sites[i]);
    for (unsigned i = 0; i < 256; i++)
        if (scn[i]) fprintf(stderr, "[prof] sc %u %u\n", i, scn[i]);
    memset(pcc, 0, NB * sizeof *pcc);
    memset(calls, 0, NC * sizeof *calls);
    memset(incl, 0, NC * sizeof *incl);
    memset(sites, 0, NC * sizeof *sites);
    memset(scn, 0, sizeof scn);
    sys = usr = idle = 0; ustk = 0xFFFF; t0 = m->ct3_ticks; c0 = m->cpu.cyc; i0 = m->cpu.insns; n0 = m->cpu.nio;
    v->psel = v->pstat = v->pbusy = v->pwr = v->pupd = v->preg = v->pblk = v->pfill = v->pcopy = v->busycc = 0;
}
