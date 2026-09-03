/* debug.c — guest-PC breakpoints and memory windows (--break / --dump).
 *
 * WHY THIS EXISTS.  The split-I/D shim regression of 2026-09-02 (the CP/M
 * repository's docs/SPLITID-BISECT.md) was eight instructions in
 * src/bdos/splitfast.s that assembled base and index the wrong way round, so
 * every saved-register access in the SC-trap fast path read and wrote through
 * dispatch scratch.  Two host-side tests exercised the C half of the same shim
 * and passed throughout; nothing on the host executes the assembler half.  It
 * was finally seen by building TEMPORARY instruments into the guest: a dump of
 * the SC trap frame at the fast path's entry, which showed the register file
 * staying at zero across emulated loads while the stack held the right values.
 *
 * That instrument is what this file is.  "Stop at this guest PC and tell me the
 * registers, and what is at this address" needs no guest change, no rebuild and
 * no recompilation of the thing under test — which is exactly the property a
 * debugging instrument must have when the thing under test is what you doubt.
 *
 * CONTRACT.  Everything here is opt-in: with no --break and no --dump the
 * arrays below are empty, dbg_armed is 0, and the one branch in cpu_step's hot
 * path is never taken.  Records go to stderr, one per line, as whitespace-
 * separated key=value fields with a "[dbg]" tag — the consumers are test
 * harnesses and agents, not people, and the CP/M suite already sends the
 * emulator's stderr to /dev/null, so an armed run adds nothing to a transcript
 * anyone else is reading.
 */
#include "emu.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DBG_MAXBRK  8
#define DBG_MAXDUMP 8
#define DBG_MAXLEN  4096

typedef struct { uint32_t pc; long want, seen; } Brk;
/* A window is either absolute (reg < 0: seg:off) or register-relative
 * (reg = an even register number: the long pointer RRn, seg from R[n] as a
 * segment word, offset from R[n+1], plus disp).  Register-relative is the form
 * the trap-frame case needs: the frame is at RR14, wherever that is today. */
typedef struct { int reg; uint8_t seg; uint16_t off; int len; const char *spec; } Win;

static Brk brks[DBG_MAXBRK];  static int nbrk;
static Win wins[DBG_MAXDUMP]; static int nwin;
static bool stop_at_break;

int dbg_armed;            /* nonzero once a --break exists; read by cpu_step */
int dbg_have_dumps(void){ return nwin; }

/* Parse a hex segment as either a plain segment number (0..7F) or a Z8001
 * SEGMENT WORD (seg in bits 14..8), so a PC copied out of a guest trap message
 * -- "pc=B200:3EAC" -- can be pasted in unchanged. */
static uint8_t parse_seg(unsigned long v){ return (uint8_t)((v > 0x7F ? (v >> 8) : v) & 0x7F); }

/* --break=SEG:OFF[/N]  (addresses hex, the hit count N decimal; N=0 = every
 * hit).  Returns 0 on success. */
int dbg_add_break(const char *spec){
    unsigned long sg, of; long n = 1; char *e;
    if (nbrk >= DBG_MAXBRK) { fprintf(stderr,"--break: at most %d breakpoints\n", DBG_MAXBRK); return -1; }
    sg = strtoul(spec, &e, 16);
    if (e == spec || *e != ':') return -1;
    of = strtoul(e+1, &e, 16);
    if (*e == '/') { n = strtol(e+1, &e, 10); }
    if (*e) return -1;
    brks[nbrk].pc   = ((uint32_t)parse_seg(sg) << 16) | (uint16_t)of;
    brks[nbrk].want = n; brks[nbrk].seen = 0;
    nbrk++; dbg_armed = 1;
    return 0;
}

/* --dump=SEG:OFF+LEN or --dump=rrN:DISP+LEN  (addresses and DISP hex, LEN
 * decimal).  DISP may be negative: rr14:-4+16 is the four bytes below the
 * stack pointer and the twelve above it. */
int dbg_add_dump(const char *spec){
    const char *p = spec; int reg = -1; long disp = 0; unsigned long sg = 0, of = 0; char *e;
    if (nwin >= DBG_MAXDUMP) { fprintf(stderr,"--dump: at most %d windows\n", DBG_MAXDUMP); return -1; }
    if ((p[0]=='r'||p[0]=='R') && (p[1]=='r'||p[1]=='R')) {
        reg = (int)strtol(p+2, &e, 10);
        if (e == p+2 || reg < 0 || reg > 14 || (reg & 1) || *e != ':') return -1;
        disp = strtol(e+1, &e, 16);
    } else {
        sg = strtoul(p, &e, 16);
        if (e == p || *e != ':') return -1;
        of = strtoul(e+1, &e, 16);
    }
    if (*e != '+') return -1;
    long len = strtol(e+1, &e, 10);
    if (*e || len <= 0 || len > DBG_MAXLEN) return -1;
    wins[nwin].reg  = reg;
    wins[nwin].seg  = reg < 0 ? parse_seg(sg) : 0;
    wins[nwin].off  = reg < 0 ? (uint16_t)of : (uint16_t)disp;
    wins[nwin].len  = (int)len;
    wins[nwin].spec = spec;
    nwin++;
    return 0;
}

void dbg_stop_at_break(bool on){ stop_at_break = on; }

/* Read one guest byte the way the CPU would, and put back everything the MMU
 * remembers about having done so.  mmu_translate sets REF/CHG in the
 * descriptor and can latch violation state; an observer that left those
 * behind would be changing the run it is there to observe, which is the one
 * thing a debugging instrument may never do.  The MMU is a plain struct with
 * no pointers, so saving and restoring it is exact. */
static uint8_t peek8(Machine *m, uint8_t sn, uint16_t off, bool *fault){
    MMU save = m->mmu;
    bool f = false;
    uint32_t phys = mmu_translate(&m->mmu, sn, off, false, !(m->cpu.fcw & FCW_SN), false, &f);
    m->mmu = save;
    *fault = f;
    return f ? 0xFF : phys_read8(m, phys);
}
static uint32_t peek_phys(Machine *m, uint8_t sn, uint16_t off){
    MMU save = m->mmu; bool f=false;
    uint32_t p = mmu_translate(&m->mmu, sn, off, false, !(m->cpu.fcw & FCW_SN), false, &f);
    m->mmu = save;
    return p;
}

/* Emit every --dump window against the CPU's current state.  `at' says what
 * occasioned it ("brk" or "end") so a harness can tell a mid-run window from
 * the post-mortem one. */
static void dump_windows(Machine *m, const char *at){
    CPU *c = &m->cpu;
    for (int i=0;i<nwin;i++){
        Win *w = &wins[i];
        uint8_t seg; uint16_t off;
        if (w->reg < 0) { seg = w->seg; off = w->off; }
        else            { seg = parse_seg(c->R[w->reg]); off = (uint16_t)(c->R[w->reg+1] + w->off); }
        bool anyfault = false;
        fprintf(stderr, "[dbg] mem at=%s insns=%llu win=%d spec=%s addr=%02X:%04X phys=%06X len=%d data=",
                at, (unsigned long long)c->insns, i, w->spec, seg, off,
                peek_phys(m, seg, off), w->len);
        for (int k=0;k<w->len;k++){
            bool f=false;
            uint8_t b = peek8(m, seg, (uint16_t)(off + k), &f);
            anyfault |= f;
            fprintf(stderr, "%02X", b);
        }
        fprintf(stderr, " fault=%d\n", anyfault ? 1 : 0);
    }
}

/* Called from cpu_step once the instruction at instr_start is decoded and
 * BEFORE it executes, so the state recorded is the state the instruction is
 * about to run on -- entry to a routine means entry, not one instruction in.
 * Returns true if the run should stop here (--stop-on=break), in which case
 * the instruction has not run and PC still points at it. */
bool dbg_pc_hit(CPU *c){
    for (int i=0;i<nbrk;i++){
        Brk *b = &brks[i];
        if (b->pc != c->instr_start) continue;
        if (b->want && b->seen >= b->want) return false;
        b->seen++;
        fprintf(stderr, "[dbg] brk pc=%02X:%04X hit=%ld insns=%llu fcw=%04X sys=%d op=%04X",
                (c->instr_start>>16)&0x7F, c->instr_start&0xFFFF, b->seen,
                (unsigned long long)c->insns, c->fcw, (c->fcw & FCW_SN) ? 1 : 0,
                c->opcode[0]);
        for (int r=0;r<16;r++) fprintf(stderr, " r%d=%04X", r, c->R[r]);
        fprintf(stderr, "\n");
        if (nwin) dump_windows(c->m, "brk");
        if (stop_at_break && b->want && b->seen >= b->want) {
            c->m->stop = true;
            c->m->stop_why = "breakpoint (--stop-on=break)";
            return true;
        }
        return false;
    }
    return false;
}

/* End of run: the post-mortem windows.  Whatever ended the session, "what was
 * in memory when it stopped" is the question a --dump was asked for, and the
 * run that answers it need not have hit a breakpoint at all. */
void dbg_finish(Machine *m){ if (nwin) dump_windows(m, "end"); }
