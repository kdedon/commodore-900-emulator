/* uexec.c — run a linked Z8001 `l.out' as a PROCESS instead of booting a board.
 *
 * The rest of this emulator models a Commodore 900: ROM at physical 0, 1 MB of
 * DRAM in segments 8..0x17, and peripherals everywhere else.  A compiled
 * program is a different shape entirely — it links into whatever segment ld
 * gave it, its stack lives in segment 0, and its idea of the outside world is
 * the COHERENT system-call trap — so this file gives it its own front end:
 *
 *   -runobjint <l.out> [args...]   load the image, CALL f(args...), print R1
 *   -runobj    <l.out> [a b want [bits]]  the same with 64/32-bit float operands
 *   -runexec/--exec <l.out> [args...]     run crt0+main against a host-backed
 *                                         COHERENT syscall shim
 *
 * The machine path is untouched: everything here runs on a Machine with
 * flat_mem set (RAM over the whole 24-bit space, no ROM and no device decode)
 * and the MMU left in its reset pass-through state, so a logical seg:off is the
 * physical address seg<<16|off and nothing this file does can be reached from a
 * disk boot.
 *
 * The reference for the syscall semantics is the toolchain's Go harness,
 * tools/go/n2z8001/runsim.go; where a comment here says "the kernel does X" the
 * citation is that file's, and the numbering is sys/z8001/src/tab.c.  The one
 * rule that overrides convenience everywhere below: an unimplemented call must
 * FAIL LOUDLY.  A plausible-looking return value from a call that did nothing is
 * the failure mode a test oracle may not have.
 */
/* The syscall shim services the guest's calls with the host's
 * open/read/link/mknod/utime/opendir, which is the whole point of it.  hostfs.h
 * is where those calls come from: on a POSIX host it is the include list and
 * the feature-test macros that expose them under -std=c99, and on Windows it
 * supplies the ones the C runtime spells differently, refuses, or would perform
 * in text mode. */
#include "hostfs.h"

#include "emu.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

/* ─────────────────────────── COHERENT constants ───────────────────────────
 * errno values (include/errno.h).  The kernel does not return these: trap.c
 * stores u.u_error into the user word at MUERR (= ADDR(USTACK, 0xFFFE)) and
 * returns -1 in R1; libc's `errno' is the absolute symbol errno_ = 0x0000FFFE.
 * Anything that fails here has to write that word too, or the guest reads
 * whatever was there before — sbrk(3) is the sharp case, since it clears errno,
 * calls brk(2) and decides purely on errno whether the break was refused. */
#define cEPERM 1
#define cENOENT 2
#define cEINTR 4
#define cEIO 5
#define cENXIO 6
#define cEBADF 9
#define cEAGAIN 11
#define cENOMEM 12
#define cEACCES 13
#define cEFAULT 14
#define cEEXIST 17
#define cEXDEV 18
#define cENOTDIR 20
#define cEISDIR 21
#define cEINVAL 22
#define cEMFILE 24
#define cENOTTY 25
#define cENOSPC 28
#define cESPIPE 29
#define cEROFS 30
#define cEMLINK 31
#define cEPIPE 32

/* File-type bits of an i-node mode (include/sys/ino.h), spelled out rather than
 * taken from the host's S_IF* because the guest's header is the definition
 * mknod(2) is being matched against. */
#define mIFMT   0170000
#define mIFPIPE 0010000
#define mIFCHR  0020000
#define mIFDIR  0040000
#define mIFBLK  0060000
#define mIFREG  0100000

#define ERRNO_OFF 0xFFFE       /* MUERR: `errno' in the user stack segment */

/* Layout of the user stack segment (segment 0) that run_exec builds:
 *
 *   0x0040     bootstrap stub            0x0060 IRET / 0x0062 HALT stubs
 *   .. USER_SP the program's stack, growing DOWN from USER_SP
 *   USER_SP    argc, argv, envp (the frame crt0 reads)
 *   ARGDESC    argv[] then envp[] pointer arrays, sized to the actual counts
 *   ..         argv/envp string pool, growing UP from the end of those arrays
 *              (POOL_BASE is its floor, not its base)
 *   POOL_LIMIT hard ceiling — the guard band below errno
 *   ERRNO_OFF  errno
 */
#define USER_SP    0xCF00
#define ARGDESC    0xCF10
#define POOL_BASE  0xD000
#define POOL_LIMIT 0xFF00

#define USTR_MAX 8192          /* bound on a user-space string scan */
#define LOUT_MAGIC 0407        /* struct ldheader's l_magic (include/l.out.h) */

/* ─────────────────────────── harness plumbing ─────────────────────────── */

static Machine *UM;            /* the machine the loaded image runs on */
static int inst_max;           /* highest segment this run installed (see u_ok) */

/* harness_fatal reports a bug in THIS harness, not in the guest program, and
 * stops.  Everything that reaches it would otherwise be a silent wrong answer. */
static void harness_fatal(const char *fmt, ...){
    va_list ap; va_start(ap, fmt);
    fputs("[c900: HARNESS BUG: ", stderr);
    vfprintf(stderr, fmt, ap);
    fputs("]\n", stderr);
    va_end(ap);
    exit(4);
}

/* lout_bad reports a file that is not a loadable l.out and stops.  The header
 * fields are sizes and file offsets that every caller immediately slices the
 * file with, so an unchecked header turns any non-l.out input into a wild read
 * naming a byte range rather than the file that could not be loaded. */
static void lout_bad(const char *path, const char *fmt, ...){
    va_list ap; va_start(ap, fmt);
    fprintf(stderr, "c900: %s: not a loadable l.out: ", path);
    vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
    va_end(ap);
    exit(1);
}

/* ─────────────────────────── flat guest memory ───────────────────────────
 * A logical (segment, offset) is the physical address segment<<16|offset: the
 * MMU is left in its reset pass-through state and flat_mem turns off the board
 * decode, so these accessors and the CPU's own see exactly the same bytes. */
static uint8_t *segp(int seg){ return UM->ram + (((uint32_t)(seg & 0xFF)) << 16); }
static void gw8(int seg, int off, uint8_t v){ segp(seg)[off & 0xFFFF] = v; }
static uint8_t gr8(int seg, int off){ return segp(seg)[off & 0xFFFF]; }
static void gw16(int seg, int off, uint16_t v){ gw8(seg,off,(uint8_t)(v>>8)); gw8(seg,off+1,(uint8_t)v); }
static uint16_t gr16(int seg, int off){ return (uint16_t)gr8(seg,off)<<8 | gr8(seg,off+1); }
static void gw32(int seg, int off, uint32_t v){ gw16(seg,off,(uint16_t)(v>>16)); gw16(seg,off+2,(uint16_t)v); }

/* u_addr resolves byte k of a user buffer to (segment, offset), carrying a
 * 16-bit offset overflow into the segment number the way the kernel's vadd()
 * does — it does not wrap back to offset 0.  The segment is 7 bits, so a carry
 * past 0x7F leaves the address space entirely and is reported as 0x100, which
 * has no memory. */
static void u_addr(uint32_t v, int k, int *seg, int *off){
    int o = (int)(v & 0xFFFF) + k;
    int s = (int)((v >> 24) & 0x7F) + (o >> 16);
    if (s > 0x7F) s = 0x100;
    *seg = s; *off = o & 0xFFFF;
}
/* u_mapped: does this segment have memory behind it in this run? */
static bool u_mapped(int seg){ return seg <= inst_max; }
/* u_range_ok: does every byte of [v, v+n) have memory behind it?  Checked once
 * per segment the range touches rather than once per byte. */
static bool u_range_ok(uint32_t v, int n){
    for (int pos = 0; pos < n; ) {
        int seg, off; u_addr(v, pos, &seg, &off);
        if (!u_mapped(seg)) return false;
        pos += 0x10000 - off;
    }
    return true;
}
static uint8_t u_r8(uint32_t v, int k){ int s,o; u_addr(v,k,&s,&o); return gr8(s,o); }
static void u_w8(uint32_t v, int k, uint8_t b){ int s,o; u_addr(v,k,&s,&o); gw8(s,o,b); }

/* u_str reads a NUL-terminated string from user space into buf, following the
 * same carry rule.  The scan is bounded: an unterminated string, or a pointer
 * into unmapped memory, reports failure rather than walking the address space. */
static bool u_str(uint32_t v, char *buf, size_t bufsz){
    for (size_t k = 0; k < USTR_MAX && k + 1 < bufsz; k++) {
        int seg, off; u_addr(v, (int)k, &seg, &off);
        if (!u_mapped(seg)) return false;
        uint8_t c = gr8(seg, off);
        buf[k] = (char)c;
        if (c == 0) return true;
    }
    return false;
}

/* ─────────────────────────── l.out header ─────────────────────────── */

/* lout_hdr describes a parsed header in either on-file dialect: the NATIVE
 * PDP-canonical form (tbase 48: LE shorts, 32-bit fields hi-word-first each-LE,
 * 22-byte symbol records) that the C toolchain emits, or the retired host-LP64
 * form (tbase 88: BE shorts, 8-byte size fields, 32-byte symbol records) so
 * pre-switch artifacts stay loadable.  The dialect is keyed off the header-size
 * field, which is BE-vs-LE unambiguous (48 = 0x0030/0x3000, 88 = 0x0058/0x5800). */
typedef struct {
    int tbase, flag;
    int ss[9];
    int symrec;        /* symbol record stride: 22 native, 32 legacy */
    bool native;
    uint8_t entry_seg; /* l_entry: virtual seg:offset */
    uint16_t entry_off;
} LoutHdr;

/* A section GROUP is one segment's worth of image: a list of (file offset,
 * offset within the segment, length) pieces, plus the bss that follows them.
 * Pieces rather than one base+size because under LF_SHR a segment's two
 * sections are not adjacent in the file. */
typedef struct { int foff, moff, len; } LoutPiece;
typedef struct { LoutPiece p[2]; int np; int span; int bss; } LoutGroup;
/* The resolved layout: which groups exist, which segment each lands in, and
 * where in that segment it starts. */
typedef struct { LoutGroup g[4]; uint8_t seg[4]; int base[4]; int ng, last_seg; } LoutLayout;
int lout_layout(const LoutHdr *h, LoutLayout *L, char *err, size_t errn);

/* lout_check verifies the section sizes against the file: each is non-negative
 * and text+data+symbols lie inside it.  These are the extents every caller
 * slices the image with, and a size read out of a file that is not an l.out is
 * an arbitrary 32-bit number. */
static void lout_check(size_t len, const char *path, LoutHdr *h){
    for (int i = 0; i < 9; i++)
        if (h->ss[i] < 0) lout_bad(path, "section %d size %d is negative", i, h->ss[i]);
    long end = (long)h->tbase + h->ss[0] + h->ss[4] + h->ss[7];
    if (end < 0 || end > (long)len)
        lout_bad(path, "header describes %ld bytes (text %d + data %d + symbols %d at %d) "
                       "but the file is %ld", end, h->ss[0], h->ss[4], h->ss[7], h->tbase, (long)len);
}

static LoutHdr lout_parse(const uint8_t *b, size_t len, const char *path){
    LoutHdr h; memset(&h, 0, sizeof h);
    const int minhdr = 88;   /* the larger of the two dialects' header sizes */
    if (len < (size_t)minhdr)
        lout_bad(path, "%zu bytes is shorter than an l.out header", len);
    #define LE16(o) ((int)b[o] | (int)b[(o)+1]<<8)
    #define BE16(o) ((int)b[o]<<8 | (int)b[(o)+1])
    if (LE16(6) == 48) {                       /* native PDP-canonical */
        if (LE16(0) != LOUT_MAGIC)
            lout_bad(path, "magic %#o, want %#o", LE16(0), LOUT_MAGIC);
        h.tbase = 48; h.symrec = 22; h.native = true;
        h.flag = LE16(2);
        for (int i = 0; i < 9; i++)            /* pdp32: hi word first, each LE */
            h.ss[i] = LE16(8+4*i)<<16 | LE16(8+4*i+2);
        h.entry_seg = (uint8_t)((LE16(44)>>8) & 0x7F);
        h.entry_off = (uint16_t)LE16(46);
        lout_check(len, path, &h);
        return h;
    }
    /* legacy host dialect: BE shorts, 8-byte sizes (value BE in the low 4) */
    if (BE16(0) != LOUT_MAGIC || BE16(6) != 88)
        lout_bad(path, "magic %#o header-size %d, want %#o and 48 (native) or 88 (legacy)",
                 BE16(0), BE16(6), LOUT_MAGIC);
    h.tbase = BE16(6); h.symrec = 32; h.native = false;
    h.flag = BE16(2);
    for (int i = 0; i < 9; i++)
        h.ss[i] = BE16(8+8*i)<<16 | BE16(8+8*i+2);
    h.entry_seg = b[80] & 0x7F;
    h.entry_off = (uint16_t)BE16(82);
    lout_check(len, path, &h);
    return h;
    #undef LE16
    #undef BE16
}

/* sym_addr reads a symbol record's seg:offset value (record at o). */
static void sym_addr(const LoutHdr *h, const uint8_t *b, int o, uint8_t *seg, uint16_t *off){
    if (h->native) {          /* ls_addr pdp32 at +18: [hi_lo hi_hi lo_lo lo_hi] */
        *seg = b[o+19] & 0x7F;
        *off = (uint16_t)b[o+20] | (uint16_t)b[o+21]<<8;
    } else {
        *seg = b[o+24] & 0x7F;
        *off = (uint16_t)b[o+26]<<8 | b[o+27];
    }
}

static uint8_t *read_file(const char *path, size_t *len){
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "c900: %s: %s\n", path, strerror(errno)); exit(1); }
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    if (n < 0) { fprintf(stderr, "c900: %s: cannot size\n", path); exit(1); }
    uint8_t *b = malloc((size_t)n + 1);
    if (fread(b, 1, (size_t)n, f) != (size_t)n) { fprintf(stderr, "c900: %s: short read\n", path); exit(1); }
    fclose(f);
    *len = (size_t)n;
    return b;
}

/* ─────────────────────────── the machine under a process ─────────────────── */

/* seg_reg_word: the segment half of a far-pointer register pair (Z8001 segment
 * in bits 14:8) — used to seed the stack-segment register R14. */
static uint16_t seg_reg_word(uint8_t seg){ return (uint16_t)seg << 8; }
/* seg_call_addr: the first word of a segmented long address operand (present
 * bit + segment) for a CALL/JP into `seg'. */
static uint16_t seg_call_addr(uint8_t seg){ return 0x8000 | ((uint16_t)seg << 8); }

/* umach_new builds the Machine every mode here runs on: flat RAM, a reset
 * pass-through MMU, no ROM, no disk, no console.  inst_max is how far the
 * "installed" address space reaches, which is what the EFAULT checks read. */
static void umach_new(int maxseg){
    UM = machine_new();
    UM->flat_mem = true;
    UM->flat_maxseg = (uint8_t)maxseg;
    inst_max = maxseg;
}

/* urun steps the CPU until it halts or the budget runs out, and reports a guest
 * FAULT distinguishably from either.  Nothing in a user-mode run installs the
 * EPU/privileged/segment-trap vectors, so any of those pending at an
 * instruction boundary means the guest executed something it could not: saying
 * so names the instruction, where vectoring through an uninitialised PSA entry
 * would silently jump to segment 0 offset 0 and run the argv pool as code.
 * Returns 0 normally, 1 on a fault, 2 on budget exhaustion. */
static int urun(uint64_t budget, volatile bool *stop_flag){
    CPU *c = &UM->cpu;
    for (uint64_t i = 0; i < budget; i++) {
        if (c->halted) return 0;
        if (stop_flag && *stop_flag) return 0;
        if (c->irq_req & (IRQ_EPU|IRQ_TRAP|IRQ_SEGTRAP)) {
            const char *what = (c->irq_req & IRQ_EPU) ? "illegal/extended instruction"
                             : (c->irq_req & IRQ_TRAP) ? "privileged instruction"
                             : "segment trap";
            fprintf(stderr, "[c900: GUEST FAULT: %s at %02X:%04X (opcode %04X) "
                            "after %llu instructions]\n",
                    what, (c->instr_start>>16)&0x7F, c->instr_start & 0xFFFF,
                    c->cur_op, (unsigned long long)c->insns);
            return 1;
        }
        cpu_step(c);
    }
    fprintf(stderr, "[c900: instruction budget (%llu) exhausted at %02X:%04X]\n",
            (unsigned long long)budget, (UM->cpu.pc>>16)&0x7F, UM->cpu.pc & 0xFFFF);
    return 2;
}

/* seg_image loads a LINKED l.out for the CALL-into-f modes.  Segment 0 holds
 * the reset PSA, the caller stub and the stack — cc2 frame access is seg-0
 * short-form X-mode, so stack locals live in segment 0, matching the real C900
 * (CODE seg 3 / STACK seg 0).  The CODE segment (read from entry symbol `name's
 * linked address; 0 for a flat link) holds text followed by the PRVD data and
 * literal pool: n2 places globals AND the far-pointer literal pool (the
 * {seg,offset} constants a `gp=&global' loads) in PRVD, not BSS, so skipping the
 * data load leaves those zero and a far pointer reads {0,0}.  A flat link loads
 * text at 0x200 (ld -R 0x200); a segmented link bases its code segment at 0. */
static void seg_image(const uint8_t *b, size_t len, const char *path, const char *name,
                      uint8_t *code_seg, uint16_t *entry){
    LoutHdr h = lout_parse(b, len, path);
    int text_size = h.ss[0], data_size = h.ss[4], sym_size = h.ss[7];
    const uint8_t *text = b + h.tbase;
    const uint8_t *data = b + h.tbase + text_size;

    uint8_t cseg = 0; uint16_t ent = 0; bool have = false;
    int symbase = h.tbase + text_size + data_size;
    size_t nlen = strlen(name);
    for (int o = symbase; o + h.symrec <= symbase + sym_size; o += h.symrec) {
        char nm[17];
        memcpy(nm, b + o, 16); nm[16] = 0;
        /* C symbols may carry the COHERENT trailing '_' */
        if (!strcmp(nm, name) || (strlen(nm) == nlen+1 && !strncmp(nm, name, nlen) && nm[nlen]=='_')) {
            sym_addr(&h, b, o, &cseg, &ent);
            have = true;
            break;
        }
    }
    int textoff = 0x200;
    if (cseg != 0) textoff = 0;
    if (!have) ent = (uint16_t)textoff;

    /* Text and the PRVD pool go into ONE segment here, so an object whose two
     * sections do not fit a 64K page cannot be run in this mode at all. */
    if (textoff + text_size + data_size > 0x10000)
        lout_bad(path, "text %d + data %d at offset %#x does not fit the single code segment "
                       "this mode loads into", text_size, data_size, textoff);

    umach_new(cseg > 0 ? cseg : 0);
    for (int i = 0; i < text_size; i++) gw8(cseg, textoff + i, text[i]);
    for (int i = 0; i < data_size; i++) gw8(cseg, textoff + text_size + i, data[i]);
    gw16(0, 0x0002, 0xC000);   /* reset FCW: SEG | system mode */
    gw16(0, 0x0004, 0x0000);   /* reset PC segment (the stub lives in segment 0) */
    *code_seg = cseg; *entry = ent;
}

/* ─────────────────────────── -runobjint ───────────────────────────
 * Run a LINKED a.out (ld -R 0x200 -e f) whose f() takes zero or more int-width
 * arguments and returns a 16-bit int in R1 — the FULL deliverable pipeline
 * (cc0->cc1->cc2->ld->execute).  Prints `R1 = N (signed M)'. */
int run_objint(const char *path, int argc, char **argv){
    host_stdio_binary();
    size_t len; uint8_t *b = read_file(path, &len);
    uint8_t code_seg; uint16_t entry;
    seg_image(b, len, path, "f", &code_seg, &entry);

    /* Caller stub in segment 0: set the stack (R14=seg0, R15=SP), push each
     * argument as one 16-bit word RIGHT-TO-LEFT so the leftmost lands lowest
     * (the callee's prolog then finds them at FP+6, FP+8, ...), CALL f in its
     * code segment (the segmented CALL pushes the full seg:offset return
     * address, so f's RET returns across segments to the HALT), HALT.  One word
     * per argument is what a compiled call site pushes: these entries take
     * int-width parameters and K&R promotion widens a char parameter to int. */
    uint16_t stub[64]; int n = 0;
    stub[n++] = 0x210E; stub[n++] = seg_reg_word(0);   /* LD R14,#seg0 */
    stub[n++] = 0x210F; stub[n++] = 0xE000;            /* LD R15,#0xE000 */
    for (int i = argc - 1; i >= 0; i--) {
        char *end; long v = strtol(argv[i], &end, 0);
        if (end == argv[i] || *end)
            harness_fatal("-runobjint: argument \"%s\" is not a number", argv[i]);
        if (n + 3 > 60) harness_fatal("-runobjint: too many arguments for the caller stub");
        stub[n++] = 0x2100; stub[n++] = (uint16_t)v; stub[n++] = 0x93F0; /* LD R0,#arg; PUSH @R15,R0 */
    }
    stub[n++] = 0x5F00; stub[n++] = seg_call_addr(code_seg); stub[n++] = entry; /* CALL f */
    stub[n++] = 0x7A00;                                                        /* HALT */
    for (int i = 0; i < n; i++) gw16(0, 0x0040 + i*2, stub[i]);
    gw16(0, 0x0006, 0x0040);   /* reset PC -> stub */

    cpu_reset(&UM->cpu);
    urun(20000000ull, NULL);
    /* Both readings of the same 16 bits: `R1 = N' unsigned (what every existing
     * caller greps) and the signed interpretation beside it, because a function
     * returning a negative int leaves 0xFFF9 in R1 and an unsigned-only report
     * reads 65529 — a difference in the HARNESS, not in the code under test. */
    uint16_t r1 = UM->cpu.R[1];
    printf("R1 = %u (signed %d)\n", r1, (int)(int16_t)r1);
    free(b);
    return 0;
}

/* ─────────────────────────── -runobj (float) ─────────────────────────── */
int run_floatobj(const char *path, uint64_t a, uint64_t bb, uint64_t want, int retbits){
    host_stdio_binary();
    size_t len; uint8_t *b = read_file(path, &len);
    uint8_t code_seg; uint16_t entry;
    seg_image(b, len, path, "f", &code_seg, &entry);

    uint16_t stub[64]; int n = 0;
    stub[n++] = 0x210E; stub[n++] = seg_reg_word(0);
    stub[n++] = 0x210F; stub[n++] = 0xE000;
    /* push b (rightmost) then a, 8 bytes each big-endian (high word at the
     * lowest address -> push the low word first, the high word last) */
    uint64_t vals[2] = { bb, a };
    for (int j = 0; j < 2; j++)
        for (int s = 0; s < 4; s++) {
            stub[n++] = 0x2100; stub[n++] = (uint16_t)(vals[j] >> (s*16)); stub[n++] = 0x93F0;
        }
    stub[n++] = 0x5F00; stub[n++] = seg_call_addr(code_seg); stub[n++] = entry;
    stub[n++] = 0x7A00;
    for (int i = 0; i < n; i++) gw16(0, 0x0040 + i*2, stub[i]);
    gw16(0, 0x0006, 0x0040);

    cpu_reset(&UM->cpu);
    urun(200000ull, NULL);
    uint16_t *R = UM->cpu.R;
    uint64_t got; const char *reg;
    if (retbits == 32) { got = (uint64_t)R[0]<<16 | R[1]; reg = "RR0"; }
    else { got = (uint64_t)R[0]<<48 | (uint64_t)R[1]<<32 | (uint64_t)R[2]<<16 | R[3]; reg = "RQ0"; }
    printf("f(%#llx, %#llx) -> %s = %#llx  (want %#llx)  %s\n",
           (unsigned long long)a, (unsigned long long)bb, reg,
           (unsigned long long)got, (unsigned long long)want,
           got == want ? "PASS" : "FAIL");
    free(b);
    return 0;
}

/* ═══════════════════════════ -runexec: the syscall shim ═══════════════════ */

/* retClass is the RETURN WIDTH the kernel's system-call table declares for a
 * call (sys/z8001/src/tab.c, second column).  It is not cosmetic: trap.c
 * returns an INT in R1 and a LONG/PTR in RR0 (R0:R1), so a 32-bit result handed
 * back through the INT path keeps only its low half.  Writing the same 16-bit
 * value to BOTH R0 and R1 is correct for an INT and only ACCIDENTALLY correct
 * for a LONG whose value is 0 or -1.  Hence two helpers, and a table that makes
 * using the wrong one a harness bug rather than a wrong answer. */
typedef enum { rcINT, rcWIDE } RetClass;
static RetClass class_of(uint8_t num){
    switch (num) {
    case 17: /* brk     -> char *   (PTR) */
    case 19: /* lseek   -> long          */
    case 45: /* unique  -> long          */
    case 48: /* signal  -> int (*)() PTR */
    case 72: /* alarm2  -> long          */
    case 73: /* tick    -> long          */
        return rcWIDE;
    default:
        return rcINT;
    }
}

/* sys_name names the calls a gap message or an accept-list entry may need to
 * spell out.  It covers everything this harness does not service, and keeps the
 * entries for calls it has SINCE implemented (mknod, setuid, setgid) so a
 * caller carrying an old N2ACCEPT list forward finds them harmlessly accepted
 * rather than rejected as names that do not exist. */
static const char *sys_name(uint8_t n){
    switch (n) {
    case 2: return "fork";   case 7: return "wait";   case 11: return "exec";
    case 14: return "mknod"; case 21: return "mount"; case 22: return "umount";
    case 23: return "setuid";case 25: return "stime"; case 26: return "ptrace";
    case 27: return "alarm"; case 29: return "pause"; case 34: return "nice";
    case 37: return "kill";  case 41: return "dup";   case 42: return "pipe";
    case 43: return "times"; case 44: return "profil";case 45: return "unique";
    case 46: return "setgid";case 51: return "acct";  case 53: return "lock";
    case 61: return "chroot";case 62: return "setpgrp"; case 63: return "getpgrp";
    case 66: return "fcntl"; case 67: return "poll";  case 72: return "alarm2";
    case 73: return "tick";
    default: return NULL;
    }
}
static const char *describe_sys(uint8_t n){
    static char buf[32];
    const char *nm = sys_name(n);
    if (nm) snprintf(buf, sizeof buf, "%d (%s)", n, nm);
    else    snprintf(buf, sizeof buf, "%d", n);
    return buf;
}

/* default_accept is the set of unimplemented calls a run may hit WITHOUT being
 * stopped.  Every member is here for one of two reasons and no others:
 *
 *  - it cannot exist in this harness at all.  There is one process and one
 *    loaded image, so fork/wait/exec/pipe have nothing to return, and acct(2)
 *    would create an accounting file nothing would ever write a record into.
 *
 *  - it is deliberately LEFT failing rather than faked: a fake alarm that never
 *    fires followed by a pause() that never returns hangs the run forever,
 *    which is strictly worse than -1.
 *
 * Anything NOT here stops the run, because the alternative is a program that
 * quietly computed the wrong answer — dup(2) returning -1 makes the next
 * close(2) shut the caller's only copy, and that looks like a result. */
static bool default_accept(uint8_t n){
    switch (n) {
    case 2: case 7: case 11: case 42:   /* no second process: fork/wait/exec/pipe */
    case 27: case 29: case 37:          /* deliberately loud: alarm/pause/kill */
    case 51:                            /* acct: no process accounting exists here */
        return true;
    default: return false;
    }
}

/* The unimplemented-call policy.  Default: strict, with default_accept applied.
 *   N2ACCEPT=fork,dup,14   also accept these (names or numbers)
 *   N2ACCEPT=all           accept everything — never stop
 *   N2ACCEPT=none          accept nothing, not even default_accept
 *   N2STRICT=0             accept everything;  N2STRICT=1 accept nothing */
static bool accept_set[128];
static bool accept_all;
static bool sys_num_of(const char *s, uint8_t *out){
    while (*s == ' ' || *s == '\t') s++;
    char t[64]; size_t i = 0;
    while (*s && *s != ' ' && *s != '\t' && i + 1 < sizeof t) t[i++] = *s++;
    t[i] = 0;
    for (int n = 0; n < 74; n++) {
        const char *nm = sys_name((uint8_t)n);
        if (nm && !strcmp(nm, t)) { *out = (uint8_t)n; return true; }
    }
    char *end; long v = strtol(t, &end, 10);
    /* NMICALL is 74 (sys/param.h): a number outside the table cannot be issued
     * by any program, so accepting one would only hide a typo. */
    if (end != t && !*end && v >= 0 && v < 74) { *out = (uint8_t)v; return true; }
    return false;
}
static void policy_init(void){
    for (int n = 0; n < 128; n++) accept_set[n] = default_accept((uint8_t)n);
    const char *s = getenv("N2STRICT");
    if (s && *s) {
        if (!strcmp(s,"0") || !strcmp(s,"no") || !strcmp(s,"off")) accept_all = true;
        else memset(accept_set, 0, sizeof accept_set);   /* historical N2STRICT=1 */
    }
    const char *a = getenv("N2ACCEPT");
    if (a && *a) {
        char *dup = strdup(a);
        for (char *f = strtok(dup, ","); f; f = strtok(NULL, ",")) {
            while (*f == ' ') f++;
            if (!*f) continue;
            if (!strcmp(f,"all")) { accept_all = true; continue; }
            if (!strcmp(f,"none")) { memset(accept_set, 0, sizeof accept_set); continue; }
            uint8_t n;
            if (!sys_num_of(f, &n))
                harness_fatal("N2ACCEPT: \"%s\" is not a system call number or name", f);
            accept_set[n] = true;
        }
        free(dup);
    }
}
static bool policy_ok(uint8_t n){ return accept_all || (n < 128 && accept_set[n]); }

/* coh_errno maps a host errno onto the COHERENT one a guest would have seen.
 * An unrecognised failure becomes EIO rather than 0: the one answer that must
 * never come out of a failed call is "no error". */
static uint16_t coh_errno(int e){
    switch (e) {
    case EPERM: return cEPERM;   case ENOENT: return cENOENT;
    case EINTR: return cEINTR;   case ENXIO:  return cENXIO;
    case ENODEV: return cENXIO;  case EBADF:  return cEBADF;
    case EAGAIN: return cEAGAIN; case ENOMEM: return cENOMEM;
    case EACCES: return cEACCES; case EFAULT: return cEFAULT;
    case EEXIST: return cEEXIST; case EXDEV:  return cEXDEV;
    case ENOTDIR: return cENOTDIR; case EISDIR: return cEISDIR;
    case EINVAL: return cEINVAL; case EMFILE: return cEMFILE;
    case ENFILE: return cEMFILE; case ENOTTY: return cENOTTY;
    case ENOSPC: return cENOSPC; case ESPIPE: return cESPIPE;
    case EROFS:  return cEROFS;  case EMLINK: return cEMLINK;
    case EPIPE:  return cEPIPE;
    default: return cEIO;
    }
}

/* guest_ino folds a host inode number into the 16-bit ino_t a COHERENT
 * filesystem uses.  Faithful is impossible — the host number does not fit — but
 * the fold preserves the property every caller depends on: two names for one
 * file agree, and two different files almost never do.  A CONSTANT fails that
 * every time, which makes cp/mv/ln treat every pair of files as the same file
 * and stops getwd(3) at the first directory it looks at.  0 is reserved: it
 * marks a free slot in a directory. */
static uint16_t guest_ino(uint64_t ino){
    uint64_t h = ino * 2654435761ull + (ino >> 32);
    uint16_t v = (uint16_t)(h >> 16) ^ (uint16_t)h;
    return v ? v : 1;
}
static uint16_t host_dev(dev_t d){
    return (uint16_t)((d & 0xFF) << 8) | (uint16_t)((d >> 8) & 0xFF);
}

/* gfile is one guest file descriptor.  A directory has no host descriptor to
 * read from (the host refuses read(2) on a directory), so it carries the
 * synthesized COHERENT directory image and its own position instead. */
typedef struct {
    bool used;
    int  fd;          /* -1 for a synthesized directory */
    uint8_t *dir;     /* COHERENT `struct direct' image, NULL for a plain file */
    long dirlen, pos;
    char path[1024];
} GFile;
#define MAXFD 256
static GFile gfiles[MAXFD];

static int alloc_fd(void){
    /* open(2) hands back the LOWEST free descriptor, which is what the
     * `close(0); open(f)' redirect idiom depends on; a monotonic counter hands
     * back 3 and the program reads the wrong file. */
    for (int fd = 0; fd < MAXFD; fd++) if (!gfiles[fd].used) return fd;
    return -1;
}
static GFile *get_fd(int fd){
    if (fd < 0 || fd >= MAXFD || !gfiles[fd].used) return NULL;
    return &gfiles[fd];
}

/* dir_image builds the on-disk COHERENT directory a program sees when it
 * read(2)s a directory: 16-byte `struct direct' records (sys/dir.h), a 2-byte
 * inode followed by a 14-byte NUL-padded name, with "." and ".." first.  libc
 * here has no opendir/getdents — ls, find, du, sh's globber and getwd all open
 * the directory and read the raw records — so without this a directory read
 * returns 0, which is indistinguishable from an empty directory.
 *
 * d_ino is stored in PDP-canonical (little-endian) order on disk, NOT in
 * machine order: every reader byte-swaps it with canino() on the way in.
 * Writing it big-endian left getwd(3) comparing a swapped inode against the
 * unswapped one stat(2) reports, so no entry of ".." ever matched. */
static uint16_t ino_of(const char *p){
    struct stat st;
    return host_lstat(p, &st) == 0 ? guest_ino(host_ino(p, &st)) : 1;
}
static uint8_t *dir_put(uint8_t *buf, size_t *cap, size_t *n, uint16_t ino, const char *name){
    if (*n + 16 > *cap) { *cap *= 2; buf = realloc(buf, *cap); }
    memset(buf + *n, 0, 16);
    buf[*n] = (uint8_t)ino; buf[*n+1] = (uint8_t)(ino >> 8);
    size_t l = strlen(name); if (l > 14) l = 14;
    memcpy(buf + *n + 2, name, l);
    *n += 16;
    return buf;
}
static uint8_t *dir_image(const char *path, long *outlen){
    DIR *d = opendir(path);
    if (!d) return NULL;
    size_t cap = 64*16, n = 0;
    uint8_t *buf = malloc(cap);
    char sub[2048];
    int skipped = 0;
    buf = dir_put(buf, &cap, &n, ino_of(path), ".");
    snprintf(sub, sizeof sub, "%s/..", path);
    buf = dir_put(buf, &cap, &n, ino_of(sub), "..");
    struct dirent *e;
    while ((e = readdir(d))) {
        if (!strcmp(e->d_name, ".") || !strcmp(e->d_name, "..")) continue;
        /* A COHERENT name is 14 bytes with no terminator, so a longer host name
         * has no representation.  Truncating it would put a name in the
         * directory that stat(2) cannot resolve, and getwd(3) -- which stats
         * every entry of ".." until one matches -- gives up at the first such
         * entry.  Leaving it out is the only answer that stays consistent; the
         * count is reported so the omission is not silent. */
        if (strlen(e->d_name) > 14) { skipped++; continue; }
        snprintf(sub, sizeof sub, "%s/%s", path, e->d_name);
        buf = dir_put(buf, &cap, &n, ino_of(sub), e->d_name);
    }
    closedir(d);
    if (skipped)
        fprintf(stderr, "[c900: %s: %d name(s) longer than DIRSIZ=14 omitted from the directory]\n",
                path, skipped);
    *outlen = (long)n;
    return buf;
}

/* is_tty reports whether a descriptor is a terminal on the host.  ioctl(2) and
 * therefore isatty(3) answer from this: reporting "terminal" unconditionally
 * for fds 0/1/2 makes ls(1) columnate into a pipe, more(1) paginate a
 * redirected file and stdio pick line buffering when the real system would not
 * — a wrong answer that looks like a plausible one. */
static bool is_tty(GFile *g){ return g && g->fd >= 0 && isatty(g->fd); }

/* write_stat_buf lays a host stat into a guest `struct stat' (sys/stat.h) at the
 * far pointer sbp.  The COHERENT struct is 30 bytes, all fields big-endian:
 *   +0 dev(2) +2 ino(2) +4 mode(2) +6 nlink(2) +8 uid(2) +10 gid(2)
 *   +12 rdev(2) +14 size(4) +18 atime(4) +22 mtime(4) +26 ctime(4)
 * st_mode carries the S_IF* type in its high bits, which is what callers test
 * first (a pager rejects a directory, an archiver rejects anything not
 * regular).  The whole 30 bytes are faulted on up front, so a buffer that
 * straddles the end of its segment reports failure instead of indexing past it. */
static bool write_stat_buf(uint32_t sbp, const struct stat *st, const char *path){
    const int statsz = 30;
    if (!u_range_ok(sbp, statsz)) return false;
    for (int k = 0; k < statsz; k++) u_w8(sbp, k, 0);
    #define P16(o,v) do{ u_w8(sbp,(o),(uint8_t)((v)>>8)); u_w8(sbp,(o)+1,(uint8_t)(v)); }while(0)
    #define P32(o,v) do{ u_w8(sbp,(o),(uint8_t)((v)>>24)); u_w8(sbp,(o)+1,(uint8_t)((v)>>16)); \
                         u_w8(sbp,(o)+2,(uint8_t)((v)>>8)); u_w8(sbp,(o)+3,(uint8_t)(v)); }while(0)
    uint16_t mode = (uint16_t)(st->st_mode & 0777);
    if (st->st_mode & S_ISUID) mode |= 0004000;
    if (st->st_mode & S_ISGID) mode |= 0002000;
    if (st->st_mode & S_ISVTX) mode |= 0001000;
    if      (S_ISDIR(st->st_mode))  mode |= mIFDIR;
    else if (S_ISCHR(st->st_mode))  mode |= mIFCHR;
    else if (S_ISBLK(st->st_mode))  mode |= mIFBLK;
    else if (S_ISFIFO(st->st_mode)) mode |= mIFPIPE;
    else                            mode |= mIFREG;
    uint16_t rdev = 0;
    /* st_rdev distinguishes one device special file from another.  Leaving it 0
     * for every one of them makes ttyname(3) — which scans /dev for a matching
     * rdev — name the first character device it finds. */
    if (S_ISCHR(st->st_mode) || S_ISBLK(st->st_mode))
        rdev = (uint16_t)(((st->st_rdev >> 8) & 0xFF) << 8) | (uint16_t)(st->st_rdev & 0xFF);
    /* st_size is a 32-bit fsize_t on the guest, so a host file over 4G cannot be
     * described at all.  Report the ceiling rather than the low 32 bits: a
     * wrapped size reads as a small file and a caller sized its buffer from it. */
    unsigned long long sz = (unsigned long long)st->st_size;
    if (sz > 0xFFFFFFFFull) sz = 0xFFFFFFFFull;
    uint32_t mt = (uint32_t)st->st_mtime;
    P16(0,  host_dev(st->st_dev));
    P16(2,  guest_ino(host_ino(path, st)));
    P16(4,  mode);
    P16(6,  (uint16_t)st->st_nlink);
    P16(12, rdev);
    P32(14, (uint32_t)sz);
    P32(18, mt);
    P32(22, mt);
    P32(26, mt);
    #undef P16
    #undef P32
    return true;
}

/* ── the live state of the running process ── */
static bool     exec_exited;
static int      exec_code;
static uint32_t brk_addr;
static int      heap_hi;
static int16_t  guest_uid, guest_gid;
static bool     warned_syscall[128];
static int      n_warned;
static const char *n2root;

/* put_str appends one argv/envp string to the pool and returns its seg-0
 * address. */
static int str_off;
static uint32_t put_str(const char *s){
    int l = (int)strlen(s);
    if (str_off + l + 1 > POOL_LIMIT)
        harness_fatal("argv/envp string pool overflow at %#x (limit %#x, %d bytes of strings): "
                      "the command line does not fit below the errno word at %#x",
                      str_off, POOL_LIMIT, str_off - POOL_BASE, ERRNO_OFF);
    int at = str_off;
    for (int i = 0; i < l; i++) gw8(0, at + i, (uint8_t)s[i]);
    gw8(0, at + l, 0);
    str_off += l + 1;
    return (uint32_t)at;
}

/* remap_path: target absolute data paths resolve against N2ROOT (default: none)
 * so a program finds the datfiles the image would have had. */
static void remap_path(const char *p, char *out, size_t n){
    if (n2root && *n2root && p[0] == '/') snprintf(out, n, "%s%s", n2root, p);
    else snprintf(out, n, "%s", p);
}

/* The call currently being serviced.  sys_hook's return/failure helpers are
 * file-scope functions rather than closures, so these three carry the context
 * they would otherwise have captured. */
static CPU *cur_cpu;
static uint8_t cur_num;
static int cur_sseg;

/* ret_int returns an INT-class result.  R1 is the kernel's answer; R0 gets the
 * same word (it is caller-save, so no guest can tell).  ret_wide returns a
 * LONG/PTR-class result in RR0 (R0 = high half).  The class check is what stops
 * a 32-bit answer being handed back through the 16-bit path, where it would
 * silently keep only its low half. */
static void ret_int(int16_t v){
    if (class_of(cur_num) != rcINT)
        harness_fatal("syscall %d is LONG/PTR in tab.c but returned through the INT helper", cur_num);
    cur_cpu->R[0] = (uint16_t)v; cur_cpu->R[1] = (uint16_t)v;
}
static void ret_wide(uint32_t v){
    if (class_of(cur_num) != rcWIDE)
        harness_fatal("syscall %d is INT in tab.c but returned through the LONG/PTR helper", cur_num);
    cur_cpu->R[0] = (uint16_t)(v >> 16); cur_cpu->R[1] = (uint16_t)v;
}
/* fail delivers the kernel's error return: -1 AND the errno word at
 * USTACK:0xFFFE.  It is the one path that is class-independent -- all-ones is -1
 * as an int and as a long -- so it writes both halves directly.  Nothing here
 * clears errno on success; neither does the kernel.  The errno word lives in the
 * stack segment, which comes from the guest's own R14, so the store is checked
 * like any other user-space write. */
static void fail(uint16_t code){
    if (u_mapped(cur_sseg)) gw16(cur_sseg, ERRNO_OFF, code);
    cur_cpu->R[0] = 0xFFFF; cur_cpu->R[1] = 0xFFFF;
}
static void fail_err(void){ fail(coh_errno(errno)); }

/* path_arg reads a pathname out of user space and maps it into the host tree.
 * It reports false when the pointer does not address readable memory or the
 * string is unterminated -- the kernel's EFAULT -- so a wild pointer becomes an
 * error return rather than an empty pathname that would turn into a
 * plausible-looking ENOENT. */
static bool path_arg(uint32_t v, char *out, size_t n){
    char raw[2048];
    if (!u_str(v, raw, sizeof raw)) return false;
    remap_path(raw, out, n);
    return true;
}

/* How many segments an image wants is a property of the section layout, not of
 * one flag.  The authority is the kernel's exec(2) (coh/exec.c), which switches
 * on l_flag & (LF_SHR|LF_SEP) and allocates a different number per arm:
 *
 *   0                one segment:  [si][pi][sd][pd] then bi+bd of bss
 *   LF_SHR           two:          shared [si][sd]
 *                                  private [pi][pd] then bi+bd of bss
 *   LF_SEP           two:          text [si][pi] + bi,  data [sd][pd] + bd
 *   LF_SHR|LF_SEP    four:         [si] [pi]+bi [sd] [pd]+bd
 *
 * LF_SHR is the common case here: the June 1985 Mark Williams passes are all
 * l_flag 025 = LF_SHR|LF_NRB|LF_32.  With sd == 0 that arm puts text alone in
 * the shared segment and data alone, based at 0, in the private one.
 *
 * In the file the four initialised sections are adjacent in the order si, pi,
 * sd, pd from l_tbase; the memory offsets differ per arm, and under LF_SHR the
 * shared segment's pieces are not contiguous in the file.  So a group is a list
 * of (file offset, offset within segment, length) pieces, not one base+size.
 *
 * (exec.c's LF_SEP arm reads the data group from sh+si+bi.  bi is a BSS size
 * and occupies no file bytes; sd is at sh+si+pi.  We follow the file, which the
 * LF_SHR arm agrees with.  The two differ only when a link has both pi and bi
 * non-zero, which nothing here does.)
 *
 * A group with nothing in it gets NO segment: ssalloc() returns NULL for a zero
 * size (coh/seg.c:49) without touching the segment queues, and uproto() skips a
 * NULL slot without advancing the hardware segment number (z8001/commodore.c).
 * So a group's segment is its position among the NON-EMPTY groups.
 *
 * Returns the number of groups, or -1 with `err' filled in.
 */
int lout_layout(const LoutHdr *h, LoutLayout *L, char *err, size_t errn){
    enum { L_SHRI = 0, L_PRVI = 1, L_BSSI = 2, L_SHRD = 3, L_PRVD = 4, L_BSSD = 5 };
    int si = h->ss[L_SHRI], pi = h->ss[L_PRVI], bi = h->ss[L_BSSI];
    int sd = h->ss[L_SHRD], pd = h->ss[L_PRVD], bd = h->ss[L_BSSD];
    int f_si = h->tbase, f_pi = f_si + si, f_sd = f_pi + pi, f_pd = f_sd + sd;

    LoutGroup g[4]; int ng = 0;
    memset(g, 0, sizeof g);
    memset(L, 0, sizeof *L);
    #define PIECE(gi, fo, mo, ln) do { if ((ln) > 0) { \
            g[gi].p[g[gi].np].foff = (fo); g[gi].p[g[gi].np].moff = (mo); \
            g[gi].p[g[gi].np].len  = (ln); g[gi].np++; } \
            if ((mo) + (ln) > g[gi].span) g[gi].span = (mo) + (ln); } while (0)
    switch (h->flag & 03) {                /* LF_SHR = 01, LF_SEP = 02 */
    case 0:
        ng = 1;
        PIECE(0, f_si, 0, si + pi + sd + pd);
        g[0].bss = bi + bd;
        break;
    case 01:                               /* LF_SHR */
        ng = 2;
        PIECE(0, f_si, 0,  si); PIECE(0, f_sd, si, sd);
        PIECE(1, f_pi, 0,  pi); PIECE(1, f_pd, pi, pd);
        g[1].bss = bi + bd;
        break;
    case 02:                               /* LF_SEP */
        ng = 2;
        PIECE(0, f_si, 0, si + pi); g[0].bss = bi;
        PIECE(1, f_sd, 0, sd + pd); g[1].bss = bd;
        break;
    default:                               /* LF_SHR|LF_SEP */
        ng = 4;
        PIECE(0, f_si, 0, si);
        PIECE(1, f_pi, 0, pi); g[1].bss = bi;
        PIECE(2, f_sd, 0, sd);
        PIECE(3, f_pd, 0, pd); g[3].bss = bd;
        break;
    }
    #undef PIECE

    /* Drop the empty groups, keeping order: what survives is what ssalloc()
     * would have returned a segment for. */
    for (int i = 0; i < ng; i++)
        if (g[i].span > 0 || g[i].bss > 0)
            L->g[L->ng++] = g[i];
    if (L->ng == 0) {
        snprintf(err, errn, "no section has any content");
        return -1;
    }

    /* Group 0 holds the entry, and crt0's `start' is first in the text section,
     * so the entry OFFSET is that group's base within its segment.  Every later
     * group is based at 0, which is what makes a far pointer into data carry the
     * data segment number and nothing else. */
    /* Multi-segment groups (`ld -L'): a section >64K spans consecutive hardware
     * segments — ptov rolls the flat offset into the segment field at each 64K
     * boundary.  Byte i of a group based at `bs' in segment `s' lands at
     * segment s+(bs+i)/65536, offset (bs+i)%65536. */
    int seg_n = (int)h->entry_seg;
    for (int i = 0; i < L->ng; i++) {
        L->base[i] = (i == 0) ? (int)h->entry_off : 0;
        L->seg[i]  = (uint8_t)seg_n;
        int total = L->base[i] + L->g[i].span + L->g[i].bss;
        int nseg  = (total + 0xFFFF) / 0x10000;
        if (nseg < 1) nseg = 1;
        /* A group is contiguous virtual memory, so a group that does not fit its
         * segments has nowhere to go: only a MULTI-segment group may exceed 64K,
         * and only because ld linked it that way.  The check exists because the
         * segment number is a uint8 downstream — a wrapped one loads the image
         * over the PSA in segment 0 and the run appears to start normally. */
        if (seg_n + nseg - 1 > 0x7F) {
            snprintf(err, errn, "section group %d needs segments %d..%d, past the "
                     "7-bit segment space", i, seg_n, seg_n + nseg - 1);
            return -1;
        }
        seg_n += nseg;
    }
    L->last_seg = seg_n - 1;
    return L->ng;
}

/* ── lout_layout regression (run by --selftest) ────────────────────────────
 * The case each row pins down is the SEGMENT NUMBER a group lands in, because
 * that is what ld already bound the program's far pointers to and what the
 * loader has no second chance to get right: an image loaded into the wrong
 * segments runs, and simply addresses nothing.
 *
 * Row 4 is the bug this exists for.  A `ld -n -i' C program has si=4662,
 * pd=276 and no shared text or shared data at all, so of LF_SHR|LF_SEP's four
 * arms two are empty -- and giving those empty arms segments of their own put
 * data in segment 6 when every reference to it says segment 4.  Before the
 * fix that row reports data in segment 6 and the program printed nothing at
 * all while still exiting with the right status. */
int lout_layout_selftest(void){
    struct { const char *name; int flag; int ss[6]; int want_ng; int want_seg[4]; } t[] = {
      /*                       si   pi  bi   sd   pd  bd                          */
      { "flag 0 one segment",   000, {4662, 0, 0,   0, 276, 12}, 1, {3} },
      { "LF_SHR text+data",     001, {4662, 0, 0,   0, 276, 12}, 2, {3,4} },
      { "LF_SEP text+data",     002, {4662, 0, 0,   0, 276, 12}, 2, {3,4} },
      { "LF_SHR|LF_SEP, no shared sections",
                                003, {4662, 0, 0,   0, 276, 12}, 2, {3,4} },
      { "LF_SHR|LF_SEP, all four present",
                                003, {1000, 200, 8, 300, 276, 12}, 4, {3,4,5,6} },
      { "LF_SEP, text over 64K (ld -L)",
                                002, {70000, 0, 0,  0, 276, 12}, 2, {3,5} },
    };
    int ok = 1;
    for (size_t i = 0; i < sizeof t / sizeof t[0]; i++) {
        LoutHdr h; memset(&h, 0, sizeof h);
        h.tbase = 48; h.native = true; h.symrec = 22;
        h.flag = t[i].flag;
        for (int k = 0; k < 6; k++) h.ss[k] = t[i].ss[k];
        h.entry_seg = 3; h.entry_off = 0;
        LoutLayout L; char err[256];
        int ng = lout_layout(&h, &L, err, sizeof err);
        if (ng != t[i].want_ng) {
            printf("lout_layout FAIL: %s: %d groups, expected %d%s%s\n",
                   t[i].name, ng, t[i].want_ng, ng < 0 ? ": " : "", ng < 0 ? err : "");
            ok = 0; continue;
        }
        for (int k = 0; k < ng; k++)
            if (L.seg[k] != t[i].want_seg[k]) {
                printf("lout_layout FAIL: %s: group %d in segment %d, expected %d\n",
                       t[i].name, k, L.seg[k], t[i].want_seg[k]);
                ok = 0;
            }
        /* The data segment is the LAST group's -- what crt0 stores `environ'
         * in and where sbrk starts -- so name it explicitly rather than trust
         * that checking every group covered it. */
        if (ok && L.seg[ng-1] != t[i].want_seg[t[i].want_ng-1]) {
            printf("lout_layout FAIL: %s: data segment %d\n", t[i].name, L.seg[ng-1]);
            ok = 0;
        }
    }
    if (ok) printf("l.out layout checks PASSED\n");
    return ok;
}

static void sys_hook(Machine *m, uint8_t num, uint32_t pc);

/* ─────────────────────────── -runexec ─────────────────────────── */
int run_exec(const char *path, int argc, char **argv){
    host_stdio_binary();
    size_t len; uint8_t *b = read_file(path, &len);
    LoutHdr h = lout_parse(b, len, path);
    uint8_t entry_seg = h.entry_seg; uint16_t entry_off = h.entry_off;

    LoutLayout L; char lerr[256];
    if (lout_layout(&h, &L, lerr, sizeof lerr) < 0) lout_bad(path, "%s", lerr);
    int ng = L.ng, last_n = L.last_seg;
    LoutGroup *g = L.g; uint8_t *gseg = L.seg; int *gbase = L.base;
    /* Install through code+data plus heap headroom: sbrk carries the break into
     * the segments after the last group (a large link/compile grows past 64K). */
    heap_hi = last_n + 8;
    if (heap_hi > 0xFF) heap_hi = 0xFF;
    umach_new(heap_hi);

    for (int i = 0; i < ng; i++)
        for (int k = 0; k < g[i].np; k++) {
            const LoutPiece *pc = &g[i].p[k];
            if (pc->foff < 0 || (size_t)(pc->foff + pc->len) > len)
                lout_bad(path, "section group %d piece %d is file bytes %d..%d of a "
                               "%ld-byte file", i, k, pc->foff, pc->foff + pc->len, (long)len);
            for (int j = 0; j < pc->len; j++) {
                int a = gbase[i] + pc->moff + j;
                gw8(gseg[i] + (a >> 16), a & 0xFFFF, b[pc->foff + j]);
            }
        }
    /* The names the rest of this function still speaks in.  The DATA segment is
     * the last group's: that is where crt0 stores `environ' and where the break
     * begins, in every one of the four arms. */
    uint8_t data_seg = gseg[ng-1];
    int dbase = gbase[ng-1];

    /* PSA (segment 0): reset vector -> bootstrap stub; syscall vector (offset
     * 0x0C, doubled to 0x18 in segmented mode) -> IRET stub.  A Z8001 PSA entry
     * is [reserved(2)][FCW(2)][seg(2)][off(2)]. */
    gw16(0, 0x0002, 0xC000);   /* reset FCW = SEG | system */
    gw16(0, 0x0004, 0x0000);   /* reset PC segment */
    gw16(0, 0x0006, 0x0040);   /* reset PC offset -> bootstrap stub */
    gw16(0, 0x001A, 0xC000);   /* syscall handler FCW */
    gw16(0, 0x001C, 0x0000);   /* syscall handler PC segment */
    gw16(0, 0x001E, 0x0060);   /* syscall handler PC offset -> IRET stub */

    /* argc/argv/envp on the user stack (segment 0).  A stored far pointer is
     * [segword][offword]; a seg-0 pointer to offset X is the 32-bit value X. */
    /* argv[0] is the path the binary was invoked as, not a placeholder: a
     * multi-name binary switches on it (gzip/gunzip/zcat are one program), and a
     * placeholder makes that dispatch untestable. */
    int nprog = argc + 1;
    /* environment: from N2ENV ("TERM=ansi:HOME=/" style, colon-separated) */
    char *envcopy = NULL; char *envv[64]; int nenv = 0;
    const char *e = getenv("N2ENV");
    if (e && *e) {
        envcopy = strdup(e);
        for (char *t = strtok(envcopy, ":"); t && nenv < 64; t = strtok(NULL, ":")) envv[nenv++] = t;
    }
    /* The two pointer arrays are sized to the actual argument counts and laid
     * end to end below the string pool.  Fixed-size arrays let a 27-argument
     * command line run argv straight through envp's first pointer — and a
     * compiler driver command line reaches 27 arguments. */
    int argv_arr = ARGDESC;
    int envp_arr = argv_arr + (nprog + 1) * 4;
    /* The string pool starts where the pointer arrays END, rather than at a
     * fixed POOL_BASE.  A fixed base capped the two arrays at 60 pointers
     * between ARGDESC and POOL_BASE, and `ar cr libc.a *.o' passes 170 — a
     * count a build reaches routinely and a hand run never does.  POOL_BASE
     * survives only as the FLOOR, so a short command line lays out exactly
     * where it always did. */
    str_off = envp_arr + (nenv + 1) * 4;
    if (str_off < POOL_BASE) str_off = POOL_BASE;
    /* The pool grows UP toward `errno' at 0xFFFE.  That word is really written
     * (every failing syscall stores an errno there), so a long enough command
     * line would walk over it silently and the next failing call would appear
     * to have succeeded.  put_str stops at POOL_LIMIT; the arrays themselves
     * have to be checked here, because nothing writes them through put_str. */
    if (str_off > POOL_LIMIT)
        harness_fatal("argv(%d)+envp(%d) pointer arrays need %#x..%#x, past the "
                      "guard band at %#x", nprog, nenv, ARGDESC, str_off, POOL_LIMIT);
    gw32(0, argv_arr, put_str(path));
    for (int i = 0; i < argc; i++) gw32(0, argv_arr + (i+1)*4, put_str(argv[i]));
    gw32(0, argv_arr + nprog*4, 0);           /* argv[argc] = NULL */
    for (int i = 0; i < nenv; i++) gw32(0, envp_arr + i*4, put_str(envv[i]));
    gw32(0, envp_arr + nenv*4, 0);            /* envp[nenv] = NULL */
    gw16(0, USER_SP + 0, (uint16_t)nprog);
    gw32(0, USER_SP + 2, (uint32_t)argv_arr);
    gw32(0, USER_SP + 6, (uint32_t)envp_arr);

    /* bootstrap stub (segment 0): set the stack (RR14 = seg0:USER_SP), jump to
     * the entry. */
    uint16_t stub[] = { 0x210E, 0x0000, 0x210F, USER_SP,
                        0x5E08, (uint16_t)(0x8000 | ((uint16_t)entry_seg << 8)), entry_off };
    for (unsigned i = 0; i < sizeof stub/sizeof stub[0]; i++) gw16(0, 0x0040 + i*2, stub[i]);
    gw16(0, 0x0060, 0x7B00);   /* IRET stub: return from the SC trap to after the SC */
    gw16(0, 0x0062, 0x7A00);   /* HALT stub: exit repoints the syscall vector here */

    /* fds 0/1/2 are real descriptors like any other, so read/write/lseek/fstat/
     * ioctl/close all take one path.  Special-casing them leaves lseek(1,...)
     * and fstat(0,...) failing on a redirected stdin/stdout that the real system
     * handles, and makes ioctl answer "terminal" for them always. */
    for (int i = 0; i < 3; i++) {
        gfiles[i].used = true; gfiles[i].fd = i;
        snprintf(gfiles[i].path, sizeof gfiles[i].path, "/dev/std%s",
                 i == 0 ? "in" : i == 1 ? "out" : "err");
    }
    policy_init();
    n2root = getenv("N2ROOT");
    /* The guest's credentials.  It starts as root, which is the identity every
     * COHERENT image is guaranteed to have a passwd entry for, and setuid/setgid
     * move it exactly as usetuid/usetgid do: both set the real and the effective
     * id together.  They are NOT applied to the host process — see case 23. */
    guest_uid = guest_gid = 0;
    /* The program break, as a far address.  It starts past the initialized data
     * and the bss, which is where libc's sbrk begins handing memory out.  sbrk
     * abandons the tail of a segment and restarts at offset 0 of the next when a
     * request will not fit, so the break legitimately jumps segments and a far
     * address past the data segment is normal, not an error. */
    brk_addr = (uint32_t)data_seg << 24 | (uint32_t)(dbase + g[ng-1].span + g[ng-1].bss);

    exec_exited = false; exec_code = 0;
    UM->sys_hook = sys_hook;
    cpu_reset(&UM->cpu);
    /* Budget: compiler passes over large inputs run billions of cycles.
     * $N2BUDGET lowers it, which is how a harness can be shown handling a run
     * that stops without an exit -- the condition it must not read as one. */
    uint64_t budget = 8000000000ull;
    const char *bs = getenv("N2BUDGET");
    if (bs && *bs) budget = strtoull(bs, NULL, 0);
    int why = urun(budget, &exec_exited);

    /* A gap has to survive to the end of the run: the one-shot warning is
     * emitted thousands of lines before the result a reader is looking at, and
     * in a piped harness it is usually gone by then. */
    if (n_warned) {
        fprintf(stderr, "[c900: run used %d accepted-but-unimplemented syscall(s):", n_warned);
        for (int n = 0; n < 128; n++)
            if (warned_syscall[n]) fprintf(stderr, " %s", describe_sys((uint8_t)n));
        fprintf(stderr, " -- results above may be wrong]\n");
    }
    fflush(stdout);
    /* The banner is for a person running one program by hand.  A BUILD runs
     * thousands of guest processes and reads their exit STATUS, which this
     * function returns either way, so N2QUIET=1 drops the line rather than
     * making every recipe filter it -- filtering costs a pipeline, and a
     * pipeline's exit status is its last element's, which is how a failed
     * compile becomes a successful build.  The gap warning above is NOT
     * suppressed: it says the answer may be wrong.
     *
     * A run that faulted or ran out of budget never reached the guest's exit,
     * so it has no exit status to print: `[exit 0]' there states a clean exit
     * that did not happen, and every harness that reads the banner rather than
     * the status -- which is the status this call returns 4 for -- scores it as
     * a program that ran to completion and wrote nothing.  The banner and the
     * process status say the same thing in every case. */
    if (!getenv("N2QUIET")) {
        if (!exec_exited && why)
            fprintf(stderr, "[no exit: %s]\n",
                    why == 1 ? "guest fault" : "instruction budget exhausted");
        else
            fprintf(stderr, "[exit %d]\n", exec_code);
    }
    free(b); free(envcopy);
    if (!exec_exited && why) return 4;     /* fault or budget: not the guest's status */
    return exec_code & 0xFF;
}

/* ─────────────────────────── the system-call service ─────────────────────── */
static void sys_hook(Machine *m, uint8_t num, uint32_t pc){
    CPU *c = &m->cpu;
    int sseg = c->R[14] & 0x7F;    /* RR14 stack segment */
    int sp   = c->R[15];           /* stack offset */
    cur_cpu = c; cur_num = num; cur_sseg = sseg;

    /* Every argument is read out of the guest's own stack, so a program that has
     * destroyed RR14/R15 names memory this run never installed.  There is no
     * argument to decode and so no call to service: the run stops at the
     * instruction that made it, the same way an unimplemented call does. */
    const int argblock = 16;       /* 4-byte segmented return address + the widest arg list */
    if (!u_range_ok((uint32_t)sseg << 24 | (uint32_t)sp, argblock)) {
        fprintf(stderr, "[c900: syscall %s at pc %06x with the stack at %d:%04x, "
                        "which is not mapped -- run STOPPED]\n", describe_sys(num), pc, sseg, sp);
        exit(4);
    }
    /* syscall args sit above the 4-byte segmented return address the CALL pushed */
    #define ARGW(o)  gr16(sseg, sp + 4 + (o))
    #define ARGL(o)  ((uint32_t)gr16(sseg, sp + 4 + (o)) << 16 | gr16(sseg, sp + 4 + (o) + 2))

    static char pbuf[2048], pbuf2[2048];

    if (getenv("SYSDBG"))
        fprintf(stderr, "[sys %d @seg%d:%04x  a0=%04x a2l=%08x a6=%04x]\n",
                num, sseg, sp, ARGW(0), ARGL(2), ARGW(6));

    /* ── user-space buffer access (read/write) ─────────────────────────────
     * COUNT: uread/uwrite declare `unsigned n', so a guest that passes a
     * negative count really does get an unsigned one — read(fd,buf,-1) is a
     * 65535-byte request on the machine too, NOT EINVAL.  On the machine such a
     * transfer walks off the end of the buffer's segment and the per-byte
     * accessor faults, and EFAULT is what comes back.
     *
     * ADDRESS: a user address is seg:off and the offset is 16 bits, so a
     * transfer that runs past 0xFFFF carries into the SEGMENT number the way the
     * kernel's vadd() does.  A byte that lands in a segment this run never
     * installed has no memory behind it and is a fault. */
    switch (num) {
    case 1: {   /* exit(status) */
        exec_exited = true; exec_code = (int)ARGW(0);
        /* This trap is about to dispatch: repoint the syscall vector at the HALT
         * stub so exit stops the CPU here instead of IRET-returning into the
         * (looping) exit path. */
        gw16(0, 0x001E, 0x0062);
        break;
    }
    case 4: {   /* write(fd, buf, nb) */
        GFile *g = get_fd((int)ARGW(0));
        if (!g || g->dir) { fail(cEBADF); break; }   /* closed fd, or a directory */
        /* buf is a stored far pointer [segword][offword]; segword = seg<<8, so
         * the segment sits in bits 31..24 and the offset in bits 15..0.  Every
         * byte is read out of user space, so the WHOLE count is faulted up front. */
        uint32_t buf = ARGL(2); int nb = (int)ARGW(6);
        if (!u_range_ok(buf, nb)) { fail(cEFAULT); break; }
        uint8_t *p = malloc(nb ? nb : 1);
        for (int k = 0; k < nb; k++) p[k] = u_r8(buf, k);
        ssize_t n = write(g->fd, p, (size_t)nb);
        free(p);
        if (n < 0) { fail_err(); break; }
        ret_int((int16_t)n);
        break;
    }
    case 3: {   /* read(fd, buf, nb) */
        GFile *g = get_fd((int)ARGW(0));
        if (!g) { fail(cEBADF); break; }
        uint32_t buf = ARGL(2); int nb = (int)ARGW(6);
        uint8_t *p = malloc(nb ? nb : 1);
        int n;
        if (g->dir) {   /* a directory: hand out `struct direct' records */
            long avail = g->dirlen - g->pos;
            if (avail < 0) avail = 0;
            n = (int)(avail < nb ? avail : nb);
            memcpy(p, g->dir + g->pos, (size_t)n);
            g->pos += n;
        } else {
            ssize_t r = read(g->fd, p, (size_t)nb);
            if (r < 0) { free(p); fail_err(); break; }
            n = (int)r;
        }
        /* Only the bytes actually delivered are stored, so only those are
         * faulted on: a 65535-byte request near the end of a segment that
         * returns 10 bytes succeeds here exactly as it does on the machine. */
        if (!u_range_ok(buf, n)) { free(p); fail(cEFAULT); break; }
        for (int k = 0; k < n; k++) u_w8(buf, k, p[k]);
        free(p);
        ret_int((int16_t)n);   /* 0 = EOF */
        break;
    }
    case 5: {   /* open(path, oflag, perm) -- the 3-arg S5 form (tab.c entry 5 is P+I+I).
                 * oflag values are COHERENT's (include/fcntl.h): O_WRONLY 0x01,
                 * O_RDWR 0x02, O_APPEND 0x08, O_CREAT 0x100, O_TRUNC 0x200,
                 * O_EXCL 0x400.  Dropping O_TRUNC/O_APPEND leaves
                 * `open(f,O_WRONLY|O_CREAT|O_TRUNC,0666)' -- what most programs
                 * write instead of creat(2) -- with the tail of the old file
                 * behind it, and reports success. */
        if (!path_arg(ARGL(0), pbuf, sizeof pbuf)) { fail(cEFAULT); break; }
        int oflag = (int)ARGW(4);
        mode_t perm = (mode_t)(ARGW(6) & 0777);
        if (perm == 0) perm = 0666;
        if (getenv("SYSDBG"))
            fprintf(stderr, "[open \"%s\" oflag=%#x perm=%o]\n", pbuf, oflag, perm);
        struct stat st;
        if (stat(pbuf, &st) == 0 && S_ISDIR(st.st_mode)) {
            /* A directory opens read-only and is read as `struct direct'
             * records; opening one for writing is EISDIR, as on the guest. */
            if (oflag & 3) { fail(cEISDIR); break; }
            long dl; uint8_t *img = dir_image(pbuf, &dl);
            if (!img) { fail_err(); break; }
            int fd = alloc_fd();
            if (fd < 0) { free(img); fail(cEMFILE); break; }
            gfiles[fd].used = true; gfiles[fd].fd = -1;
            gfiles[fd].dir = img; gfiles[fd].dirlen = dl; gfiles[fd].pos = 0;
            snprintf(gfiles[fd].path, sizeof gfiles[fd].path, "%s", pbuf);
            ret_int((int16_t)fd);
            break;
        }
        int flag = O_RDONLY;
        switch (oflag & 3) { case 1: flag = O_WRONLY; break; case 2: case 3: flag = O_RDWR; break; }
        if (oflag & 0x008) flag |= O_APPEND;
        if (oflag & 0x100) flag |= O_CREAT;
        if (oflag & 0x200) flag |= O_TRUNC;
        if (oflag & 0x400) flag |= O_EXCL;
        int hfd = open(pbuf, flag|O_BINARY, perm);
        if (hfd < 0) { fail_err(); break; }
        int fd = alloc_fd();
        if (fd < 0) { close(hfd); fail(cEMFILE); break; }
        gfiles[fd].used = true; gfiles[fd].fd = hfd; gfiles[fd].dir = NULL;
        snprintf(gfiles[fd].path, sizeof gfiles[fd].path, "%s", pbuf);
        ret_int((int16_t)fd);
        break;
    }
    case 8: {   /* creat(path, perm) -- create/truncate for writing */
        if (!path_arg(ARGL(0), pbuf, sizeof pbuf)) { fail(cEFAULT); break; }
        int hfd = open(pbuf, O_RDWR|O_CREAT|O_TRUNC|O_BINARY, (mode_t)(ARGW(4) & 0777));
        if (hfd < 0) { fail_err(); break; }
        int fd = alloc_fd();
        if (fd < 0) { close(hfd); fail(cEMFILE); break; }
        gfiles[fd].used = true; gfiles[fd].fd = hfd; gfiles[fd].dir = NULL;
        snprintf(gfiles[fd].path, sizeof gfiles[fd].path, "%s", pbuf);
        ret_int((int16_t)fd);
        break;
    }
    case 10:    /* unlink(path) */
        if (!path_arg(ARGL(0), pbuf, sizeof pbuf)) { fail(cEFAULT); break; }
        if (unlink(pbuf) < 0) fail_err(); else ret_int(0);
        break;
    case 9: {   /* link(from, to) */
        if (!path_arg(ARGL(0), pbuf, sizeof pbuf) || !path_arg(ARGL(4), pbuf2, sizeof pbuf2)) { fail(cEFAULT); break; }
        /* mkdir(1) builds a directory in three calls -- mknod(IFDIR), then
         * link(dir,"dir/.") and link(parent,"dir/..") -- because on COHERENT
         * mknod makes an EMPTY directory inode and the two self-links are the
         * caller's job.  The host's mkdir(2) makes them itself and then refuses
         * both calls: "dir/." already exists (EEXIST) and a directory cannot be
         * hard-linked at all (EPERM).  So when the target names an entry a host
         * directory is born with, and it is already there, the link is reported
         * done rather than attempted. */
        const char *bn = strrchr(pbuf2, '/'); bn = bn ? bn + 1 : pbuf2;
        if (!strcmp(bn, ".") || !strcmp(bn, "..")) {
            struct stat sf, stt;
            if (stat(pbuf, &sf) == 0 && stat(pbuf2, &stt) == 0 &&
                S_ISDIR(sf.st_mode) && S_ISDIR(stt.st_mode)) { ret_int(0); break; }
        }
        if (host_link(pbuf, pbuf2) < 0) fail_err(); else ret_int(0);
        break;
    }
    case 14: {  /* mknod(path, mode, rdev) -- tab.c entry 14 is P+I+I, so rdev is
                 * the 16-bit dev_t, not a long.  This is the call mkdir(1) is
                 * built on, and umknod is the rule followed: anything but a pipe
                 * needs the super-user, an existing name is EEXIST, and rdev is
                 * ignored for everything that is not a device. */
        if (!path_arg(ARGL(0), pbuf, sizeof pbuf)) { fail(cEFAULT); break; }
        uint16_t mode = ARGW(4);
        uint16_t typ = mode & mIFMT;
        mode_t perm = (mode_t)(mode & 0777);
        if (typ != mIFPIPE && guest_uid != 0) { fail(cEPERM); break; }
        if (typ == mIFCHR || typ == mIFBLK) {
            /* A device node cannot be made here: creating one on the host needs
             * privileges this harness does not have and must not want, and there
             * would be no driver behind it if it appeared.  EPERM is a
             * DIVERGENCE -- on the machine, as root, the call succeeds -- but it
             * is the errno a non-super-user really gets, every caller already has
             * a branch for it, and it cannot be mistaken for the node having been
             * made. */
            fail(cEPERM); break;
        }
        int rc;
        switch (typ) {
        case mIFDIR: rc = host_mkdir(pbuf, perm); break;
        /* A real FIFO.  Nothing here can ever be its far end -- there is one
         * process -- so it is only ever a self-rendezvous.  Creating it is still
         * the faithful answer: the callers that reach this check only that the
         * node was made. */
        case mIFPIPE: rc = host_mkfifo(pbuf, perm); break;
        case 0: case mIFREG: {
            int f = open(pbuf, O_CREAT|O_EXCL|O_WRONLY|O_BINARY, perm);
            rc = f < 0 ? -1 : (close(f), 0);
            break;
        }
        default: errno = EINVAL; rc = -1; break;
        }
        if (rc < 0) fail_err(); else ret_int(0);
        break;
    }
    case 12:    /* chdir(path) */
        if (!path_arg(ARGL(0), pbuf, sizeof pbuf)) { fail(cEFAULT); break; }
        if (chdir(pbuf) < 0) fail_err(); else ret_int(0);
        break;
    case 36:    /* sync() -- host writes are already through; nothing to flush */
        ret_int(0);
        break;
    /* There is deliberately NO case 13.  Syscall 13 is `unone' in the kernel
     * table and no libc stub issues it -- time(3) here is a wrapper over
     * ftime(2).  Servicing 13 would let a program work in this harness and fail
     * on the machine. */
    case 35: {  /* ftime(tbp) -- `struct timeb' (sys/timeb.h): time_t time (4),
                 * unsigned short millitm, short timezone, short dstflag.  This is
                 * what time(3) calls, so it is the clock every C program reads.
                 * The zone is reported as GMT so a run is reproducible. */
        uint32_t p = ARGL(0);
        if (p) {
            if (!u_range_ok(p, 10)) { fail(cEFAULT); break; }
            uint32_t sec; uint16_t ms;
            host_now(&sec, &ms);
            u_w8(p,0,(uint8_t)(sec>>24)); u_w8(p,1,(uint8_t)(sec>>16));
            u_w8(p,2,(uint8_t)(sec>>8));  u_w8(p,3,(uint8_t)sec);
            u_w8(p,4,(uint8_t)(ms>>8)); u_w8(p,5,(uint8_t)ms);
            for (int k = 6; k < 10; k++) u_w8(p,k,0);   /* timezone, dstflag */
        }
        ret_int(0);
        break;
    }
    case 30: {  /* utime(path, timep) -- `struct utimbuf': two 4-byte time_t,
                 * actime then modtime, big-endian through a far pointer.  A null
                 * pointer means "now", as on the real system. */
        if (!path_arg(ARGL(0), pbuf, sizeof pbuf)) { fail(cEFAULT); break; }
        uint32_t nowsec; uint16_t nowms;
        host_now(&nowsec, &nowms);
        long at = (long)nowsec, mt = (long)nowsec;
        uint32_t p = ARGL(4);
        if (p) {
            if (!u_range_ok(p, 8)) { fail(cEFAULT); break; }
            at = 0; mt = 0;
            for (int k = 0; k < 4; k++) at = at<<8 | u_r8(p, k);
            for (int k = 0; k < 4; k++) mt = mt<<8 | u_r8(p, 4+k);
        }
        if (host_utime(pbuf, at, mt) < 0) fail_err(); else ret_int(0);
        break;
    }
    case 16:    /* chown(path, uid, gid) -- accepted and not applied: the guest
                 * runs as root here and the host file's owner is not the guest's
                 * to change. */
        ret_int(0);
        break;
    case 24: case 57:   /* getuid, geteuid -- the guest's own id, never the host's.
                         * It starts at 0 because a guest resolves this through its
                         * own /etc/passwd and root is the entry an image is
                         * guaranteed to have; reporting the HOST uid names somebody
                         * the image has never heard of.  usetuid sets the real and
                         * effective id together, so both calls answer the same. */
        ret_int(guest_uid);
        break;
    case 47: case 56:   /* getgid, getegid */
        ret_int(guest_gid);
        break;
    case 23: {  /* setuid(uid) -- ACCEPTED, TRACKED, AND NOT APPLIED TO THE HOST.
                 * The permission rule is usetuid's: changing to a different id
                 * requires the super-user, so the first call from the guest's
                 * initial root always succeeds and a second one to a third id gets
                 * EPERM, exactly as on the machine.  What is NOT done is any change
                 * to the host process: this harness IS the host process, and a real
                 * setuid would either fail or, worse, succeed and irreversibly drop
                 * the privileges every later open(2) in the run depends on.  The
                 * DIVERGENCE that leaves is that the guest's file access does not
                 * narrow when it drops privilege. */
        int16_t uid = (int16_t)ARGW(0);
        if (uid != guest_uid && guest_uid != 0) { fail(cEPERM); break; }
        guest_uid = uid; ret_int(0);
        break;
    }
    case 46: {  /* setgid(gid) -- as setuid; usetgid gates on the effective UID,
                 * not on the group being changed from. */
        int16_t gid = (int16_t)ARGW(0);
        if (gid != guest_gid && guest_uid != 0) { fail(cEPERM); break; }
        guest_gid = gid; ret_int(0);
        break;
    }
    case 15:    /* chmod(path, mode) */
        if (!path_arg(ARGL(0), pbuf, sizeof pbuf)) { fail(cEFAULT); break; }
        if (chmod(pbuf, (mode_t)(ARGW(4) & 0777)) < 0) fail_err(); else ret_int(0);
        break;
    case 17: {  /* brk(addr) -- PTR class: the answer is a far pointer in RR0.
                 * brk(0) reports the current break; a successful brk reports the
                 * NEW one, which is what the kernel does (ubrk returns
                 * vadd(sb,s_size), not 0).  libc's sbrk cannot see the difference
                 * (it tracks __end itself and reads only errno), but a program
                 * calling brk(2) directly can. */
        uint32_t want = ARGL(0);
        if (want == 0) { ret_wide(brk_addr); break; }
        if ((int)((want >> 24) & 0x7F) > heap_hi) {
            /* sbrk(3) clears errno, calls brk and decides ONLY on errno whether
             * the break was refused -- a bare -1 leaves it returning memory the
             * process does not have. */
            fail(cENOMEM); break;
        }
        brk_addr = want;
        ret_wide(brk_addr);
        break;
    }
    case 20:    /* getpid() -- a value a COHERENT pid could actually be.  The
                 * guest's int is 16 bits and libc reads this into one: mktemp
                 * does `register i, pid; pid = getpid();' and divides by 10 for
                 * each digit, so a value over 32767 arrives negative and the
                 * digits come out as punctuation -- including '/', which stops
                 * the temp name being a single path component at all. */
        ret_int((int16_t)(getpid() % 30000 + 100));
        break;
    case 48:    /* signal(sig, func) -- the previous handler, and nothing is
                 * delivered here: a guest asking whether it is in the foreground
                 * (signal(SIGINT, SIG_IGN)) must see SIG_DFL rather than an error.
                 * PTR class: SIG_DFL is a 32-bit NULL in RR0, not a 16-bit one. */
        ret_wide(0);
        break;
    case 60:    /* umask(mask) -- the previous mask, which nothing here enforces */
        ret_int(022);
        break;
    case 33: {  /* access(path, mode) -- R_OK 4 / W_OK 2 / X_OK 1 / F_OK 0.
                 * Ignoring the mode makes access(f, W_OK) answer 0 for a
                 * read-only file and the caller go on to fail at the open. */
        if (!path_arg(ARGL(0), pbuf, sizeof pbuf)) { fail(cEFAULT); break; }
        int amode = ARGW(4) & 7, hmode = 0;
        if (amode & 4) hmode |= R_OK;
        if (amode & 2) hmode |= W_OK;
        if (amode & 1) hmode |= X_OK;
        if (access(pbuf, hmode) < 0) fail_err(); else ret_int(0);
        break;
    }
    case 18: {  /* stat(path, sb) */
        if (!path_arg(ARGL(0), pbuf, sizeof pbuf)) { fail(cEFAULT); break; }
        struct stat st;
        if (stat(pbuf, &st) < 0) { fail_err(); break; }
        if (!write_stat_buf(ARGL(4), &st, pbuf)) { fail(cEFAULT); break; }
        ret_int(0);
        break;
    }
    case 28: {  /* fstat(fd, sb) */
        GFile *g = get_fd((int)ARGW(0));
        if (!g) { fail(cEBADF); break; }
        struct stat st;
        int rc = g->dir ? stat(g->path, &st) : fstat(g->fd, &st);
        if (rc < 0) { fail_err(); break; }
        if (!write_stat_buf(ARGL(2), &st, g->path)) { fail(cEFAULT); break; }
        ret_int(0);
        break;
    }
    case 6: {   /* close(fd) */
        int fd = (int)ARGW(0);
        GFile *g = get_fd(fd);
        if (!g) { fail(cEBADF); break; }
        if (g->fd >= 0 && fd > 2) close(g->fd);   /* never close our own std streams */
        free(g->dir);
        memset(g, 0, sizeof *g);
        ret_int(0);
        break;
    }
    case 54: {  /* ioctl(fd, cmd, arg) -- tab.c entry 54 is I+I+P.  Answered from
                 * the HOST descriptor: isatty(3) is `ioctl(fd,TIOCGETP,&sgb)>=0',
                 * and reporting success for fds 0/1/2 unconditionally tells every
                 * program it is on a terminal even when redirected -- ls
                 * columnates, more paginates, stdio line-buffers.  Command values
                 * are the non-i386 set in <sgtty.h>: TIOCSETP 0100, TIOCGETP 0101,
                 * TIOCSETC 0102, TIOCGETC 0103, TIOCSETN 0104. */
        GFile *g = get_fd((int)ARGW(0));
        if (!g) { fail(cEBADF); break; }
        if (!is_tty(g)) { fail(cENOTTY); break; }
        uint16_t cmd = ARGW(2);
        uint32_t p = ARGL(4);
        uint8_t sg[6] = { 16, 16, 010, 025, 0, 0 };   /* B9600/B9600, ^H erase, ^U kill */
        uint8_t tc[6] = { 0177, 034, 021, 023, 004, 0377 };
        uint16_t fl = 04 | 010 | 02000;               /* CRMOD|ECHO|CRT */
        sg[4] = (uint8_t)(fl >> 8); sg[5] = (uint8_t)fl;
        const uint8_t *src = NULL;
        switch (cmd) {
        case 0101: src = sg; break;   /* TIOCGETP -> struct sgttyb */
        case 0103: src = tc; break;   /* TIOCGETC -> struct tchars */
        case 0100: case 0102: case 0104: ret_int(0); break;  /* SETP/SETC/SETN: accepted, not applied */
        default: fail(cEINVAL); break;
        }
        if (src) {
            if (!u_range_ok(p, 6)) { fail(cEFAULT); break; }
            for (int k = 0; k < 6; k++) u_w8(p, k, src[k]);
            ret_int(0);
        }
        break;
    }
    case 19: {  /* lseek(fd, off32, whence) -- LONG return, so the full 32 bits go
                 * back in RR0 (R0:R1). */
        GFile *g = get_fd((int)ARGW(0));
        if (!g) { fail(cEBADF); break; }
        long off = (long)(int32_t)ARGL(2);
        int whence = (int)ARGW(6);
        if (g->dir) {   /* seek within the synthesized directory image */
            if (whence < 0 || whence > 2) { fail(cEINVAL); break; }
            if (whence == 0)      g->pos = off;
            else if (whence == 1) g->pos += off;
            else                  g->pos = g->dirlen + off;
            if (g->pos < 0) g->pos = 0;
            ret_wide((uint32_t)g->pos);
            break;
        }
        off_t pos = lseek(g->fd, off, whence);
        if (pos < 0) { fail_err(); break; }
        ret_wide((uint32_t)pos);
        break;
    }
    default:
        /* An unimplemented call FAILS, and says so once per number.  Returning
         * success is what a caller cannot distinguish from the real thing:
         * link(2) answering 0 without creating anything led RCS's rename --
         * unlink(to); link(from,to); unlink(from) -- to destroy both files while
         * reporting that it had worked.
         *
         * -1 is still not the machine's answer (there the call WORKS), so a gap
         * must be attributable.  By DEFAULT a gap stops the run at the
         * instruction that made the call; only default_accept (or N2ACCEPT) lets
         * one pass with a warning. */
        if (!policy_ok(num)) {
            fprintf(stderr, "[c900: unimplemented syscall %s at pc %06x -- run STOPPED]\n",
                    describe_sys(num), pc);
            const char *nm = sys_name(num);
            char decl[16];
            if (!nm) { snprintf(decl, sizeof decl, "%d", num); nm = decl; }
            fprintf(stderr, "[c900: if this program is expected to survive without it, declare it: "
                            "N2ACCEPT=%s (names or numbers, comma-separated); "
                            "N2STRICT=0 accepts every gap]\n", nm);
            fflush(stdout);
            exit(4);
        }
        if (num < 128 && !warned_syscall[num]) {
            warned_syscall[num] = true; n_warned++;
            fprintf(stderr, "[c900: unimplemented syscall %s @%06x -> -1 (accepted)]\n",
                    describe_sys(num), pc);
        }
        fail(cEINVAL);
        break;
    }
    #undef ARGW
    #undef ARGL
}

