/* emu.h — Commodore C900 classic (instruction-level) emulator.
 *
 * Zilog Z8001 CPU + Z8010 MMU, boot ROM, DRAM, WD2010-style hard disk plus an
 * optional Commodore floppy (both flat files, DMA memcpy), and a Z8030 SCC
 * serial console redirected to stdin/stdout. No video, no GUI.
 */
#ifndef EMU_H
#define EMU_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* ─────────────────────────── FCW flag bits ─────────────────────────── */
#define F_C   0x0080u  /* Carry */
#define F_Z   0x0040u  /* Zero */
#define F_S   0x0020u  /* Sign */
#define F_PV  0x0010u  /* Parity(byte)/Overflow(word,long) */
#define F_DA  0x0008u  /* Decimal Adjust */
#define F_H   0x0004u  /* Half carry */
#define F_MASK (F_C|F_Z|F_S|F_PV|F_DA|F_H)

#define FCW_SEG  0x8000u /* Segmented mode (Z8001) */
#define FCW_SN   0x4000u /* System/Normal (1=system) */
#define FCW_EPA  0x2000u /* Extended processor */
#define FCW_VIE  0x1000u /* Vectored interrupt enable */
#define FCW_NVIE 0x0800u /* Non-vectored interrupt enable */

/* ─────────────────────────── ALU ops ─────────────────────────── */
typedef enum {
    OpAdd, OpAdc, OpSub, OpSbc, OpAnd, OpOr, OpXor, OpCom, OpNeg,
    OpTest, OpCp, OpInc, OpDec, OpSla, OpSra, OpSll, OpSrl, OpRl, OpRr,
    OpRlc, OpRrc, OpDab, OpExts, OpLd, OpBit, OpSet, OpRes, OpTset
} AluOp;

/* widths (byte=1, word=2, long=4) */
typedef enum { WByte = 1, WWord = 2, WLong = 4 } Width;

typedef struct { uint64_t value; uint16_t flags; uint16_t mask; } AluResult;
AluResult alu_exec(AluOp op, uint64_t a, uint64_t b, bool carry_in, Width w);

/* ─────────────────────────── decode ───────────────────────────
 * The decoder is data-driven: every opcode resolves to a CtrlWord (below) whose
 * fields tell the execute engine which handler class to run and where its
 * operands come from. The enums here name the choices those fields can take.
 * This keeps execute() a compact switch on InstrClass rather than one branch
 * per opcode. */

/* InstrClass — which execute handler runs the instruction. Each value maps to
 * a case in cpu.c's execute dispatch (arithmetic/logic = ClsUnified, control
 * transfers = ClsBranch, the four block families, MMU/CPU-control ops, etc.). */
typedef enum {
    ClsUnified, ClsFcwOp, ClsExts, ClsStack, ClsBranch, ClsTCC, ClsDAB,
    ClsExchange, ClsBCDRotate, ClsBitDynamic, ClsLEA, ClsIO, ClsLDCTL,
    ClsShift, ClsBlockTransfer, ClsBlockCompare, ClsBlockString, ClsBlockIO,
    ClsLDM, ClsMultiProc, ClsMul, ClsDiv, ClsLDPS, ClsHalt, ClsSysCall,
    ClsIRET, ClsTranslate
} InstrClass;

typedef enum {
    BranchJP_DA, BranchJP_X, BranchJP_IR, BranchJR, BranchCALL_DA,
    BranchCALL_X, BranchCALL_IR, BranchCALR, BranchCALR12, BranchRET, BranchDJNZ
} BranchMode;

/* FcwOpKind — how the flag-control instructions (SETFLG/RESFLG/COMFLG,
 * LDCTLB FCW-low) manipulate the FCW low byte. */
typedef enum { FcwSet, FcwClear, FcwToggle, FcwLoadLo, FcwStoreLo } FcwOpKind;

/* StackMode — the PUSH/POP variants: word vs long (L), implicit vs
 * source/destination register, and immediate push. */
typedef enum {
    StackPush, StackPop, StackPushL, StackPopL, StackPushSrc, StackPopDst,
    StackPushLSrc, StackPopLDst, StackPushImm
} StackMode;

/* SrcSel — where the second operand comes from: a register, an immediate, an
 * indirect-register (IR) / direct-address (DA) / indexed (X) / based / based-
 * indexed memory operand, a PC-relative target, or a synthesized constant
 * (0/1/2, the nib3 count field, etc.). Drives operand fetch in execute(). */
typedef enum {
    SrcNone, SrcReg, SrcImm, SrcIR, SrcDA, SrcX, SrcOne, SrcTwo, SrcNib3Cnt,
    SrcZero, SrcImm2, SrcPCRel, SrcBased, SrcBX, SrcNib3Bit, SrcNib3CntPlus1,
    SrcOpLoByte
} SrcSel;

/* DstSel — where the result is written: a register, memory (IR/DA/X/based/
 * based-indexed), flags only (compares), or nowhere. */
typedef enum {
    DstReg, DstNone, DstFlagsOnly, DstIR, DstDA, DstX, DstPCRel, DstBased, DstBX
} DstSel;

/* RegSel — which opcode nibble names a register operand: nib3=op&0xF,
 * nib2=(op>>4)&0xF, nib1=(op>>8)&0xF. */
typedef enum { RegNib3, RegNib2, RegNib1 } RegSel;

/* MPOpKind — multi-micro-processor control ops (MBIT/MRES/MSET/MREQ). */
typedef enum { MPBit, MPRes, MPSet, MPReq } MPOpKind;

/* CtrlWord — the fully decoded description of one instruction. The decoder
 * produces it; execute() reads it. Every field is a "knob" the generic execute
 * engine consults instead of special-casing the opcode, so adding/adjusting an
 * instruction is a table change rather than new control flow. */
typedef struct {
    const char *name;    /* mnemonic, for tracing/disassembly */
    InstrClass  exec;    /* which execute handler class runs this op */
    AluOp       alu_op;  /* ALU operation (for ClsUnified/ClsShift/etc.) */
    Width       width;   /* operand width; 0 means "unset" → treat as WWord */
    int         words;   /* instruction length in 16-bit words */
    bool        priv;    /* privileged: traps if executed in Normal mode */
    SrcSel      src2;    /* source of the second operand */
    DstSel      dst;     /* destination of the result */
    SrcSel      count;   /* shift/rotate count or block length source */
    RegSel      src1_nib;/* opcode nibble naming source-1 register */
    RegSel      src2_nib;/* opcode nibble naming source-2 register */
    RegSel      dst_nib; /* opcode nibble naming destination register */
    FcwOpKind   fcw_op;  /* flag-control variant (ClsFcwOp) */
    uint16_t    fcw_mask;/* which FCW bits the flag op touches */
    StackMode   stk_mode;/* PUSH/POP variant (ClsStack) */
    BranchMode  br_mode; /* branch/call/return addressing (ClsBranch) */
    bool        io_write;/* I/O op direction: true = OUT, false = IN */
    bool        io_spec; /* special I/O space (MMU command ports) */
    bool        block_decr;/* block op auto-decrements (…D variants) vs increments */
    bool        block_rept;/* repeating block op (…R variants) */
    MPOpKind    mp_op;   /* multi-processor control variant */
    bool        valid;   /* false → illegal/unimplemented encoding */
} CtrlWord;

/* Decode: fill *out for the instruction whose words are words[0..].
 * Returns number of 16-bit words the instruction occupies. */
void decode_init(void);
int  decode_length(uint16_t op0, bool segmented);
const CtrlWord *decode_lookup(uint16_t op0, const uint16_t *words, int navail,
                              bool segmented);

/* ─────────────────────────── CPU ─────────────────────────── */
typedef struct Machine Machine;

typedef struct {
    uint16_t R[16];
    uint32_t pc;       /* 23-bit segmented internal form: seg<<16 | off */
    uint16_t fcw;
    uint32_t psap;     /* seg<<16 | off */
    uint16_t refresh;
    uint16_t nspseg, nspoff;

    bool     halted;
    uint8_t  irq_req;      /* pending trap/int bits */
    bool     nvi_line, vi_line, nmi_line, segt_line;
    bool     nmi_prev;
    /* external vector supplied on VI INTACK (device-driven) */
    uint16_t vi_vector;

    uint16_t cur_op;       /* opcode[0] of executing instr (for trap tag) */

    /* per-instruction decode context */
    uint16_t opcode[6];
    int      oplen;
    bool     seg_w1, seg_w2;
    uint32_t instr_start;  /* PC at start of current instruction */
    bool     repeat_pending; /* block op should re-execute (PC rewound) */
    bool     fault;        /* a segment-trap fault occurred this instruction */

    Machine *m;
    uint64_t insns;
    uint8_t  serviced;     /* irq bit serviced on the last step (0 if none) */
} CPU;

/* decode.c helpers */
bool decode_seg_addr_w1(uint16_t op, bool segmented);
bool decode_seg_addr_w2(uint16_t op, bool segmented);

/* irq bits */
#define IRQ_RESET   0x01
#define IRQ_SYSCALL 0x02
#define IRQ_VI      0x04
#define IRQ_NVI     0x08
#define IRQ_SEGTRAP 0x10
#define IRQ_NMI     0x20
#define IRQ_TRAP    0x40
#define IRQ_EPU     0x80

void cpu_reset(CPU *c);
void cpu_step(CPU *c);   /* execute exactly one instruction (or take a pending trap) */

/* ─────────────────────────── MMU (Z8010) ─────────────────────────── */
typedef struct { uint16_t base; uint8_t limit; uint8_t attr; } MmuDesc;

/* attribute bits */
#define A_REF  0x80
#define A_CHG  0x40
#define A_DIRW 0x20
#define A_DMAI 0x10
#define A_EXC  0x08
#define A_CPU  0x04
#define A_SYS  0x02
#define A_RD   0x01

typedef struct {
    MmuDesc  desc[64];
    uint8_t  mode;     /* MSEN 0x80, TRNS 0x40, URS 0x20, MST 0x10, NMS 0x08, ID */
    uint8_t  sar;      /* 6-bit */
    uint8_t  dsc;      /* 2-bit */
    uint8_t  viol_type;
    uint8_t  viol_sn;
    uint16_t viol_off;
    uint8_t  bcsr;
    uint8_t  isn;
    uint8_t  ioff;
    bool     viol_captured;
    /* trap output for current translate */
    bool     segt;     /* segment trap requested (level) */
    bool     warn_pending; /* one-shot: a write warning (PWW/SWW) fired this
                            * access; the CPU consumes it after the instruction
                            * completes and takes a segment trap WITHOUT
                            * rewinding PC (the write went through) */

    /* fast-path cache: for descriptors with full limit (0xFF) and no
     * protection bits, translation is just (base<<8)+off (+ REF/CHG). */
    uint32_t fast_base[64];
    bool     fast_ok[64];
} MMU;

#define MODE_MSEN 0x80
#define MODE_TRNS 0x40
#define MODE_URS  0x20
#define MODE_MST  0x10
#define MODE_NMS  0x08

void mmu_reset(MMU *u);
/* command access via special I/O; cmd = high byte of port. */
uint16_t mmu_cmd_read(MMU *u, uint8_t cmd);
void     mmu_cmd_write(MMU *u, uint8_t cmd, uint8_t data);
/* Translate a logical (seg,off) to 24-bit physical; sets *fault on hard viol. */
uint32_t mmu_translate(MMU *u, uint8_t sn, uint16_t off,
                       bool is_write, bool is_normal, bool is_fetch, bool *fault);

/* ─────────────────────────── Machine / bus ─────────────────────────── */

#define ROM_SIZE   0x8000u        /* 32 KiB */
#define PHYS_SIZE  0x1000000u     /* 24-bit physical space */

/* One Z8030 SCC channel. Channel B is the C900 console; Channel A is the
 * second RS-232 port. Bytes are moved through the host terminal (see main.c). */
typedef struct {
    uint8_t  wr[16];       /* write-register file (WR0..WR15) */
    uint8_t  ptr;          /* Z8030 register pointer: if non-zero, overrides the
                            * address-decoded register for the next access, then
                            * auto-resets (set by a WR0 write; see scc_read/write) */
    bool     rx_avail;     /* a received byte is waiting in rx_data */
    uint8_t  rx_data;      /* last byte delivered to this channel's receiver */
    /* Rx Overrun (RR1 D5).  The real Z8030 latches this when a character is
     * shifted in over one the CPU has not read, and holds it until the Error
     * Reset command (WR0 D5:D3 = 110).  Set only when --rx-overrun asked for
     * a receiver that can lose a character; without it this is always false
     * and RR1 reads exactly as it always has. */
    bool     rx_overrun;
    uint64_t rx_lost;      /* characters this receiver dropped (reported at exit) */
} SCCChan;

/* ───────────────────────── SERIAL PORT TABLE ─────────────────────────
 * The C900 carries THREE Z8030 SCCs, six RS-232 ports.  The map is not an
 * inference: it is the port table of COHERENT's own async line driver,
 * `sys/z8001/drv/al.c:162-169` (`altty[]`), whose `NSCC 3` / `NMINOR 6` /
 * `VECTOR 0x10` are at `al.c:31-34`.  The LR board's two chips are U31 and
 * U36 (`planning/LR_BOARD_SCHEMATIC.md:83-88,349-350`); U36's socket is empty
 * on the inventoried board but the schematic provisions it, so both are
 * modelled and both are absent unless asked for.
 *
 *   port  address  chip                         what it is
 *   ----  -------  ---------------------------  --------------------------
 *     0    0x0100  motherboard SCC (U74) chan B  the ROM's serial CONSOLE
 *     1    0x0120  motherboard SCC (U74) chan A  the one spare motherboard
 *                                                port, the guest's
 *                                                /dev/tty51 — this is what
 *                                                --wire has always meant
 *     2    0x0300  LR SCC #1 (U31) chan B        rear DB25 CN3
 *     3    0x0320  LR SCC #1 (U31) chan A        rear DB25 CN4
 *     4    0x0380  LR SCC #2 (U36) chan B        internal header CN5
 *     5    0x03A0  LR SCC #2 (U36) chan A        internal header CN6
 *
 * Within a chip's block, bit 5 of the port selects the channel (1 -> A) and
 * bits 4:1 select the register, exactly as the motherboard SCC already
 * decodes (see scc_read/scc_write).  A7 selects U31 from U36 within the
 * 0x0300 page (`planning/PLA_REVERSE_ENGINEERING.md:159-161`); which PLA term
 * does that is still open there, so each chip is decoded on its 0x80 page and
 * mirrors in the upper half, the same shape the motherboard SCC already has.
 *
 * WHAT IS FIRM AND WHAT IS NOT: the six addresses, NSCC, NMINOR and the
 * vector base are firm — they are read out of shipping kernel source.  The
 * exact PLA equation that produces the LR chip select is NOT established
 * (`LR_BOARD_SCHEMATIC.md:248,339-340` says "range derivation TBD"), so the
 * mirroring above is modelled by analogy with the motherboard, not traced. */
#define SCC_CHIPS      3            /* U74 (motherboard), U31, U36 (LR) */
#define SCC_PORTS      6            /* two channels per chip */
#define WIRE_MAX       SCC_PORTS
#define SERPORT_CONSOLE 0           /* motherboard chan B — never wired */
#define SERPORT_MB_A    1           /* motherboard chan A — plain --wire */
#define SERPORT_LR_FIRST 2          /* ports 2..5 live on the LR board */

/* Hard disk / floppy: flat file, one 512-byte sector per block */
typedef struct {
    FILE    *fp;
    uint32_t sectors;    /* total 512-byte sectors */
    uint32_t cyls, heads, spt;
    bool     present;
} Disk;

/* Commodore floppy: 80 tracks × 2 sides, zone-bit density (16/15/14/13
 * sectors per track) → NFBLK blocks addressed linearly, exactly the order a
 * flat image file stores them in. */
#define FLOPPY_BLOCKS 2392u

/* ───────────────── OKI MSM58321 real-time clock (board U16) ─────────────────
 * A 4-bit clock/calendar module bit-banged through CIO #1. See bus.c for the 
 * pin protocol and the datasheet citations. */
typedef struct {
    bool     present;      /* false = no module fitted (--rtc=none) */
    uint8_t  regs[16];     /* register file, 4 bits each */
    uint8_t  addr;         /* latched address A3..A0 (no auto-increment) */
    bool     stopped;      /* STOP level: oscillator halted */
    uint8_t  prev_pb;      /* previous Port B level, for strobe edge detect */
    bool     prev_cs;      /* previous /CS level (asserted = true) */
    uint64_t cs_since;     /* instruction count when /CS was last asserted */

    /* The divider. `ips' instructions of emulated CPU time = one second;
     * `base' is the instruction count at the start of the current second and
     * `held' freezes the divider's contents while STOP is high. */
    uint64_t ips;
    uint64_t base;
    uint64_t held;

    /* Observation counters, reported at shutdown (never read by the guest). */
    uint64_t n_read, n_write, n_tick, n_tick_selected;
} RTC;

struct Machine {
    CPU  cpu;
    MMU  mmu;
    uint8_t rom[ROM_SIZE];
    uint8_t *ram;        /* full 24-bit space for simplicity (16 MiB) */

    SCCChan scc_b;       /* console channel (AD5=0) */
    SCCChan scc_a;       /* second channel */
    /* The LR board's four ports, in SERIAL PORT TABLE order: index 0..3 are
     * ports 2..5.  Modelled only when lr_scc is set; with it clear the 0x0300
     * page is not decoded at all and reads there float back as 0, which is
     * what this emulator did before these chips existed. */
    SCCChan lr_scc_chan[SCC_PORTS - SERPORT_LR_FIRST];
    bool    lr_scc;      /* the LR board's SCCs answer (--lr-scc) */
    bool    wire_on;     /* channel A is attached to a host endpoint (--wire) */
    bool    wire_port_on[SCC_PORTS];   /* which ports have a host endpoint */
    bool    any_wire;    /* any port at all is wired — keeps the run loop's
                          * per-port scan out of an unwired run entirely */
    /* Opt-in receiver overrun (--rx-overrun).  Off, the run loop refuses to
     * take a byte off a wire until the guest has read the previous one, so a
     * character can never be lost and an interrupt-driven receive ring is
     * indistinguishable from a polled one.  On, the receiver behaves like the
     * real chip: the new byte lands, the old one is gone, and RR1 D5 latches
     * the overrun until an Error Reset. */
    bool    rx_overrun_on;

    Disk disk;           /* hard disk, command block at hdc_cmdblk */
    Disk floppy;         /* optional floppy, command block at hdc_cmdblk+0x10 */

    /* PDMAC DMA address latch (23-bit) for HDC */
    uint32_t dma_addr;

    /* system-control latches 0x0200-0x02FF */
    uint16_t latch[0x80];

    /* CIO functional state (reg file per chip; reg = (port>>1)&0x3F) */
    uint8_t  cio1[64];
    uint8_t  cio2[64];
    bool     cio1_reset, cio2_reset;
    /* CIO#1 Port A keyboard input */
    uint8_t  pa_data;
    bool     pa_irf;

    RTC      rtc;          /* OKI MSM58321 on CIO #1 Port B + PC1 */

    /* interrupt aggregation */
    bool     disk_vi;      /* PDMAC disk-completion VI (vector 0x80), one-shot */
    bool     last_vi_disk; /* the VI currently asserted came from the disk */

    /* CIO #1 Counter/Timer 3 — the 100 Hz Coherent system tick */
    bool     ct3_running;
    uint32_t ct3_accum;    /* instruction accumulator toward terminal count */

    /* HDC command block base (physical); relocatable via WDCCBA */
    uint32_t hdc_cmdblk;

    /* CIO timer emulation for Coherent system tick */
    uint64_t tick_counter;

    bool     trace;
    bool     stop;       /* request emulator halt */
    uint64_t insn_limit; /* 0 = unlimited */

    /* scripted serial input (for non-interactive testing).  RX_BLOCKED_POLLS is
     * the streak of empty console RR0 reads that means the guest is waiting for
     * a byte rather than running: an input loop polls continuously, an output
     * path checks at most once per character printed. */
#define RX_BLOCKED_POLLS 64
    uint8_t  inq[8192];
    int      inq_len, inq_pos;
    uint64_t last_tx_insn;  /* insn count of the guest's last console activity (output byte
                             * sent, or input byte read). After a long blocked read the
                             * output-side clock is stale, and a byte fed on it lands before
                             * the guest has even finished processing the previous one */
    bool     shell_up;      /* '#' prompt seen after boot → safe to feed input */
    uint32_t prompt_seq;    /* count of '#' prompts printed since boot */
    uint32_t inq_wait_seq;  /* scripted input waits for this prompt_seq before its next byte */
    int      inq_gateoff;   /* queue index from which the prompt gate stops applying (-1 = never) */
    bool     inq_now[8192]; /* per-byte TYPE-AHEAD flag (the \i escape): this byte is handed to
                             * the receiver the moment it is free, with no gate and no quiet
                             * wait, exactly as a person typing ahead of a running program would
                             * deliver it.  The pacing below decides WHEN a byte the guest is
                             * not yet ready for should go in; \i is the caller saying it does
                             * not want that decision made for this byte -- see bus.c */
    bool     inq_cr_wait;   /* last scripted byte fed was a CR/LF: a command is in flight */
    const char *inq_mark;   /* --input-mark: text the guest must PRINT before any type-ahead
                             * byte is released ("type this when you see that"), or NULL */
    int      inq_mark_pos;  /* how much of inq_mark has matched the console output so far */
    bool     inq_mark_seen; /* the whole of it has been printed: type-ahead is now open, and
                             * stays open -- the mark is a starting gun, not a gate */
    uint32_t rx_poll_streak;/* consecutive console RR0 polls (rx empty) since the last console TX;
                             * at RX_BLOCKED_POLLS the guest is reading, not running */
    bool     guest_polls;   /* a blocked-reading RR0 poll streak has been observed since the last
                             * scripted byte was fed: the current reader polls, so the streak is
                             * a usable ready signal (cleared on every feed -- not sticky) */
    bool     at_prompt;     /* the last byte the guest transmitted on the console was a prompt
                             * character ('#' or '>'): it has stopped talking AT a prompt, which
                             * is stronger than having printed one somewhere in its output */
    uint64_t idle_quiet;    /* instructions of console silence at a prompt, with the scripted
                             * input used up, that end the run (0 = never; --idle) */
    uint16_t stop_port;     /* I/O port whose 0xC900 write ends the run (0 = none; --stop-port) */
    bool     park_watch;    /* --stop-on=park: end the run when the guest is parked -- halted,
                             * woken only by a periodic interrupt, and doing nothing when
                             * woken.  A guest that is merely IDLE looks exactly like that:
                             * COHERENT's idle loop halts and the clock tick is the only thing
                             * that wakes it, so a script that sleeps longer than the watch's
                             * budget is called parked while it is working correctly.  The
                             * channel is therefore selectable like every other, and a harness
                             * whose guest sleeps names its own channels instead */
    const char *stop_mark;  /* --stop-mark: text whose PRINTING ends the run (NULL = disarmed).
                             * The guest says when it is finished, in a channel every guest
                             * already has -- a serial port it is already writing to -- so this
                             * needs no program on the disk and no privileged I/O, unlike the
                             * stop doorbell, and it truncates nothing, unlike --idle: the
                             * moment is still the guest's, it is just announced in output
                             * rather than on a port.  It is watched on EVERY SCC transmit
                             * path, not the console alone, so a guest that reports on a wired
                             * port can end its own run the same way */
    int      stop_mark_pos[SCC_PORTS];
                            /* how much of stop_mark each port's OWN output has matched.  The
                             * state is per port because the ports are independent streams: one
                             * shared counter would let half a mark printed on one port be
                             * completed by bytes from another, ending the run on text that no
                             * channel ever printed */
    const char *stop_why;   /* why the run ended, for the closing message and the exit status */
    bool     max_reached;   /* the run ended by exhausting --max rather than by stopping */

    /* ── user-mode (--exec) hooks; both are off for a machine boot ──
     * flat_mem replaces the board's memory decode (32K ROM at 0, DRAM only in
     * segments 8..0x17, open bus elsewhere) with plain RAM over the whole 24-bit
     * space, which is what a process image needs. */
    bool     flat_mem;
    uint8_t  flat_maxseg; /* highest segment this image installed; above it the
                           * flat space is OPEN BUS (reads 0xFF, writes dropped),
                           * the same as the board's unpopulated space.  A guest
                           * that computes a far pointer into a segment nothing
                           * loaded must not find its own data there -- that is
                           * exactly how a dropped segment half reads as correct. */
    void   (*sys_hook)(struct Machine *m, uint8_t num, uint32_t pc);
};

Machine *machine_new(void);
int  machine_load_rom(Machine *m, const char *dir);
/* Seed the RTC to a wall-clock time (the module has no battery here, so the
 * host has to supply one at start-up) and choose the divider rate. */
void rtc_set_time(Machine *m, int y, int mo, int d, int h, int mi, int s);
void rtc_report(Machine *m, FILE *out);
int  machine_attach_disk(Machine *m, const char *path);
int  machine_attach_floppy(Machine *m, const char *path);
void machine_run(Machine *m);

/* physical memory access (post-translation) */
uint8_t  phys_read8(Machine *m, uint32_t addr);
void     phys_write8(Machine *m, uint32_t addr, uint8_t v);
uint16_t phys_read16(Machine *m, uint32_t addr);
void     phys_write16(Machine *m, uint32_t addr, uint16_t v);

/* I/O access (normal + special). Returns data (word). */
uint16_t io_read(Machine *m, uint16_t port, bool is_byte, bool special);
void     io_write(Machine *m, uint16_t port, uint16_t data, bool is_byte, bool special);

/* Wire glue (host endpoints, see wire.c).  The indexed forms take a port
 * number from the SERIAL PORT TABLE above; the three un-indexed ones are the
 * original entry points and mean port 1 (the motherboard's channel A), which
 * is what --wire has always attached. */
int  wire_open_n(int port, const char *spec, bool trace);   /* 0 on success */
int  wire_poll_char_n(int port);        /* returns byte or -1 */
void wire_put_char_n(int port, int ch);
int  wire_open(const char *spec, bool trace);   /* 0 on success */
void wire_close(void);
int  wire_poll_char(void);      /* returns byte or -1 */
void wire_put_char(int ch);

/* The SCC channel a port number names, or NULL for a port whose board is not
 * fitted.  Defined in bus.c. */
SCCChan *scc_port(Machine *m, int port);
void     scc_wire_service(Machine *m);   /* one pass of the wired receivers */
void     scc_rx_port(Machine *m, int port, uint8_t b);

/* --selftest only: give a port a host endpoint that is the near end of a
 * socketpair, and hand back the far end so the test can play the far machine.
 * Returns -1 where AF_UNIX is not available, and the caller skips. */
int  wire_test_pair(int port, int *host_fd);

/* ── debugging instruments (src/debug.c; --break / --dump) ──
 * All opt-in: dbg_armed stays 0 until a --break is accepted, and the one test
 * in cpu_step's hot path costs a predictable branch.  Records go to stderr as
 * "[dbg] " key=value lines. */
extern int dbg_armed;              /* nonzero: at least one breakpoint is set */
int  dbg_add_break(const char *spec);   /* "SEG:OFF[/N]"        0 = accepted */
int  dbg_add_dump(const char *spec);    /* "SEG:OFF+LEN" | "rrN:DISP+LEN" */
void dbg_stop_at_break(bool on);        /* --stop-on=break */
bool dbg_pc_hit(CPU *c);                /* pre-execute hook; true = stop now */
void dbg_finish(Machine *m);            /* end-of-run memory windows */

/* console glue */
void console_init(void);
void console_shutdown(void);
int  console_poll_char(void);   /* returns byte or -1 */
void console_put_char(int ch);

#endif /* EMU_H */
