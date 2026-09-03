# Minimal Commodore 900 emulator

Instruction-level Commodore 900 emulator, based on kevind's HDL Commodore 900 core.

## How it works

It emulates only the bare minimum necessary for software cross-development on the host machine:

* Z8001 CPU + Z8010 MMU
* 1 MB RAM
* 20 MB hard drive
* Optional floppy drive (see `--floppy` below)
* Both SCC serial channels: 
  - channel B (base 0x100, the guest's `/dev/tty50`) is the console and redirects to the host terminal
  - channel A (base 0x120, the guest's `/dev/tty51`) is the second RS-232 port and can be attached to a host socket with `--wire` (see below)

No graphics card at present.

## Running

Run the emulator from the `bin/` directory, since the defaults are resolved relative to the working directory:

```sh
cd bin
./c900
```

The command line accepts the following arguments. Value options may be written either as `--opt=VALUE` or `--opt VALUE`:

- `--firmware=DIR` — directory holding the BIOS ROMs `bios_h.bin` and `bios_l.bin` (default `../rom`).
- `--disk=FILE` — raw hard-disk image (default `../disk/hdd.bin`). A hard disk is mandatory; the emulator exits with an error if the image can't be opened.
- `--floppy=FILE` — raw floppy image, attached to the floppy drive (`/dev/fd1`). Optional; when omitted the floppy drive is empty. The image is a flat 512-byte-per-sector Coherent floppy (up to 2392 blocks); a short image has its trailing sectors read back as zeros.
- `--trace` — print periodic PC/FCW progress to stderr.
- `--max=N` — stop after N instructions (0, the default, runs until Ctrl-]).
- `--input="..."` — feed scripted console keystrokes, with `\r`, `\n`, `\t`, and `\\` escapes,
  plus the `\g` and `\i` pacing marks. See "Scripted console input" below.
- `--input-mark=TEXT` — hold every `\i` (type-ahead) byte until the guest has *printed*
  TEXT on the console. See "Scripted console input" below.
- `--wire=PATH` — attach SCC channel A (the guest's `/dev/tty51`) to the AF_UNIX
  stream socket `PATH`. The console is unaffected. Two emulators pointed at one
  socket — with a host program in the middle copying each end's bytes to the
  other — are two machines on one point-to-point serial line, which is what the
  guest OS needs to run SLIP between them.
- `--break=SEG:OFF[/N]` — record the CPU state at a guest PC. See "Breakpoints and memory dumps" below.
- `--dump=SPEC` — record a window of guest memory, at every breakpoint hit and once at the end of the run. See "Breakpoints and memory dumps" below.
- `--stop-on=LIST` — which early-stop channels are armed: `idle`, `port`, `break`, `all` (the default), or `none` to run to `--max` whatever happens. See "Stopping a scripted run" below.
- `--idle=N` — instructions of console silence the `idle` channel waits for (default 40000000).
- `--stop-port=P` — the `port` channel's I/O port (default `0x0FFE`).
- `--require-stop` — exit 3 if the run ended by exhausting `--max` rather than by stopping, so a caller can assert that the session *finished* rather than merely that the emulator survived.
- `--wire-trace` — hexdump every byte crossing `--wire` to stderr.
- `--selftest` — run the built-in CPU/ALU regression and exit (needs neither ROM nor disk).
- `--help`, `-h` — print the option list and exit.

For example, to boot with a floppy image attached:

```sh
./c900 --floppy ../disk/disk1_hr.bin
```

## Scripted console input

`--input` queues bytes for the console receiver, but it does not hand them over
as fast as it can. Each byte is **paced**: it waits until the guest looks ready
to read it — a prompt character (`#`, `>`) has been printed, the console has
been quiet for a while, or the guest is spinning on the receiver status
register. That pacing is not politeness, it is correctness. The modelled SCC
receiver holds one byte, and whoever reads next gets it: a byte handed over too
early is swallowed by a boot-ROM probe, by a still-running command, or by a
program's own output path — CP/M's BDOS polls the keyboard between the
characters it prints, looking for `^S`/`^Q`/`^C`. A swallowed byte is simply
gone, and the script silently runs on one line short.

Two escapes queue no byte of their own and change that pacing:

- `\g` — from this point in the script on, stop waiting for a prompt. A
  full-screen program never prints one, so without this the first `\r` typed
  into `ED` is the last byte it can ever receive. The quiet-console wait still
  applies.
- `\i` — the **next** byte is *type-ahead*: it is handed to the receiver the
  moment the receiver is free, with no gate, no prompt and no quiet wait, the
  way a person typing ahead of a running program delivers one. It applies to
  exactly one byte; the rest of the script stays paced.

`\i` is how you reach a guest that is *printing*. The pacing above deliberately
waits for a guest that looks idle, and a guest in a print loop never does — so
a scripted `^S` could never arrive while there was still output to stop, which
is the only moment at which it means anything. `\i` puts that judgement back in
the hands of whoever wrote the script.

`--input-mark=TEXT` says *when*: no `\i` byte is released until the guest has
printed TEXT on the console. This is the one synchronisation a test can state
exactly — "send it the moment that appears" — and, unlike an instruction count,
it does not move when the guest is rebuilt. The mark is a starting gun rather
than a gate: once seen it stays open, and paced bytes are never affected by it.

```sh
# type ^S into a program's output the instant it announces itself, and
# nothing else ever: the guest stops printing and stays stopped.
./c900 --disk=cpm.bin --input="$(printf 'CONBRK P 0200\r\i\023')" \
       --input-mark='CONBRK-START' --max=150000000
```

## Stopping a scripted run

There are two channels that can stop a run early and `--stop-on` selects them:
* `--stop-on=all` (the default) arms both
* `--stop-on=none` restores the old behaviour exactly
* `--stop-on=idle` The run ends when all
three of these hold: the scripted input is exhausted, the last byte the guest
transmitted on the console was a prompt character, and it has transmitted
nothing for another 40M instructions.
* `--stop-on=port` A word write of `0xC900` to an unmapped I/O port ends the run. 
* `--stop-on=break` The run ends once a `--break` has recorded its last hit, with the
instruction at that PC not yet executed.

## Breakpoints and memory dumps

`--break` and `--dump` answer the two questions a guest cannot be asked from
inside itself: *what were the registers when execution reached here*, and
*what is at this address*. Both are off unless given, both write to stderr,
and neither changes a single byte of guest state — memory windows are read
through the MMU exactly as the guest would read them, and everything the MMU
remembers about having been asked is put back afterwards.

- `--break=SEG:OFF[/N]` records one line per hit *before* the instruction at
  that PC executes, so "entry to a routine" means entry. Addresses are hex.
  `SEG` may be a segment **number** (`30`) or a Z8001 segment **word**
  (`3000`), so a PC copied out of a guest trap message pastes in unchanged.
  `/N` (decimal, default 1) is how many hits to record; `/0` records every
  one, which for a hot address is a great deal of output. Repeatable, up to 8.
- `--dump=SPEC` records a window of memory at every `--break` hit *and* once
  when the run ends, whatever ended it. Two forms:
  - `SEG:OFF+LEN` — a fixed logical address.
  - `rrN:DISP+LEN` — relative to the long pointer `RRn`: segment from `Rn`
    (as a segment word), offset from `Rn+1`, plus `DISP`. `rr14:0+36` is the
    stack frame, wherever it happens to be on this run, which is the form a
    trap frame needs. `DISP` is hex and may be negative; `LEN` is decimal.

  Repeatable, up to 8.

Output is one record per line, whitespace-separated `key=value` fields behind
a `[dbg]` tag, because the readers are test harnesses and agents rather than
people. The CP/M suite already sends this emulator's stderr to `/dev/null`, so
an armed run adds nothing to a transcript anyone else is reading.

```
[dbg] brk pc=30:9AD0 hit=2 insns=7729880 fcw=C040 sys=1 op=A1EC r0=7FFF r1=0000 r2=F900 ... r15=FBDC
[dbg] mem at=brk insns=7729880 win=0 spec=rr14:0+36 addr=3F:FBDC phys=0DFBDC len=36 data=00000000F900... fault=0
```

`fault=1` on a `mem` record means some byte of the window did not translate
(those read back as `FF`); `at=` is `brk` for a window taken at a breakpoint
and `end` for the one taken when the run stops.

### Worked example: the split-I/D fast path

The regression documented in the CP/M repository's `docs/SPLITID-BISECT.md`
was eight instructions in `src/bdos/splitfast.s` that assembled base and index
the wrong way round, so every saved-register access in the SC-trap fast path
went through dispatch scratch. Two host-side tests exercised the same shim's C
half and passed throughout; nothing on the host runs the assembler half. It
was found by building temporary instruments into the guest and rebuilding it —
which is the worst possible way to inspect the thing you already doubt.

The same evidence, with no guest change at all: break at the fast path's entry
and dump the trap frame the SC handler left at `RR14` (saved `r0`–`r13`, then
the identifier, FCW, PC segment and PC offset).

```sh
./c900 --disk=arxtest.bin --input="ASZ8K MINI.8KN\r" \
       --break=30:9AD0/4 --dump=rr14:0+36 --stop-on=idle,break --max=300000000
```

Working system — the saved-register slots fill in as the shim emulates each
access (`r2`, then `r4`, then `r1`/`r6`), and the saved PC advances:

```
data=000000000000...00000000 7FFF 0040 B200 5700
data=00000000F9000000...0000 7FFF 0040 B200 5704
data=00000000F90000002D2A... 7FFF 0040 B200 5708
data=00001EB8F90000002D2A... 7FFF 0000 B200 5714
```

Broken system — four consecutive traps, every saved register still zero, only
the PC moving. The emulated accesses are landing somewhere else:

```
data=000000000000...00000000 7FFF 0040 B200 5700
data=000000000000...00000000 7FFF 0040 B200 5704
data=000000000000...00000000 7FFF 0040 B200 5708
data=000000000000...00000000 7FFF 0040 B200 571A
```

## Running one program instead of a machine

The emulator can run a single linked Z8001 `l.out` as a **process**, with no
ROM and disk. The image is loaded into its segments, a stack with
`argc`/`argv`/`envp` is built under it, and the COHERENT system calls it makes
are serviced against the host filesystem.

```
c900 --exec ./prog [args...]     # run crt0+main against the syscall shim
c900 -runobjint ./a.out 6 7      # CALL f(6,7) in a linked object, print R1
c900 -runobj ./a.out A B WANT    # the same with IEEE double operands
```

The mode word must come first, because **every argument after the program's
path belongs to the guest**.  `--exec` exits with the guest's own exit status; `[exit N]`
is also printed on stderr.  A run that faulted or exhausted the instruction
budget never reached an exit: it prints `[no exit: ...]` instead and exits 4.

Three environment variables can be defined: 
* `N2ROOT` prefixes the guest's absolute paths (so a program finds the data files its
image would have had)
* `N2ENV` supplies a colon-separated environment
* `N2ACCEPT`/`N2STRICT` decide which unimplemented system calls a run may
survive.

The default for an unimplemented call is to **stop the run** and
name it, rather than return a value.  A handful of calls that cannot exist here at all (`fork`, `wait`, `exec`, `pipe`) or that are better left failing (`alarm`, `pause`,
`kill`) are allowed through with a warning that is repeated at the end of the
run.  Guest faults — an illegal instruction, a privileged instruction, a
segment trap — are reported with the instruction and its address, so a program
that died is never confused with a host-side failure.

## Building

To build the emulator, simply type `make`. gcc is used for compilation, but since the C files are straight C99, other compilers should work too.

## Tools

BIOS dump and a hard disk image with Coherent system installed are provided so that the emulator can run without any dependencies. The software is used with permission of Robert Swartz, the copyright holder on Mark Williams software.

A Python script in the `tools/` directory can be used to examine disk images and copy files to/from them.
