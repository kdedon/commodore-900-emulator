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
- `--input="..."` — feed scripted console keystrokes, with `\r`, `\n`, `\t`, and `\\` escapes.
- `--wire=PATH` — attach SCC channel A (the guest's `/dev/tty51`) to the AF_UNIX
  stream socket `PATH`. The console is unaffected. Two emulators pointed at one
  socket — with a host program in the middle copying each end's bytes to the
  other — are two machines on one point-to-point serial line, which is what the
  guest OS needs to run SLIP between them.
- `--stop-on=LIST` — which early-stop channels are armed: `idle`, `port`, `all` (the default), or `none` to run to `--max` whatever happens. See "Stopping a scripted run" below.
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

## Stopping a scripted run

There are two channels that can stop a run early and `--stop-on` selects them:
* `--stop-on=all` (the default) arms both
* `--stop-on=none` restores the old behaviour exactly
* `--stop-on=idle` The run ends when all
three of these hold: the scripted input is exhausted, the last byte the guest
transmitted on the console was a prompt character, and it has transmitted
nothing for another 40M instructions.
* `--stop-on=port` A word write of `0xC900` to an unmapped I/O port ends the run. 

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
