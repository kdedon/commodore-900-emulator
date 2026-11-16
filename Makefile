# make        build bin/c900 (bin/c900.exe on Windows, with MinGW-w64)
# make test   build and run the self-test
# make clean  remove the binary

CC      ?= gcc
# -flto inlines the memory accessors into the CPU loop.
CFLAGS  ?= -O3 -flto -std=c99 -Wall
SRCDIR   = src
BINDIR   = bin
SRCS     = $(SRCDIR)/alu.c $(SRCDIR)/decode.c $(SRCDIR)/cpu.c \
           $(SRCDIR)/mmu.c $(SRCDIR)/bus.c $(SRCDIR)/wire.c \
           $(SRCDIR)/uexec.c $(SRCDIR)/debug.c $(SRCDIR)/lc.c $(SRCDIR)/prof.c $(SRCDIR)/main.c
HDRS     = $(SRCDIR)/emu.h $(SRCDIR)/hostfs.h

ifeq ($(OS),Windows_NT)
    EXE := .exe
else
    EXE :=
endif

TARGET = $(BINDIR)/c900$(EXE)
RM ?= rm -f

all: $(TARGET)

# One compiler invocation, so -flto sees every file.
$(TARGET): $(SRCS) $(HDRS) | $(BINDIR)
	$(CC) $(CFLAGS) -o $@ $(SRCS)

$(BINDIR):
	mkdir -p $(BINDIR)

test: $(TARGET)
	./$(TARGET) --selftest

clean:
	$(RM) $(TARGET)

.PHONY: all test clean
