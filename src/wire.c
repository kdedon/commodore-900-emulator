/* wire.c — an SCC channel on a host endpoint.
 *
 * There are WIRE_MAX independently-attachable serial ports (see emu.h's
 * SERIAL PORT TABLE): the motherboard SCC's channel A, which is what --wire
 * has always meant and is still port 1, and the LR board's four.  Each holds
 * its own socket, its own read buffer and its own trace flag; the un-indexed
 * wire_open/wire_poll_char/wire_put_char below are exactly the old entry
 * points, hard-wired to port 1, so nothing that used them has changed.
 *
 * The console is channel B; this is the machine's SECOND RS-232 port, and it is
 * the line /etc/rc.net attaches SLIP to.  Two emulators whose channel A meets
 * in one host bridge are therefore two machines on one point-to-point serial
 * link, with no change to the guest at all.
 *
 * The endpoint is an AF_UNIX stream socket rather than a pty because what
 * crosses it is binary: SLIP delimits frames with 0xC0 and escapes with 0xDB,
 * and a pty in anything but fully raw mode rewrites CR/LF and acts on ^C, ^S
 * and ^Z.  A socket has no line discipline to get wrong, is 8-bit clean by
 * construction, and reports a definite end when the far end goes away.
 *
 * This file and the console_* functions in main.c are the only host-OS-specific
 * code in the emulator.  The feature-test macro is why it is a file of its own:
 * the rest of the emulator is strict C99, which hides the socket API. */
#define _GNU_SOURCE
#include "emu.h"
#include <stdio.h>
#include <string.h>

#ifndef _WIN32
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/un.h>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0        /* elsewhere SIGPIPE is disarmed in wire_open() */
#endif

/* One host endpoint per modelled port.  Ports are numbered as emu.h's table
 * numbers them; index 0 is the console (never wired) and stays unused so a
 * port number can be used as an index without arithmetic at every call site. */
typedef struct {
    int     fd;
    bool    trace;
    uint8_t buf[8192];
    int     len, pos;
} WireEnd;

static WireEnd g_end[WIRE_MAX];
static bool    g_init;

static WireEnd *end_of(int port){
    if (!g_init) {                       /* -1 is "closed"; 0 is a real fd */
        for (int i = 0; i < WIRE_MAX; i++) g_end[i].fd = -1;
        g_init = true;
    }
    if (port < 0 || port >= WIRE_MAX) return NULL;
    return &g_end[port];
}

/* Hexdump one direction of the wire.  Not a debug leftover: this is the only
 * place the link can be seen as bytes, and a wire carrying nothing looks
 * exactly like a guest that never transmitted without it. */
static void trace_bytes(int port, const char *dir, const uint8_t *p, int n){
    fprintf(stderr, "[wire%d %s %d]", port, dir, n);
    for (int i = 0; i < n; i++) fprintf(stderr, " %02X", p[i]);
    fprintf(stderr, "\n");
}

int wire_open_n(int port, const char *spec, bool trace){
    const char *path = spec;
    struct sockaddr_un sa;
    WireEnd *e = end_of(port);
    if (!e) { fprintf(stderr, "--wire: no such port %d\n", port); return -1; }
    if (!strncmp(spec, "unix:", 5)) path = spec + 5;
    if (strlen(path) >= sizeof sa.sun_path) {
        fprintf(stderr, "--wire: path too long: %s\n", path);
        return -1;
    }
    memset(&sa, 0, sizeof sa);
    sa.sun_family = AF_UNIX;
    strcpy(sa.sun_path, path);
    signal(SIGPIPE, SIG_IGN);        /* a peer that exits must not kill us */

    /* The bridge may still be starting.  Retrying for a few seconds keeps a
     * two-machine harness from depending on which process wins the race. */
    for (int attempt = 0; attempt < 50; attempt++) {
        int fd = socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd < 0) { perror("--wire: socket"); return -1; }
        if (connect(fd, (struct sockaddr *)&sa, sizeof sa) == 0) {
            int fl = fcntl(fd, F_GETFL);
            fcntl(fd, F_SETFL, fl | O_NONBLOCK);
            e->fd = fd;
            e->trace = trace;
            return 0;
        }
        close(fd);
        struct timespec ts; ts.tv_sec = 0; ts.tv_nsec = 100000000L;   /* 100 ms */
        nanosleep(&ts, NULL);
    }
    fprintf(stderr, "--wire: cannot connect to %s: %s\n", path, strerror(errno));
    return -1;
}

void wire_close(void){
    for (int i = 0; i < WIRE_MAX; i++) {
        WireEnd *e = end_of(i);
        if (e && e->fd >= 0) { close(e->fd); e->fd = -1; }
    }
}

/* One received byte, or -1 when the wire has nothing waiting.  Reads are
 * buffered: the run loop asks for a byte only as the guest's receiver empties,
 * and a recv() per byte at that rate is most of the cost of running attached. */
int wire_poll_char_n(int port){
    WireEnd *e = end_of(port);
    if (!e || e->fd < 0) return -1;
    if (e->pos >= e->len) {
        int n = (int)recv(e->fd, e->buf, sizeof e->buf, 0);
        if (n <= 0) return -1;                  /* nothing ready, or peer gone */
        if (e->trace) trace_bytes(port, "in ", e->buf, n);
        e->len = n; e->pos = 0;
    }
    return e->buf[e->pos++];
}

/* Send one transmitted byte.  A full socket buffer drops the byte rather than
 * stalling the machine: the modelled transmitter is always empty, so the only
 * thing that can back up here is the host, and a real line drops too when the
 * far end has stopped listening. */
void wire_put_char_n(int port, int ch){
    uint8_t b = (uint8_t)ch;
    WireEnd *e = end_of(port);
    if (!e || e->fd < 0) return;
    if (e->trace) trace_bytes(port, "out", &b, 1);
    while (send(e->fd, &b, 1, MSG_NOSIGNAL) < 0 && errno == EINTR)
        ;
}

/* --selftest support.  A socketpair rather than a bound path: the test needs
 * both ends and no filesystem, and the emulator's end goes through exactly the
 * buffering and non-blocking recv() a real --wire endpoint does. */
int wire_test_pair(int port, int *host_fd){
    int sv[2];
    WireEnd *e = end_of(port);
    if (!e) return -1;
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sv) != 0) return -1;
    for (int i = 0; i < 2; i++) {
        int fl = fcntl(sv[i], F_GETFL);
        fcntl(sv[i], F_SETFL, fl | O_NONBLOCK);
    }
    if (e->fd >= 0) close(e->fd);
    e->fd = sv[0]; e->trace = false; e->len = e->pos = 0;
    *host_fd = sv[1];
    return 0;
}

#else   /* no AF_UNIX sockets here */

int  wire_open_n(int port, const char *spec, bool trace){
    (void)port; (void)spec; (void)trace;
    fprintf(stderr, "--wire is not supported on this host\n");
    return -1;
}
void wire_close(void){}
int  wire_poll_char_n(int port){ (void)port; return -1; }
void wire_put_char_n(int port, int ch){ (void)port; (void)ch; }
int  wire_test_pair(int port, int *host_fd){ (void)port; (void)host_fd; return -1; }

#endif

/* The original, un-indexed entry points.  --wire has always meant the
 * motherboard SCC's channel A, so these are that port and nothing else. */
int  wire_open(const char *spec, bool trace){ return wire_open_n(SERPORT_MB_A, spec, trace); }
int  wire_poll_char(void){ return wire_poll_char_n(SERPORT_MB_A); }
void wire_put_char(int ch){ wire_put_char_n(SERPORT_MB_A, ch); }
