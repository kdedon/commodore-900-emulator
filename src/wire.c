/* wire.c — SCC channel A (the guest's /dev/tty51) on a host endpoint.
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

static int     g_wire = -1;
static bool    g_trace;
static uint8_t g_buf[8192];
static int     g_len, g_pos;

/* Hexdump one direction of the wire.  Not a debug leftover: this is the only
 * place the link can be seen as bytes, and a wire carrying nothing looks
 * exactly like a guest that never transmitted without it. */
static void trace_bytes(const char *dir, const uint8_t *p, int n){
    fprintf(stderr, "[wire %s %d]", dir, n);
    for (int i = 0; i < n; i++) fprintf(stderr, " %02X", p[i]);
    fprintf(stderr, "\n");
}

int wire_open(const char *spec, bool trace){
    const char *path = spec;
    struct sockaddr_un sa;
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
            g_wire = fd;
            g_trace = trace;
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
    if (g_wire >= 0) { close(g_wire); g_wire = -1; }
}

/* One received byte, or -1 when the wire has nothing waiting.  Reads are
 * buffered: the run loop asks for a byte only as the guest's receiver empties,
 * and a recv() per byte at that rate is most of the cost of running attached. */
int wire_poll_char(void){
    if (g_wire < 0) return -1;
    if (g_pos >= g_len) {
        int n = (int)recv(g_wire, g_buf, sizeof g_buf, 0);
        if (n <= 0) return -1;                  /* nothing ready, or peer gone */
        if (g_trace) trace_bytes("in ", g_buf, n);
        g_len = n; g_pos = 0;
    }
    return g_buf[g_pos++];
}

/* Send one transmitted byte.  A full socket buffer drops the byte rather than
 * stalling the machine: the modelled transmitter is always empty, so the only
 * thing that can back up here is the host, and a real line drops too when the
 * far end has stopped listening. */
void wire_put_char(int ch){
    uint8_t b = (uint8_t)ch;
    if (g_wire < 0) return;
    if (g_trace) trace_bytes("out", &b, 1);
    while (send(g_wire, &b, 1, MSG_NOSIGNAL) < 0 && errno == EINTR)
        ;
}

#else   /* no AF_UNIX sockets here */

int  wire_open(const char *spec, bool trace){
    (void)spec; (void)trace;
    fprintf(stderr, "--wire is not supported on this host\n");
    return -1;
}
void wire_close(void){}
int  wire_poll_char(void){ return -1; }
void wire_put_char(int ch){ (void)ch; }

#endif
