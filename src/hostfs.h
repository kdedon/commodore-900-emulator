/* hostfs.h -- the host filesystem services the COHERENT system-call layer in
 * uexec.c is built on, on every host it runs on.
 *
 * On a POSIX host these are the system's own calls and this header is little
 * more than the include list.  On Windows the C runtime spells most of them
 * with a leading underscore, is missing a few outright, and -- the one that
 * silently corrupts data rather than failing -- opens files in TEXT mode, where
 * every 0x0A written grows an 0x0D in front of it and every 0x0D 0x0A read
 * collapses to one byte.  The guest reads and writes object files through these
 * calls, so a text-mode descriptor does not fail, it produces a wrong object.
 *
 * What Windows cannot do at all is answered here, once, rather than at each
 * call site: no FIFOs, no symbolic links, and hard links only through the Win32
 * entry point.  Each returns what a POSIX host returns when the filesystem
 * refuses, so the guest sees an errno it already has a branch for.
 */
#ifndef HOSTFS_H
#define HOSTFS_H

/* Before ANY system header: these select which declarations the C library
 * exposes, and -std=c99 alone exposes only ISO C.  A header included ahead of
 * them settles the question first and lstat, strdup and utimes stay hidden. */
#if !defined(_WIN32)
#define _XOPEN_SOURCE 700
#define _DEFAULT_SOURCE 1
#endif

#include <stdint.h>

#if !defined(_WIN32)

#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <errno.h>

#ifndef O_BINARY
#define O_BINARY 0			/* every descriptor is already binary */
#endif
#define host_stdio_binary() ((void)0)
#define host_mkdir(p,m)  mkdir((p), (m))
#define host_mkfifo(p,m) mkfifo((p), (m))
#define host_link(a,b)   link((a), (b))
#define host_lstat(p,sb) lstat((p), (sb))
#define host_ino(p,sb)   ((uint64_t)(sb)->st_ino)

/* host_now and host_utime are the only two clock/timestamp operations the
 * syscall layer performs; naming them keeps `struct timeval' -- which Windows
 * defines in a socket header -- out of the portable path entirely. */
static inline void host_now(uint32_t *sec, uint16_t *msec){
	struct timeval tv;
	gettimeofday(&tv, NULL);
	*sec = (uint32_t)tv.tv_sec;
	*msec = (uint16_t)(tv.tv_usec / 1000);
}
static inline int host_utime(const char *p, long atime, long mtime){
	struct timeval tvs[2];
	tvs[0].tv_sec = atime; tvs[0].tv_usec = 0;
	tvs[1].tv_sec = mtime; tvs[1].tv_usec = 0;
	return utimes(p, tvs);
}

#else  /* _WIN32 */

#include <stdlib.h>			/* _fullpath */
#include <io.h>
#include <direct.h>
#include <process.h>
#include <fcntl.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/timeb.h>
#include <sys/utime.h>
#include <time.h>
#include <errno.h>

#define open   _open
#define read   _read
#define write  _write
#define close  _close
#define lseek  _lseek
#define unlink _unlink
#define chdir  _chdir
#define chmod  _chmod
#define isatty _isatty
#define dup    _dup
#define umask  _umask
#define getpid _getpid

/* access(2)'s modes.  X_OK is 0 rather than 1: Windows has no execute
 * permission bit, and _access rejects mode 1 outright, so asking whether a file
 * is executable becomes asking whether it exists -- which is the answer that
 * lets a caller proceed to the open it was going to do anyway. */
#ifndef R_OK
#define R_OK 4
#define W_OK 2
#define X_OK 0
#define F_OK 0
#endif
#define access _access

/* The set-id and sticky bits have no Windows equivalent, so no file carries
 * them; the guest is told so rather than being told nothing. */
#ifndef S_ISUID
#define S_ISUID 0
#endif
#ifndef S_ISGID
#define S_ISGID 0
#endif
#ifndef S_ISVTX
#define S_ISVTX 0
#endif
#ifndef S_ISBLK
#define S_ISBLK(m) (0)
#endif
#ifndef S_ISFIFO
#define S_ISFIFO(m) (0)
#endif

/* Declared rather than reached through <windows.h>, which would define `far',
 * `IN' and `OUT' over this file's own names. */
__declspec(dllimport) int __stdcall CreateHardLinkA(const char *, const char *, void *);
__declspec(dllimport) unsigned long __stdcall GetLastError(void);

#define host_mkdir(p,m)  (((void)(m)), _mkdir(p))
#define host_lstat(p,sb) stat((p), (sb))	/* no symbolic links */

static inline int host_mkfifo(const char *p, int m){
	(void)p; (void)m;
	errno = EPERM;				/* no FIFOs on this host */
	return -1;
}
static inline int host_link(const char *from, const char *to){
	if (CreateHardLinkA(to, from, NULL)) return 0;
	switch (GetLastError()) {
	case 80:  case 183: errno = EEXIST; break;	/* FILE_EXISTS, ALREADY_EXISTS */
	case 2:   case 3:   errno = ENOENT; break;	/* FILE_NOT_FOUND, PATH_NOT_FOUND */
	case 5:             errno = EACCES; break;	/* ACCESS_DENIED */
	default:            errno = EPERM;  break;	/* including a volume with no hard links */
	}
	return -1;
}
/* The file identity a guest inode is folded from.  Windows leaves st_ino 0 for
 * EVERY file, so folding it would give every file the same inode -- which is
 * the one answer guest_ino must not give: cp, mv and ln read equal inodes as
 * "these are the same file and there is nothing to do", and getwd(3) stops at
 * the first entry of "..".  The canonical path is the identity used instead:
 * one file reached by two spellings still agrees, which is what those callers
 * test.  Two hard links to one file do not, and that is the divergence -- the
 * guest's own use of link(2) is mkdir(1) building "." and "..", which the link
 * case answers before any inode is compared. */
static inline uint64_t host_ino(const char *path, const struct stat *st){
	(void)st;
	char full[1024];
	const char *p = _fullpath(full, path, sizeof full) ? full : path;
	uint64_t h = 1469598103934665603ull;		/* FNV-1a */
	for (; *p; p++) {
		unsigned char c = (unsigned char)*p;
		if (c >= 'A' && c <= 'Z') c += 32;	/* the filesystem is case-insensitive */
		if (c == '\\') c = '/';
		h = (h ^ c) * 1099511628211ull;
	}
	return h;
}
static inline void host_now(uint32_t *sec, uint16_t *msec){
	struct __timeb64 tb;
	_ftime64(&tb);
	*sec = (uint32_t)tb.time;
	*msec = (uint16_t)tb.millitm;
}
static inline int host_utime(const char *p, long atime, long mtime){
	struct _utimbuf ub;
	ub.actime = (time_t)atime;
	ub.modtime = (time_t)mtime;
	return _utime(p, &ub);
}

/* The three inherited descriptors are put in binary mode before the guest is
 * started: a guest writing an object file to a redirected stdout would
 * otherwise have an 0x0D inserted before every 0x0A in it. */
static inline void host_stdio_binary(void){
	_setmode(0, _O_BINARY);
	_setmode(1, _O_BINARY);
	_setmode(2, _O_BINARY);
}

#endif /* _WIN32 */
#endif /* HOSTFS_H */
