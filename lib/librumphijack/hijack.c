/*      $NetBSD: hijack.c,v 1.1 2011/01/07 19:52:43 pooka Exp $	*/

/*-
 * Copyright (c) 2011 Antti Kantee.  All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__RCSID("$NetBSD: hijack.c,v 1.1 2011/01/07 19:52:43 pooka Exp $");

#include <sys/param.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/poll.h>

#include <rump/rump.h>
#include <rump/rumpclient.h>
#include <rump/rump_syscalls.h>

#include <assert.h>
#include <dlfcn.h>
#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

/* XXX: need runtime selection.  low for now due to FD_SETSIZE */
#define HIJACK_FDOFF 128

enum {	RUMPCALL_SOCKET, RUMPCALL_ACCEPT, RUMPCALL_BIND, RUMPCALL_CONNECT,
	RUMPCALL_GETPEERNAME, RUMPCALL_GETSOCKNAME, RUMPCALL_LISTEN,
	RUMPCALL_RECVFROM, RUMPCALL_RECVMSG,
	RUMPCALL_SENDTO, RUMPCALL_SENDMSG,
	RUMPCALL_GETSOCKOPT, RUMPCALL_SETSOCKOPT,
	RUMPCALL_SHUTDOWN,
	RUMPCALL_READ, RUMPCALL_READV,
	RUMPCALL_WRITE, RUMPCALL_WRITEV,
	RUMPCALL_IOCTL, RUMPCALL_FCNTL,
	RUMPCALL_CLOSE,
	RUMPCALL_SELECT, RUMPCALL_POLL, RUMPCALL_POLLTS,
	RUMPCALL__NUM
};

const char *sysnames[] = {
	"__socket30",
	"accept",
	"bind",
	"connect",
	"getpeername",
	"getsockname",
	"listen",
	"recvfrom",
	"recvmsg",
	"sendto",
	"sendmsg",
	"getsockopt",
	"setsockopt",
	"shutdown",
	"read",
	"readv",
	"write",
	"writev",
	"ioctl",
	"fcntl",
	"close",
	"__select50",
	"poll",
	"__pollts50",
};

static ssize_t	(*host_read)(int, void *, size_t);
static ssize_t	(*host_readv)(int, const struct iovec *, int);
static ssize_t	(*host_write)(int, const void *, size_t);
static ssize_t	(*host_writev)(int, const struct iovec *, int);
static int	(*host_ioctl)(int, unsigned long, ...);
static int	(*host_fcntl)(int, int, ...);
static int	(*host_close)(int);
static int	(*host_select)(int, fd_set *, fd_set *, fd_set *,
			       struct timeval *);
static int	(*host_poll)(struct pollfd *, nfds_t, int);
#if 0
static int	(*host_pollts)(struct pollfd *, nfds_t,
			       const struct timespec *, const sigset_t *);
#endif

#define assertfd(_fd_) assert((_fd_) >= HIJACK_FDOFF)

static void *rumpcalls[RUMPCALL__NUM];

/*
 * This is called from librumpclient in case of LD_PRELOAD.
 * It ensures correct RTLD_NEXT.
 */
static void *
hijackdlsym(void *handle, const char *symbol)
{

	return dlsym(handle, symbol);
}

static void __attribute__((constructor))
rcinit(void)
{
	int (*rumpcinit)(void);
	void **rumpcdlsym;
	void *hand;
	int i;

	hand = dlopen("librumpclient.so", RTLD_LAZY|RTLD_GLOBAL);
	if (!hand)
		err(1, "cannot open librumpclient.so");
	rumpcinit = dlsym(hand, "rumpclient_init");
	_DIAGASSERT(rumpcinit);

	rumpcdlsym = dlsym(hand, "rumpclient_dlsym");
	*rumpcdlsym = hijackdlsym;

	host_read = dlsym(RTLD_NEXT, "read");
	host_readv = dlsym(RTLD_NEXT, "readv");
	host_write = dlsym(RTLD_NEXT, "write");
	host_writev = dlsym(RTLD_NEXT, "writev");
	host_ioctl = dlsym(RTLD_NEXT, "ioctl");
	host_fcntl = dlsym(RTLD_NEXT, "fcntl");
	host_close = dlsym(RTLD_NEXT, "close");
	host_select = dlsym(RTLD_NEXT, "select");
	host_poll = dlsym(RTLD_NEXT, "poll");

	for (i = 0; i < RUMPCALL__NUM; i++) {
		char sysname[128];

		snprintf(sysname, sizeof(sysname), "rump_sys_%s", sysnames[i]);
		rumpcalls[i] = dlsym(hand, sysname);
		if (!rumpcalls[i]) {
			fprintf(stderr, "%s\n", sysname);
			exit(1);
		}
	}

	if (rumpcinit() == -1)
		err(1, "rumpclient init");
}

#define ADJ(fd) (fd - HIJACK_FDOFF)
//#define DEBUGJACK
#ifdef DEBUGJACK
#define DPRINTF(x) printf x
#else
#define DPRINTF(x)
#endif

/*
 * Following wrappers always call the rump kernel.
 */

int __socket30(int, int, int);
int
__socket30(int domain, int type, int protocol)
{
	int (*rc_socket)(int, int, int);
	int fd;

	DPRINTF(("socket\n"));
	rc_socket = rumpcalls[RUMPCALL_SOCKET];
	fd = rc_socket(domain, type, protocol);
	if (fd != -1)
		fd += HIJACK_FDOFF;
	return fd;
}

int
accept(int s, struct sockaddr *addr, socklen_t *addrlen)
{
	int (*rc_accept)(int, struct sockaddr *, socklen_t *);
	int fd;

	DPRINTF(("accept %d\n", s));
	assertfd(s);
	rc_accept = rumpcalls[RUMPCALL_ACCEPT];
	fd = rc_accept(ADJ(s), addr, addrlen);
	if (fd != -1)
		fd += HIJACK_FDOFF;
	return fd;
}

int
bind(int s, const struct sockaddr *name, socklen_t namelen)
{
	int (*rc_bind)(int, const struct sockaddr *, socklen_t);

	DPRINTF(("bind\n"));
	assertfd(s);
	rc_bind = rumpcalls[RUMPCALL_BIND];
	return rc_bind(ADJ(s), name, namelen);
}

int
connect(int s, const struct sockaddr *name, socklen_t namelen)
{
	int (*rc_connect)(int, const struct sockaddr *, socklen_t);

	DPRINTF(("connect %d\n", s));
	assertfd(s);
	rc_connect = rumpcalls[RUMPCALL_CONNECT];
	return rc_connect(ADJ(s), name, namelen);
}

int
getpeername(int s, struct sockaddr *name, socklen_t *namelen)
{
	int (*rc_getpeername)(int, struct sockaddr *, socklen_t *);

	DPRINTF(("getpeername\n"));
	assertfd(s);
	rc_getpeername = rumpcalls[RUMPCALL_GETPEERNAME];
	return rc_getpeername(ADJ(s), name, namelen);
}

int
getsockname(int s, struct sockaddr *name, socklen_t *namelen)
{
	int (*rc_getsockname)(int, struct sockaddr *, socklen_t *);

	DPRINTF(("getsockname\n"));
	assertfd(s);
	rc_getsockname = rumpcalls[RUMPCALL_GETSOCKNAME];
	return rc_getsockname(ADJ(s), name, namelen);
}

int
listen(int s, int backlog)
{
	int (*rc_listen)(int, int);

	DPRINTF(("listen\n"));
	assertfd(s);
	rc_listen = rumpcalls[RUMPCALL_LISTEN];
	return rc_listen(ADJ(s), backlog);
}

ssize_t
recv(int s, void *buf, size_t len, int flags)
{

	return recvfrom(s, buf, len, flags, NULL, NULL);
}

ssize_t
recvfrom(int s, void *buf, size_t len, int flags, struct sockaddr *from,
	socklen_t *fromlen)
{
	int (*rc_recvfrom)(int, void *, size_t, int,
	    struct sockaddr *, socklen_t *);

	DPRINTF(("recvfrom\n"));
	assertfd(s);
	rc_recvfrom = rumpcalls[RUMPCALL_RECVFROM];
	return rc_recvfrom(ADJ(s), buf, len, flags, from, fromlen);
}

ssize_t
recvmsg(int s, struct msghdr *msg, int flags)
{
	int (*rc_recvmsg)(int, struct msghdr *, int);

	DPRINTF(("recvmsg\n"));
	assertfd(s);
	rc_recvmsg = rumpcalls[RUMPCALL_RECVMSG];
	return rc_recvmsg(ADJ(s), msg, flags);
}

ssize_t
send(int s, const void *buf, size_t len, int flags)
{

	return sendto(s, buf, len, flags, NULL, 0);
}

ssize_t
sendto(int s, const void *buf, size_t len, int flags,
	const struct sockaddr *to, socklen_t tolen)
{
	int (*rc_sendto)(int, const void *, size_t, int,
	    const struct sockaddr *, socklen_t);

	if (s == -1)
		return len;

	DPRINTF(("sendto\n"));
	assertfd(s);
	rc_sendto = rumpcalls[RUMPCALL_SENDTO];
	return rc_sendto(ADJ(s), buf, len, flags, to, tolen);
}

ssize_t
sendmsg(int s, const struct msghdr *msg, int flags)
{
	int (*rc_sendmsg)(int, const struct msghdr *, int);

	DPRINTF(("sendmsg\n"));
	assertfd(s);
	rc_sendmsg = rumpcalls[RUMPCALL_SENDTO];
	return rc_sendmsg(ADJ(s), msg, flags);
}

int
getsockopt(int s, int level, int optname, void *optval, socklen_t *optlen)
{
	int (*rc_getsockopt)(int, int, int, void *, socklen_t *);

	DPRINTF(("getsockopt\n"));
	assertfd(s);
	rc_getsockopt = rumpcalls[RUMPCALL_GETSOCKOPT];
	return rc_getsockopt(ADJ(s), level, optname, optval, optlen);
}

int
setsockopt(int s, int level, int optname, const void *optval, socklen_t optlen)
{
	int (*rc_setsockopt)(int, int, int, const void *, socklen_t);

	DPRINTF(("setsockopt\n"));
	assertfd(s);
	rc_setsockopt = rumpcalls[RUMPCALL_SETSOCKOPT];
	return rc_setsockopt(ADJ(s), level, optname, optval, optlen);
}

int
shutdown(int s, int how)
{
	int (*rc_shutdown)(int, int);

	DPRINTF(("shutdown\n"));
	assertfd(s);
	rc_shutdown = rumpcalls[RUMPCALL_SHUTDOWN];
	return rc_shutdown(ADJ(s), how);
}

/*
 * Hybrids
 */

ssize_t
read(int fd, void *buf, size_t len)
{
	int (*op_read)(int, void *, size_t);
	ssize_t n;

	DPRINTF(("read %d\n", fd));
	if (fd < HIJACK_FDOFF) {
		op_read = host_read;
	} else {
		fd = ADJ(fd);
		op_read = rumpcalls[RUMPCALL_READ];
	}

	n = op_read(fd, buf, len);
	return n;
}

ssize_t
readv(int fd, const struct iovec *iov, int iovcnt)
{
	int (*op_readv)(int, const struct iovec *, int);

	if (fd < HIJACK_FDOFF) {
		op_readv = host_readv;
	} else {
		fd = ADJ(fd);
		op_readv = rumpcalls[RUMPCALL_READV];
	}

	DPRINTF(("readv\n"));
	return op_readv(fd, iov, iovcnt);
}

ssize_t
write(int fd, const void *buf, size_t len)
{
	int (*op_write)(int, const void *, size_t);

	if (fd < HIJACK_FDOFF) {
		op_write = host_write;
	} else {
		fd = ADJ(fd);
		op_write = rumpcalls[RUMPCALL_WRITE];
	}

	return op_write(fd, buf, len);
}

ssize_t
writev(int fd, const struct iovec *iov, int iovcnt)
{
	int (*op_writev)(int, const struct iovec *, int);

	if (fd < HIJACK_FDOFF) {
		op_writev = host_writev;
	} else {
		fd = ADJ(fd);
		op_writev = rumpcalls[RUMPCALL_WRITEV];
	}

	return op_writev(fd, iov, iovcnt);
}

int
ioctl(int fd, unsigned long cmd, ...)
{
	int (*op_ioctl)(int, unsigned long cmd, ...);
	va_list ap;
	int rv;

	DPRINTF(("ioctl\n"));
	if (fd < HIJACK_FDOFF) {
		op_ioctl = host_ioctl;
	} else {
		fd = ADJ(fd);
		op_ioctl = rumpcalls[RUMPCALL_IOCTL];
	}

	va_start(ap, cmd);
	rv = op_ioctl(fd, cmd, va_arg(ap, void *));
	va_end(ap);
	return rv;
}

int
fcntl(int fd, int cmd, ...)
{
	int (*op_fcntl)(int, int, ...);
	va_list ap;
	int rv;

	DPRINTF(("fcntl\n"));
	if (fd < HIJACK_FDOFF) {
		op_fcntl = host_fcntl;
	} else {
		fd = ADJ(fd);
		op_fcntl = rumpcalls[RUMPCALL_FCNTL];
	}

	va_start(ap, cmd);
	rv = op_fcntl(fd, cmd, va_arg(ap, void *));
	va_end(ap);
	return rv;
}

int
close(int fd)
{
	int (*op_close)(int);

	DPRINTF(("close %d\n", fd));
	if (fd < HIJACK_FDOFF) {
		op_close = host_close;
	} else {
		fd = ADJ(fd);
		op_close = rumpcalls[RUMPCALL_CLOSE];
	}

	return op_close(fd);
}

/*
 * select() has more than one implication.  e.g. we cannot know
 * the caller's FD_SETSIZE.  So just assume something and hope.
 */
static void
checkset(fd_set *setti, int nfds, int *hostcall, int *rumpcall)
{
	int i;

	if (!setti)
		return;

	for (i = 0; i < MIN(nfds, FD_SETSIZE); i++) {
		if (FD_ISSET(i, setti)) {
			if (i < HIJACK_FDOFF)
				*hostcall = 1;
			else
				*rumpcall = 1;
		}
	}
}

static void
adjustset(fd_set *setti, int nfds, int plus)
{
	int fd, i;

	if (!setti)
		return;

	for (i = 0; i < MIN(nfds, FD_SETSIZE); i++) {
		assert(i < HIJACK_FDOFF || !plus);
		assert(i >= HIJACK_FDOFF || plus);
		if (FD_ISSET(i, setti)) {
			FD_CLR(i, setti);
			if (plus)
				fd = i + HIJACK_FDOFF;
			else
				fd = i - HIJACK_FDOFF;
			FD_SET(fd, setti);
		}
	}
}

int
select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds,
	struct timeval *timeout)
{
	int (*op_select)(int, fd_set *, fd_set *, fd_set *, struct timeval *);
	int hostcall = 0, rumpcall = 0;
	int rv;

	checkset(readfds, nfds, &hostcall, &rumpcall);
	checkset(writefds, nfds, &hostcall, &rumpcall);
	checkset(exceptfds, nfds, &hostcall, &rumpcall);

	if (hostcall && rumpcall) {
		fprintf(stderr, "cannot select() two kernels! (fixme)\n");
		return EINVAL;
	}

	if (hostcall) {
		op_select = host_select;
	} else {
		adjustset(readfds, nfds, 1);
		adjustset(writefds, nfds, 1);
		adjustset(exceptfds, nfds, 1);
		op_select = rumpcalls[RUMPCALL_SELECT];
	}

	DPRINTF(("select\n"));
	rv = op_select(nfds+HIJACK_FDOFF,
	    readfds, writefds, exceptfds, timeout);
	if (rumpcall) {
		adjustset(readfds, nfds, 0);
		adjustset(writefds, nfds, 0);
		adjustset(exceptfds, nfds, 0);
	}
	return rv;
}

static void
checkpoll(struct pollfd *fds, nfds_t nfds, int *hostcall, int *rumpcall)
{
	nfds_t i;

	for (i = 0; i < nfds; i++) {
		if (fds[i].fd < HIJACK_FDOFF)
			(*hostcall)++;
		else
			(*rumpcall)++;
	}
}

static void
adjustpoll(struct pollfd *fds, nfds_t nfds, int plus)
{
	nfds_t i;

	for (i = 0; i < nfds; i++) {
		if (plus)
			fds[i].fd += HIJACK_FDOFF;
		else
			fds[i].fd -= HIJACK_FDOFF;
	}
}

/*
 * poll is easy as long as the call comes in the fds only in one
 * kernel.  otherwise its quite tricky...
 */
struct pollarg {
	struct pollfd *pfds;
	nfds_t nfds;
	int timeout;
	int pipefd;
	int errnum;
};

static void *
hostpoll(void *arg)
{
	struct pollarg *parg = arg;
	intptr_t rv;

	rv = poll(parg->pfds, parg->nfds, parg->timeout);
	if (rv == -1)
		parg->errnum = errno;
	rump_sys_write(parg->pipefd, &rv, sizeof(rv));

	return (void *)(intptr_t)rv;
}

int
poll(struct pollfd *fds, nfds_t nfds, int timeout)
{
	int (*op_poll)(struct pollfd *, nfds_t, int);
	int hostcall = 0, rumpcall = 0;
	pthread_t pt;
	nfds_t i;
	int rv;

	checkpoll(fds, nfds, &hostcall, &rumpcall);

	if (hostcall && rumpcall) {
		struct pollfd *pfd_host = NULL, *pfd_rump = NULL;
		int rpipe[2] = {-1,-1}, hpipe[2] = {-1,-1};
		struct pollarg parg;
		uintptr_t lrv;
		int sverrno = 0, trv;

		/*
		 * ok, this is where it gets tricky.  We must support
		 * this since it's a very common operation in certain
		 * types of software (telnet, netcat, etc).  We allocate
		 * two vectors and run two poll commands in separate
		 * threads.  Whichever returns first "wins" and the
		 * other kernel's fds won't show activity.
		 */
		rv = -1;

		/* allocate full vector for O(n) joining after call */
		pfd_host = malloc(sizeof(*pfd_host)*(nfds+1));
		if (!pfd_host)
			goto out;
		pfd_rump = malloc(sizeof(*pfd_rump)*(nfds+1));
		if (!pfd_rump) {
			goto out;
		}

		/* split vectors */
		for (i = 0; i < nfds; i++) {
			if (fds[i].fd < HIJACK_FDOFF) {
				pfd_host[i].fd = fds[i].fd;
				pfd_host[i].events = fds[i].events;
				pfd_rump[i].fd = -1;
			} else {
				pfd_host[i].fd = -1;
				pfd_rump[i].fd = ADJ(fds[i].fd);
				pfd_rump[i].events = fds[i].events;
			}
		}

		/*
		 * then, open two pipes, one for notifications
		 * to each kernel.
		 */
		if (rump_sys_pipe(rpipe) == -1)
			goto out;
		if (pipe(hpipe) == -1)
			goto out;

		pfd_host[nfds].fd = hpipe[0];
		pfd_host[nfds].events = POLLIN;
		pfd_rump[nfds].fd = rpipe[0];
		pfd_rump[nfds].events = POLLIN;

		/*
		 * then, create a thread to do host part and meanwhile
		 * do rump kernel part right here
		 */

		parg.pfds = pfd_host;
		parg.nfds = nfds+1;
		parg.timeout = timeout;
		parg.pipefd = rpipe[1];
		pthread_create(&pt, NULL, hostpoll, &parg);

		lrv = rump_sys_poll(pfd_rump, nfds+1, timeout);
		sverrno = errno;
		write(hpipe[1], &rv, sizeof(rv));
		pthread_join(pt, (void *)&trv);

		/* check who "won" and merge results */
		if (lrv != 0 && pfd_host[nfds].revents & POLLIN) {
			rv = trv;

			for (i = 0; i < nfds; i++) {
				if (pfd_rump[i].fd != -1)
					fds[i].revents = pfd_rump[i].revents;
			}
			sverrno = parg.errnum;
		} else if (trv != 0 && pfd_rump[nfds].revents & POLLIN) {
			rv = trv;

			for (i = 0; i < nfds; i++) {
				if (pfd_host[i].fd != -1)
					fds[i].revents = pfd_host[i].revents;
			}
		} else {
			rv = 0;
			assert(timeout != -1);
		}

 out:
		if (rpipe[0] != -1)
			rump_sys_close(rpipe[0]);
		if (rpipe[1] != -1)
			rump_sys_close(rpipe[1]);
		if (hpipe[0] != -1)
			close(hpipe[0]);
		if (hpipe[1] != -1)
			close(hpipe[1]);
		free(pfd_host);
		free(pfd_rump);
		errno = sverrno;
	} else {
		if (hostcall) {
			op_poll = host_poll;
		} else {
			op_poll = rumpcalls[RUMPCALL_POLL];
			adjustpoll(fds, nfds, 0);
		}

		rv = op_poll(fds, nfds, timeout);
		if (rumpcall)
			adjustpoll(fds, nfds, 1);
	}

	return rv;
}

int
pollts(struct pollfd *fds, nfds_t nfds, const struct timespec *ts,
	const sigset_t *sigmask)
{

	abort();
}