/*  $NetBSD: ops.c,v 1.23 2010/10/11 05:37:58 manu Exp $ */

/*-
 *  Copyright (c) 2010 Emmanuel Dreyfus. All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 *  ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 *  BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */ 

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <libgen.h>
#include <errno.h>
#include <err.h>
#include <sysexits.h>
#include <syslog.h>
#include <puffs.h>
#include <sys/vnode.h>
#include <sys/socket.h>
#include <machine/vmparam.h>

#include "perfuse_priv.h"
#include "fuse.h"

extern int perfuse_diagflags;

static int xchg_msg(struct puffs_usermount *, puffs_cookie_t, 
    perfuse_msg_t *, size_t, enum perfuse_xchg_pb_reply); 
static int no_access(puffs_cookie_t, const struct puffs_cred *, mode_t);
static void fuse_attr_to_vap(struct perfuse_state *,
    struct vattr *, struct fuse_attr *);
static int node_lookup_dir_nodot(struct puffs_usermount *,
    puffs_cookie_t, char *, size_t, struct puffs_node **);
static int node_lookup_common(struct puffs_usermount *, puffs_cookie_t, 
    const char*, struct puffs_node **);
static int node_mk_common(struct puffs_usermount *, puffs_cookie_t,
    struct puffs_newinfo *, const struct puffs_cn *pcn, perfuse_msg_t *);
static ssize_t fuse_to_dirent(struct puffs_usermount *, puffs_cookie_t,
    struct fuse_dirent *, size_t);
static int readdir_buffered(puffs_cookie_t, struct dirent *, off_t *, 
    size_t *, const struct puffs_cred *, int *, off_t *, size_t *);
static void requeue_request(struct puffs_usermount *, 
    puffs_cookie_t opc, enum perfuse_qtype);
static int dequeue_requests(struct perfuse_state *, 
    puffs_cookie_t opc, enum perfuse_qtype, int);
#define DEQUEUE_ALL 0

/* 
 *  From <sys/vnode>, inside #ifdef _KERNEL section
 */
#define IO_SYNC		(0x40|IO_DSYNC) 
#define IO_DSYNC	0x00200  
#define IO_DIRECT	0x02000

/* 
 *  From <fcntl>, inside #ifdef _KERNEL section
 */
#define F_WAIT		0x010
#define F_FLOCK		0x020
#define OFLAGS(fflags)  ((fflags) - 1)

/* 
 * Borrowed from src/sys/kern/vfs_subr.c and src/sys/sys/vnode.h 
 */
const enum vtype iftovt_tab[16] = { 
	VNON, VFIFO, VCHR, VNON, VDIR, VNON, VBLK, VNON,
        VREG, VNON, VLNK, VNON, VSOCK, VNON, VNON, VBAD,
};
const int vttoif_tab[9] = { 
	0, S_IFREG, S_IFDIR, S_IFBLK, S_IFCHR, S_IFLNK,
        S_IFSOCK, S_IFIFO, S_IFMT,
}; 

#define IFTOVT(mode) (iftovt_tab[((mode) & S_IFMT) >> 12])
#define VTTOIF(indx) (vttoif_tab[(int)(indx)])

int
perfuse_node_close_common(pu, opc, mode)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	int mode;
{
	struct perfuse_state *ps;
	perfuse_msg_t *pm;
	int op;
	uint64_t fh;
	struct fuse_release_in *fri;
	struct perfuse_node_data *pnd;
	struct puffs_node *pn;
	int error;

	ps = puffs_getspecific(pu);
	pn = (struct puffs_node *)opc;
	pnd = PERFUSE_NODE_DATA(pn);

	if (puffs_pn_getvap(pn)->va_type == VDIR) {
		op = FUSE_RELEASEDIR;
		mode = FREAD;
	} else {
		op = FUSE_RELEASE;
	}

	/*
	 * Destroy the filehandle before sending the 
	 * request to the FUSE filesystem, otherwise 
	 * we may get a second close() while we wait
	 * for the reply, and we would end up closing
	 * the same fh twice instead of closng both.
	 */
	fh = perfuse_get_fh(opc, mode);
	perfuse_destroy_fh(pn, fh);

	/*
	 * release_flags may be set to FUSE_RELEASE_FLUSH
	 * to flush locks. lock_owner must be set in that case
	 */
	pm = ps->ps_new_msg(pu, opc, op, sizeof(*fri), NULL);
	fri = GET_INPAYLOAD(ps, pm, fuse_release_in);
	fri->fh = fh;
	fri->flags = 0;
	fri->release_flags = 0;
	fri->lock_owner = pnd->pnd_lock_owner;
	fri->flags = (fri->lock_owner != 0) ? FUSE_RELEASE_FLUSH : 0;

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_FH)
		DPRINTF("%s: opc = %p, ino = %"PRId64", fh = 0x%"PRIx64"\n",
			 __func__, (void *)opc, pnd->pnd_ino, fri->fh);
#endif

	if ((error = xchg_msg(pu, opc, pm,
			      NO_PAYLOAD_REPLY_LEN, wait_reply)) != 0)
		goto out;

	ps->ps_destroy_msg(pm);

	error = 0;

out:
	if (error != 0)
		DERRX(EX_SOFTWARE, "%s: freed fh = 0x%"PRIx64" but filesystem "
		      "returned error = %d", __func__, fh, error);

	return error;
}

/* ARGSUSED1 */
static int
xchg_msg(pu, opc, pm, len, wait)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	perfuse_msg_t *pm;
	size_t len;
     	enum perfuse_xchg_pb_reply wait;
{
	struct perfuse_state *ps;
	struct perfuse_node_data *pnd;
	int error;

	ps = puffs_getspecific(pu);
	pnd = NULL;
	if ((struct puffs_node *)opc != NULL)
		pnd = PERFUSE_NODE_DATA(opc);

#ifdef PERFUSE_DEBUG
	if ((perfuse_diagflags & PDF_FILENAME) && (opc != 0))
		DPRINTF("file = \"%s\" flags = 0x%x\n", 
			perfuse_node_path(opc),
			PERFUSE_NODE_DATA(opc)->pnd_flags);
#endif
	if (pnd)
		pnd->pnd_flags |= PND_INXCHG;

	error = ps->ps_xchg_msg(pu, pm, len, wait);

	if (pnd) {
		pnd->pnd_flags &= ~PND_INXCHG;
		(void)dequeue_requests(ps, opc, PCQ_AFTERXCHG, DEQUEUE_ALL);
	}

	return error;
}

static int
no_access(opc, pcr, mode)
	puffs_cookie_t opc;
	const struct puffs_cred *pcr;
	mode_t mode;
{
	struct puffs_node *pn;
	struct vattr *va;

	/*
	 * pcr is NULL for self open through fsync or readdir.
	 * In both case, access control is useless, as it was
	 * done before, at open time.
	 */
	if (pcr == NULL)
		return 0;

	/*
	 * pcr is NULL for self open through fsync or readdir.
	 * In both case, access control is useless, as it was
	 * done before, at open time.
	 */
	if (pcr == NULL)
		return 0;

	pn = (struct puffs_node *)opc;
	va = puffs_pn_getvap(pn);
	return puffs_access(va->va_type, va->va_mode, 
			    va->va_uid, va->va_gid,
			    mode, pcr);
}

static void
fuse_attr_to_vap(ps, vap, fa)
	struct perfuse_state *ps;
	struct vattr *vap;
	struct fuse_attr *fa;
{
	vap->va_type = IFTOVT(fa->mode);
	vap->va_mode = fa->mode;
	vap->va_nlink = fa->nlink;
	vap->va_uid = fa->uid;
	vap->va_gid = fa->gid;
	vap->va_fsid = ps->ps_fsid;
	vap->va_fileid = fa->ino;
	vap->va_size = fa->size;
	vap->va_blocksize = fa->blksize;
	vap->va_atime.tv_sec = (time_t)fa->atime;
	vap->va_atime.tv_nsec = (long) fa->atimensec;
	vap->va_mtime.tv_sec = (time_t)fa->mtime;
	vap->va_mtime.tv_nsec = (long)fa->mtimensec;
	vap->va_ctime.tv_sec = (time_t)fa->ctime;
	vap->va_ctime.tv_nsec = (long)fa->ctimensec;
	vap->va_birthtime.tv_sec = 0;
	vap->va_birthtime.tv_nsec = 0;
	vap->va_gen = 0; 
	vap->va_flags = 0;
	vap->va_rdev = fa->rdev;
	vap->va_bytes = fa->size;
	vap->va_filerev = (u_quad_t)PUFFS_VNOVAL;
	vap->va_vaflags = 0;

	if (vap->va_blocksize == 0)
		vap->va_blocksize = DEV_BSIZE;

	if (vap->va_size == (size_t)PUFFS_VNOVAL) /* XXX */
		vap->va_size = 0;

	return;
}


/* 
 * Lookup name in directory opc
 * We take special care of name being . or ..
 * These are returned by readdir and deserve tweaks.
 */
static int
node_lookup_dir_nodot(pu, opc, name, namelen, pnp)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	char *name;
	size_t namelen;
	struct puffs_node **pnp;
{
	char *path;
	struct puffs_node *dpn = (struct puffs_node *)opc;
	int error;

	/*
	 *  is easy as we already know it
	 */
	if (strncmp(name, ".", namelen) == 0) {
		*pnp = (struct puffs_node *)opc;
		return 0;
	}

	/*
	 * For .. we just forget the name part
	 */
	if (strncmp(name, "..", namelen) == 0)
		namelen = 0;

	namelen = PNPLEN(dpn) + 1 + namelen + 1;
	if ((path = malloc(namelen)) == NULL)
		DERR(EX_OSERR, "malloc failed");
	(void)snprintf(path, namelen, "%s/%s", 
		       perfuse_node_path((puffs_cookie_t)dpn), name);

	error = node_lookup_common(pu, opc, path, pnp);
	
	free(path);

	return error;
}

static int
node_lookup_common(pu, opc, path, pnp)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	const char *path;
	struct puffs_node **pnp;
{
	struct perfuse_state *ps;
	struct perfuse_node_data *pnd;
	perfuse_msg_t *pm;
	struct fuse_entry_out *feo;
	struct puffs_node *pn;
	size_t len;
	int error;

	ps = puffs_getspecific(pu);

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_FILENAME)
		DPRINTF("%s: opc = %p, file = \"%s\" looking up \"%s\"\n",
			__func__, (void *)opc, perfuse_node_path(opc), path);
#endif
	/*
	 * Is the node already known?
	 */
	TAILQ_FOREACH(pnd, &PERFUSE_NODE_DATA(opc)->pnd_children, pnd_next) {
		if ((pnd->pnd_flags & PND_REMOVED) ||
		    (strcmp(pnd->pnd_name, path) != 0))
			continue;

		/*
		 * We have a match
		 */
		if (pnp != NULL)
			*pnp = (struct puffs_node *)(pnd->pnd_pn);

#ifdef PERFUSE_DEBUG
		if (perfuse_diagflags & PDF_FILENAME)
			DPRINTF("%s: opc = %p, file = \"%s\" found "
				"cookie = %p, ino = %"PRId64" for \"%s\"\n",
				__func__, (void *)opc, perfuse_node_path(opc), 
				(void *)pnd->pnd_pn, pnd->pnd_ino, path);
#endif
		return 0;
	}

	len = strlen(path) + 1;

	pm = ps->ps_new_msg(pu, opc, FUSE_LOOKUP, len, NULL);
	(void)strlcpy(_GET_INPAYLOAD(ps, pm, char *), path, len);

	if ((error = xchg_msg(pu, opc, pm, sizeof(*feo), wait_reply)) != 0)
		goto out;

	feo = GET_OUTPAYLOAD(ps, pm, fuse_entry_out);

	pn = perfuse_new_pn(pu, path, opc);
	PERFUSE_NODE_DATA(pn)->pnd_ino = feo->nodeid;

	fuse_attr_to_vap(ps, &pn->pn_va, &feo->attr);
	pn->pn_va.va_gen = (u_long)(feo->generation);

	if (pnp != NULL)
		*pnp = pn;

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_FILENAME)
		DPRINTF("%s: opc = %p, looked up opc = %p, ino = %"PRId64" "
			"file = \"%s\"\n", __func__, (void *)opc, pn, 
			feo->nodeid, path);
#endif
out: 
	ps->ps_destroy_msg(pm);

	return error;
}


/*
 * Common final code for methods that create objects:
 * perfuse_node_mkdir
 * perfuse_node_mknod
 * perfuse_node_symlink
 */
static int
node_mk_common(pu, opc, pni, pcn, pm)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	struct puffs_newinfo *pni;
	const struct puffs_cn *pcn;
	perfuse_msg_t *pm;
{
	struct perfuse_state *ps;
	struct puffs_node *pn;
	struct fuse_entry_out *feo;
	struct fuse_setattr_in *fsi;
	int error;

	ps =  puffs_getspecific(pu);

	if ((error = xchg_msg(pu, opc, pm, sizeof(*feo), wait_reply)) != 0)
		goto out;

	feo = GET_OUTPAYLOAD(ps, pm, fuse_entry_out);
	if (feo->nodeid == PERFUSE_UNKNOWN_INO)
		DERRX(EX_SOFTWARE, "%s: no ino", __func__);

	pn = perfuse_new_pn(pu, pcn->pcn_name, opc);
	PERFUSE_NODE_DATA(pn)->pnd_ino = feo->nodeid;

	fuse_attr_to_vap(ps, &pn->pn_va, &feo->attr);
	pn->pn_va.va_gen = (u_long)(feo->generation);

	puffs_newinfo_setcookie(pni, pn);

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_FILENAME)
		DPRINTF("%s: opc = %p, file = \"%s\", flags = 0x%x "
			"ino = %"PRId64"\n",
			__func__, (void *)pn, pcn->pcn_name,
			PERFUSE_NODE_DATA(pn)->pnd_flags, feo->nodeid);
#endif
	ps->ps_destroy_msg(pm);

	/* 
	 * Set owner and group
	 */
	(void)puffs_cred_getuid(pcn->pcn_cred, &pn->pn_va.va_uid);
	(void)puffs_cred_getgid(pcn->pcn_cred, &pn->pn_va.va_gid);

	pm = ps->ps_new_msg(pu, (puffs_cookie_t)pn, 
			    FUSE_SETATTR, sizeof(*fsi), NULL);
	fsi = GET_INPAYLOAD(ps, pm, fuse_setattr_in);
	fsi->uid = pn->pn_va.va_uid;
	fsi->gid = pn->pn_va.va_gid;
	fsi->valid = FUSE_FATTR_UID|FUSE_FATTR_GID;

	/*
	 * A fuse_attr_out is returned, but we ignore it.
	 */
	error = xchg_msg(pu, (puffs_cookie_t)pn, 
			 pm, sizeof(struct fuse_attr_out), wait_reply);

	/*
	 * The parent directory needs a sync
	 */
	PERFUSE_NODE_DATA(opc)->pnd_flags |= PND_DIRTY;

out:
	ps->ps_destroy_msg(pm);

	return error;
}

static ssize_t
fuse_to_dirent(pu, opc, fd, fd_len)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	struct fuse_dirent *fd;
	size_t fd_len;
{
	struct dirent *dents;
	size_t dents_len;
	ssize_t written;
	uint64_t fd_offset;
	struct fuse_dirent *fd_base;
	size_t len;

	fd_base = fd;
	fd_offset = 0;
	written = 0;
	dents = PERFUSE_NODE_DATA(opc)->pnd_dirent;
	dents_len = (size_t)PERFUSE_NODE_DATA(opc)->pnd_dirent_len;

	do {
		char *ndp;
		size_t reclen;

		reclen = _DIRENT_RECLEN(dents, fd->namelen);

		/*
		 * Check we do not overflow the output buffer
		 * struct fuse_dirent is bigger than struct dirent,
		 * so we should always use fd_len and never reallocate
		 * later. 
		 * If we have to reallocate,try to double the buffer
		 * each time so that we do not have to do it too often.
		 */
		if (written + reclen > dents_len) {
			if (dents_len == 0)
				dents_len = fd_len;
			else
				dents_len = 
				   MAX(2 * dents_len, written + reclen);
	
			dents = PERFUSE_NODE_DATA(opc)->pnd_dirent;
			if ((dents = realloc(dents, dents_len)) == NULL)
				DERR(EX_OSERR, "malloc failed");

			PERFUSE_NODE_DATA(opc)->pnd_dirent = dents;
			PERFUSE_NODE_DATA(opc)->pnd_dirent_len = dents_len;

			/*
			 * (void *) for delint
			 */
			ndp = (char *)(void *)dents + written;
			dents = (struct dirent *)(void *)ndp;
		}
		


		/*
		 * Filesystem was mounted without -o use_ino
		 * Perform a lookup to find it.
		 * XXX still broken
		 */
		if (fd->ino == PERFUSE_UNKNOWN_INO) {
			struct puffs_node *pn;

			if (node_lookup_dir_nodot(pu, opc, fd->name, 
						  fd->namelen, &pn) != 0) 
				DERRX(EX_SOFTWARE, 
				     "node_lookup_dir_nodot failed");

			fd->ino = PERFUSE_NODE_DATA(pn)->pnd_ino;
		}

		dents->d_fileno = fd->ino;
		dents->d_reclen = (unsigned short)reclen;
		dents->d_namlen = fd->namelen;
		dents->d_type = fd->type;
		strlcpy(dents->d_name, fd->name, fd->namelen + 1);

#ifdef PERFUSE_DEBUG
		if (perfuse_diagflags & PDF_READDIR)
			DPRINTF("%s: translated \"%s\" ino = %"PRId64"\n",
				__func__, dents->d_name, dents->d_fileno);
#endif

		dents = _DIRENT_NEXT(dents);
		written += reclen;

		/*
		 * Move to the next record.
		 * fd->off seems unreliable, for instance, flusterfs 
		 * does not clear the unused bits, and we get 
		 * 0xffffffffb9b95040 instead of just 0x40. Use 
		 * record alignement instead.
		 */
		len = FUSE_DIRENT_ALIGN(sizeof(*fd) + fd->namelen);
#ifdef PERFUSE_DEBUG
		if (perfuse_diagflags & PDF_READDIR)
			DPRINTF("%s: record at %"PRId64"/0x%"PRIx64" "
				"length = %zd/0x%zx. "
				"next record at %"PRId64"/0x%"PRIx64" "
				"max %zd/0x%zx\n",
				__func__, fd_offset, fd_offset, len, len,
				fd_offset + len, fd_offset + len, 
				fd_len, fd_len);
#endif
		fd_offset += len;

		/*
		 * Check if next record is still within the packet
		 * If it is not, we reached the end of the buffer.
		 */
		if (fd_offset >= fd_len)
			break;

		/*
		 * (void *) for delint
		 */
		ndp = (char *)(void *)fd_base + (size_t)fd_offset;
		fd = (struct fuse_dirent *)(void *)ndp;

	} while (1 /* CONSTCOND */);

	/*
	 * Adjust the dirent output length 
	 */
	if (written != -1)
		PERFUSE_NODE_DATA(opc)->pnd_dirent_len = written;
	
	return written;
}

/* ARGSUSED4 */
static int 
readdir_buffered(opc, dent, readoff, 
		 reslen, pcr, eofflag, cookies, ncookies)
	puffs_cookie_t opc;
	struct dirent *dent;
	off_t *readoff;
	size_t *reslen;
	const struct puffs_cred *pcr;
	int *eofflag;
	off_t *cookies;
	size_t *ncookies;
{
	struct dirent *fromdent;
	struct perfuse_node_data *pnd;
	char *ndp;
	
	pnd = PERFUSE_NODE_DATA(opc);

	while (*readoff < pnd->pnd_dirent_len) {
		/*
		 * (void *) for delint
		 */
		ndp = (char *)(void *)pnd->pnd_dirent + (size_t)*readoff;
		fromdent = (struct dirent *)(void *)ndp;

		if (*reslen < _DIRENT_SIZE(fromdent))
			break;

		memcpy(dent, fromdent, _DIRENT_SIZE(fromdent));
		*readoff += _DIRENT_SIZE(fromdent);
		*reslen -= _DIRENT_SIZE(fromdent);
	
		dent = _DIRENT_NEXT(dent);
	}

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_READDIR)
		DPRINTF("%s: readoff = %"PRId64",  "
			"pnd->pnd_dirent_len = %"PRId64"\n",
			__func__, *readoff, pnd->pnd_dirent_len);
#endif
	if (*readoff >=  pnd->pnd_dirent_len) {
		free(pnd->pnd_dirent);
		pnd->pnd_dirent = NULL;
		pnd->pnd_dirent_len = 0;
		*eofflag = 1;
	}

	return 0;
}

static void
requeue_request(pu, opc, type)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	enum perfuse_qtype type;
{
	struct perfuse_cc_queue pcq;
	struct perfuse_node_data *pnd;
#ifdef PERFUSE_DEBUG
	struct perfuse_state *ps;

	ps = perfuse_getspecific(pu);
#endif

	pnd = PERFUSE_NODE_DATA(opc);
	pcq.pcq_type = type;
	pcq.pcq_cc = puffs_cc_getcc(pu);
	TAILQ_INSERT_TAIL(&pnd->pnd_pcq, &pcq, pcq_next);

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_REQUEUE)
		DPRINTF("%s: REQUEUE opc = %p, pcc = %p (%s)\n", 
		        __func__, (void *)opc, pcq.pcq_cc,
			perfuse_qtypestr[type]);
#endif

	puffs_cc_yield(pcq.pcq_cc);
	TAILQ_REMOVE(&pnd->pnd_pcq, &pcq, pcq_next);

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_REQUEUE)
		DPRINTF("%s: RESUME opc = %p, pcc = %p (%s)\n",
		        __func__, (void *)opc, pcq.pcq_cc,
			perfuse_qtypestr[type]);
#endif

	return;
}

/* ARGSUSED0 */
static int
dequeue_requests(ps, opc, type, max)
	struct perfuse_state *ps;
	puffs_cookie_t opc;
	enum perfuse_qtype type;
	int max;
{
	struct perfuse_cc_queue *pcq;
	struct perfuse_node_data *pnd;
	int dequeued;

	pnd = PERFUSE_NODE_DATA(opc);
	dequeued = 0;
	TAILQ_FOREACH(pcq, &pnd->pnd_pcq, pcq_next) {
		if (pcq->pcq_type != type)
			continue;
	
#ifdef PERFUSE_DEBUG
		if (perfuse_diagflags & PDF_REQUEUE)
			DPRINTF("%s: SCHEDULE opc = %p, pcc = %p (%s)\n",
				__func__, (void *)opc, pcq->pcq_cc,
				 perfuse_qtypestr[type]);
#endif
		puffs_cc_schedule(pcq->pcq_cc);
		
		if (++dequeued == max)
			break;
	}

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_REQUEUE)
		DPRINTF("%s: DONE  opc = %p\n", __func__, (void *)opc);
#endif

	return dequeued;
}
	
void
perfuse_fs_init(pu)
	struct puffs_usermount *pu;
{
	struct perfuse_state *ps;
	perfuse_msg_t *pm;
	struct fuse_init_in *fii;
	struct fuse_init_out *fio;
	int error;

	ps = puffs_getspecific(pu);
	
        if (puffs_mount(pu, ps->ps_target, ps->ps_mountflags, ps->ps_root) != 0)
                DERR(EX_OSERR, "puffs_mount failed");

	/*
	 * Linux 2.6.34.1 sends theses flags:
	 * FUSE_ASYNC_READ | FUSE_POSIX_LOCKS | FUSE_ATOMIC_O_TRUNC
	 * FUSE_EXPORT_SUPPORT | FUSE_BIG_WRITES | FUSE_DONT_MASK
	 *
	 * Linux also sets max_readahead at 32 pages (128 kB)
	 */
	pm = ps->ps_new_msg(pu, 0, FUSE_INIT, sizeof(*fii), NULL);
	fii = GET_INPAYLOAD(ps, pm, fuse_init_in);
	fii->major = FUSE_KERNEL_VERSION;
	fii->minor = FUSE_KERNEL_MINOR_VERSION;
	fii->max_readahead = 32 * PAGE_SIZE; 
	fii->flags = (FUSE_ASYNC_READ|FUSE_POSIX_LOCKS|FUSE_ATOMIC_O_TRUNC);

	if ((error = xchg_msg(pu, 0, pm, sizeof(*fio), wait_reply)) != 0)
		DERRX(EX_SOFTWARE, "init message exchange failed (%d)", error);

	fio = GET_OUTPAYLOAD(ps, pm, fuse_init_out);
	ps->ps_max_readahead = fio->max_readahead;
	ps->ps_max_write = fio->max_write;

	ps->ps_destroy_msg(pm);

	return;
}

int
perfuse_fs_unmount(pu, flags)
	struct puffs_usermount *pu;
	int flags;
{
	perfuse_msg_t *pm;
	struct perfuse_state *ps;
	puffs_cookie_t opc;
	int error;

	ps = puffs_getspecific(pu);

	opc = (puffs_cookie_t)puffs_getroot(pu);
	pm = ps->ps_new_msg(pu, opc, FUSE_DESTROY, 0, NULL);

	if ((error = xchg_msg(pu, opc, pm, UNSPEC_REPLY_LEN, wait_reply)) != 0){
		DWARN("unmount %s", ps->ps_target);
		if (!(flags & MNT_FORCE))
			goto out;
	}

	DPRINTF("%s unmounted, exit\n", ps->ps_target);

	exit(0);
out:
	ps->ps_destroy_msg(pm);
	
	return error;
}

int
perfuse_fs_statvfs(pu, svfsb)
	struct puffs_usermount *pu;
	struct statvfs *svfsb;
{
	struct perfuse_state *ps;
	perfuse_msg_t *pm;
	puffs_cookie_t opc;
	struct fuse_statfs_out *fso;
	int error;

	ps = puffs_getspecific(pu);
	opc = (puffs_cookie_t)puffs_getroot(pu);
	pm = ps->ps_new_msg(pu, opc, FUSE_STATFS, 0, NULL);

	if ((error = xchg_msg(pu, opc, pm, sizeof(*fso), wait_reply)) != 0)
		goto out;

	fso = GET_OUTPAYLOAD(ps, pm, fuse_statfs_out);
	svfsb->f_flag = ps->ps_mountflags;
	svfsb->f_bsize = fso->st.bsize;
	svfsb->f_frsize = fso->st.frsize;
	svfsb->f_iosize = ((struct puffs_node *)opc)->pn_va.va_blocksize;
	svfsb->f_blocks = fso->st.blocks;
	svfsb->f_bfree = fso->st.bfree;
	svfsb->f_bavail = fso->st.bavail;
	svfsb->f_bresvd = fso->st.bfree - fso->st.bavail;
	svfsb->f_files = fso->st.files;
	svfsb->f_ffree = fso->st.ffree;
	svfsb->f_favail = fso->st.ffree;/* files not reserved for root */
	svfsb->f_fresvd = 0;		/* files reserved for root */ 

	svfsb->f_syncreads = ps->ps_syncreads;
	svfsb->f_syncwrites = ps->ps_syncwrites;

	svfsb->f_asyncreads = ps->ps_asyncreads;
	svfsb->f_asyncwrites = ps->ps_asyncwrites;

	svfsb->f_fsidx.__fsid_val[0] = (int32_t)ps->ps_fsid;
	svfsb->f_fsidx.__fsid_val[1] = 0;
	svfsb->f_fsid = ps->ps_fsid;
	svfsb->f_namemax = MAXPATHLEN;	/* XXX */
	svfsb->f_owner = ps->ps_owner_uid;

	(void)strlcpy(svfsb->f_mntonname, ps->ps_target, _VFS_NAMELEN);

	if (ps->ps_filesystemtype != NULL)
		(void)strlcpy(svfsb->f_fstypename, 
			      ps->ps_filesystemtype, _VFS_NAMELEN);
	else
		(void)strlcpy(svfsb->f_fstypename, "fuse", _VFS_NAMELEN);

	if (ps->ps_source != NULL)
		strlcpy(svfsb->f_mntfromname, ps->ps_source, _VFS_NAMELEN);
	else
		strlcpy(svfsb->f_mntfromname, _PATH_FUSE, _VFS_NAMELEN);
out:
	ps->ps_destroy_msg(pm);
	
	return error;
}

int
perfuse_fs_sync(pu, waitfor, pcr)
	struct puffs_usermount *pu;
	int waitfor;
	const struct puffs_cred *pcr;
{
	/* 
	 * FUSE does not seem to have a FS sync callback.
	 * Maybe do not even register this callback
	 */
	return puffs_fsnop_sync(pu, waitfor, pcr);
}

/* ARGSUSED0 */
int
perfuse_fs_fhtonode(pu, fid, fidsize, pni)
	struct puffs_usermount *pu;
	void *fid;
	size_t fidsize;
	struct puffs_newinfo *pni;
{
	DERRX(EX_SOFTWARE, "%s: UNIMPLEMENTED (FATAL)", __func__);
	return 0;
}

/* ARGSUSED0 */
int 
perfuse_fs_nodetofh(pu, cookie, fid, fidsize)
	struct puffs_usermount *pu;
	puffs_cookie_t cookie;
	void *fid;
	size_t *fidsize;
{
	DERRX(EX_SOFTWARE, "%s: UNIMPLEMENTED (FATAL)", __func__);
	return 0;
}

#if 0
/* ARGSUSED0 */
void 
perfuse_fs_extattrctl(pu, cmd, cookie, flags, namespace, attrname)
	struct puffs_usermount *pu;
	int cmd,
	puffs_cookie_t *cookie;
	int flags;
	int namespace;
	const char *attrname;
{
	DERRX(EX_SOFTWARE, "%s: UNIMPLEMENTED (FATAL)", __func__);
	return 0;
}
#endif /* 0 */

/* ARGSUSED0 */
void
perfuse_fs_suspend(pu, status)
	struct puffs_usermount *pu;
	int status;
{
	return;
}



int
perfuse_node_lookup(pu, opc, pni, pcn)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	struct puffs_newinfo *pni;
	const struct puffs_cn *pcn;
{
	struct puffs_node *pn;
	int error;

	error = 0;

	/*
	 * Special case for ..
	 */
	if (strcmp(pcn->pcn_name, "..") == 0) 
		pn = PERFUSE_NODE_DATA(opc)->pnd_parent;
	else
		error = node_lookup_common(pu, (puffs_cookie_t)opc,
					   pcn->pcn_name, &pn);

	if (error != 0)
		return error;

	if (PERFUSE_NODE_DATA(pn)->pnd_flags & PND_REMOVED)
		return ENOENT;

	/*
	 * If that node had a pending reclaim, wipe it out.
	 */
	PERFUSE_NODE_DATA(pn)->pnd_flags &= ~PND_RECLAIMED;

	puffs_newinfo_setcookie(pni, pn);
	puffs_newinfo_setvtype(pni, pn->pn_va.va_type); 
	puffs_newinfo_setsize(pni, (voff_t)pn->pn_va.va_size);
	puffs_newinfo_setrdev(pni, pn->pn_va.va_rdev);

	return error;
}

int
perfuse_node_create(pu, opc, pni, pcn, vap)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	struct puffs_newinfo *pni;
	const struct puffs_cn *pcn;
	const struct vattr *vap;
{
	perfuse_msg_t *pm;
	struct perfuse_state *ps;
	struct fuse_create_in *fci;
	struct fuse_entry_out *feo;
	struct fuse_open_out *foo;
	struct puffs_node *pn;
	const char *name;
	size_t namelen;
	size_t len;
	int error;
	
	if (PERFUSE_NODE_DATA(opc)->pnd_flags & PND_REMOVED)
		return ENOENT;

#if notyet
	/*
	 * XXX create needs -WX on the parent directory. No pcr is
 	 * given here, we cannot enforce this.
	 */
	if (no_access(opc, pcr, PUFFS_VWRITE|PUFFS_VEXEC))
		return EACCES;
#endif

	/*
	 * If create is unimplemented: Check that it does not
	 * already exists, and if not, do mknod and open
	 */
	ps = puffs_getspecific(pu);
	if (ps->ps_flags & PS_NO_CREAT) {
		error = node_lookup_common(pu, opc, pcn->pcn_name, &pn);
		if (error == 0)	
			return EEXIST;

		error = perfuse_node_mknod(pu, opc, pni, pcn, vap);
		if (error != 0)
			return error;

		error = node_lookup_common(pu, opc, pcn->pcn_name, &pn);
		if (error != 0)	
			return error;

		/*
		 * FUSE does the open at create time, while
		 * NetBSD will open in a subsequent operation.
		 * We need to open now, in order to retain FUSE
		 * semantics, but we have no credentials to use
		 * since the PUFFS interface gives no pcr here. 
		 *
		 * open with NULL pcr will skip permission checks.
		 * There is no security hole, since we know we
		 * can open this file: we just created it. The 
		 * calling process will not get a file descriptor
		 * before the kernel sends the open operation, 
		 * which will have a pcr, anyway.
		 */
		opc = (puffs_cookie_t)pn;
		error = perfuse_node_open(pu, opc, FWRITE, NULL);
		if (error != 0)	
			return error;

		return 0;
	}

	name = pcn->pcn_name;
	namelen = pcn->pcn_namelen + 1;
	len = sizeof(*fci) + namelen;

	/*
	 * flags should use O_WRONLY instead of O_RDWR, but it
	 * breaks when the caller tries to read from file.
	 * 
	 * mode must contain file type (ie: S_IFREG), use VTTOIF(vap->va_type)
	 */
	pm = ps->ps_new_msg(pu, opc, FUSE_CREATE, len, NULL);
	fci = GET_INPAYLOAD(ps, pm, fuse_create_in);
	fci->flags = O_CREAT | O_TRUNC | O_RDWR;
	fci->mode = vap->va_mode | VTTOIF(vap->va_type);
	fci->umask = 0; 	/* Seems unused by libfuse */
	(void)strlcpy((char*)(void *)(fci + 1), name, namelen);

	len = sizeof(*feo) + sizeof(*foo);
	if ((error = xchg_msg(pu, opc, pm, len, wait_reply)) != 0)
		goto out;

	feo = GET_OUTPAYLOAD(ps, pm, fuse_entry_out);
	foo = (struct fuse_open_out *)(void *)(feo + 1);
	if (feo->nodeid == PERFUSE_UNKNOWN_INO)
		DERRX(EX_SOFTWARE, "%s: no ino", __func__);
	
	/*
	 * Save the file handle and inode in node private data 
	 * so that we can reuse it later
	 */
	pn = perfuse_new_pn(pu, name, opc);
	perfuse_new_fh((puffs_cookie_t)pn, foo->fh, FWRITE);
	PERFUSE_NODE_DATA(pn)->pnd_ino = feo->nodeid;

	fuse_attr_to_vap(ps, &pn->pn_va, &feo->attr);
	pn->pn_va.va_gen = (u_long)(feo->generation);
		
	puffs_newinfo_setcookie(pni, pn);

	/*
	 * The parent directory needs a sync
	 */
	PERFUSE_NODE_DATA(opc)->pnd_flags |= PND_DIRTY;

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & (PDF_FH|PDF_FILENAME))
		DPRINTF("%s: opc = %p, file = \"%s\", flags = 0x%x "
			"ino = %"PRId64", wfh = 0x%"PRIx64"\n",
			__func__, (void *)pn, pcn->pcn_name,
			PERFUSE_NODE_DATA(pn)->pnd_flags, feo->nodeid, foo->fh);
#endif

out: 
	ps->ps_destroy_msg(pm);

	/*
	 * create is unimplmented, remember it for later,
	 * and start over using mknod and open instead.
	 */
	if (error == ENOSYS) {
		ps->ps_flags |= PS_NO_CREAT;
		return perfuse_node_create(pu, opc, pni, pcn, vap);
	}

	return error;
}


int
perfuse_node_mknod(pu, opc, pni, pcn, vap)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	struct puffs_newinfo *pni;
	const struct puffs_cn *pcn;
	const struct vattr *vap;
{
	struct perfuse_state *ps;
	perfuse_msg_t *pm;
	struct fuse_mknod_in *fmi;
	const char* path;
	size_t len;
	
	if (PERFUSE_NODE_DATA(opc)->pnd_flags & PND_REMOVED)
		return ENOENT;

	/*
	 * Only superuser can mknod objects other than 
	 * directories, files, socks, fifo and links.
	 *
	 * Create an object require -WX permission in the parent directory
	 *
	 * XXX The PUFFS interface gives us no pcr here, we cannot perfom 
	 * access control.
	 */
	switch (vap->va_type) {
	case VDIR:	/* FALLTHROUGH */
	case VREG:	/* FALLTHROUGH */
	case VFIFO:	/* FALLTHROUGH */
	case VSOCK:	/* FALLTHROUGH */
#if notyet
		if (no_access(opc, pcr, PUFFS_VWRITE|PUFFS_VEXEC))
			return EACCES;
#endif
		break;
	default:	/* VNON, VBLK, VCHR, VBAD */
#if notyet
		if (!puffs_cred_isjuggernaut(pcr))
			return EACCES;
#endif
		break;
	}


	ps = puffs_getspecific(pu);
	path = pcn->pcn_name;
	len = sizeof(*fmi) + pcn->pcn_namelen + 1; 

	/*	
	 * mode must contain file type (ie: S_IFREG), use VTTOIF(vap->va_type)
	 */
	pm = ps->ps_new_msg(pu, opc, FUSE_MKNOD, len, NULL);
	fmi = GET_INPAYLOAD(ps, pm, fuse_mknod_in);
	fmi->mode = vap->va_mode | VTTOIF(vap->va_type);
	fmi->rdev = (uint32_t)vap->va_rdev;
	fmi->umask = 0; 	/* Seems unused bu libfuse */
	(void)strlcpy((char *)(void *)(fmi + 1), path, len - sizeof(*fmi));

	return node_mk_common(pu, opc, pni, pcn, pm);
}


int
perfuse_node_open(pu, opc, mode, pcr)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	int mode;
	const struct puffs_cred *pcr;
{
	struct perfuse_state *ps;
	struct perfuse_node_data *pnd;
	perfuse_msg_t *pm;
	mode_t pmode;
	mode_t fmode;
	int op;
	struct fuse_open_in *foi;
	struct fuse_open_out *foo;
	struct puffs_node *pn;
	int error;
	
	ps = puffs_getspecific(pu);
	pn = (struct puffs_node *)opc;
	pnd = PERFUSE_NODE_DATA(opc);
	pm = NULL;
	error = 0;

	if (pnd->pnd_flags & PND_REMOVED)
		return ENOENT;

	if (puffs_pn_getvap(pn)->va_type == VDIR) {
		op = FUSE_OPENDIR;
		pmode = PUFFS_VREAD;
	} else {
		op = FUSE_OPEN;
		if (mode & FWRITE)
			pmode = PUFFS_VWRITE|PUFFS_VREAD;
		else
			pmode = PUFFS_VREAD;
	}

	/*
	 * Opening a directory require R-- on the directory
	 * Opening a file requires R-- for reading, -W- for writing
	 * In both cases, R-- is required on the parent.
	 */
	if ((no_access((puffs_cookie_t)pnd->pnd_parent, pcr, PUFFS_VREAD)) ||
	    (no_access(opc, pcr, pmode))) {
		error = EACCES;
		goto out;
	}

	/*
	 * libfuse docs say O_CREAT should not be set.
	 */
	mode &= ~O_CREAT;

	/*
	 * Do not open twice, and do not reopen for reading
	 * if we already have write handle.
	 */
	if (((mode & FREAD) && (pnd->pnd_flags & PND_RFH)) ||
	    ((mode & FREAD) && (pnd->pnd_flags & PND_WFH)) ||
	    ((mode & FWRITE) && (pnd->pnd_flags & PND_WFH))) {
		error = 0;
		goto out;
	}
	
	/*
	 * Queue open on a node so that we do not open
	 * twice. This would be better with read and
	 * write distinguished.
	 */
	while (pnd->pnd_flags & PND_INOPEN)
		requeue_request(pu, opc, PCQ_OPEN);
	pnd->pnd_flags |= PND_INOPEN;

	/*
	 * Convert PUFFS mode to FUSE mode: convert FREAD/FWRITE
	 * to O_RDONLY/O_WRONLY while perserving the other options.
	 */
	fmode = mode & ~(FREAD|FWRITE);
	fmode |= (mode & FWRITE) ? O_RDWR : O_RDONLY;

	pm = ps->ps_new_msg(pu, opc, op, sizeof(*foi), pcr);
	foi = GET_INPAYLOAD(ps, pm, fuse_open_in);
	foi->flags = fmode;
	foi->unused = 0;

	if ((error = xchg_msg(pu, opc, pm, sizeof(*foo), wait_reply)) != 0)
		goto out;

	foo = GET_OUTPAYLOAD(ps, pm, fuse_open_out);

	/*
	 * Save the file handle in node private data 
	 * so that we can reuse it later
	 */
	perfuse_new_fh(opc, foo->fh, mode);
	 
#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & (PDF_FH|PDF_FILENAME))
		DPRINTF("%s: opc = %p, file = \"%s\", "
			"ino = %"PRId64", %s%sfh = 0x%"PRIx64"\n",
			__func__, (void *)opc, perfuse_node_path(opc),
			pnd->pnd_ino, mode & FREAD ? "r" : "",
			mode & FWRITE ? "w" : "", foo->fh);
#endif

out:
	if (pm != NULL)
		ps->ps_destroy_msg(pm);

	pnd->pnd_flags &= ~PND_INOPEN;
	(void)dequeue_requests(ps, opc, PCQ_OPEN, DEQUEUE_ALL);

	return error;
}

/* ARGSUSED0 */
int
perfuse_node_close(pu, opc, flags, pcr)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	int flags;
	const struct puffs_cred *pcr;
{
	struct perfuse_node_data *pnd;

	pnd = PERFUSE_NODE_DATA(opc);

	if (!(pnd->pnd_flags & PND_OPEN))
		return EBADF;

	/* 
	 * Actual close is postponed at inactive time.
	 */
	return 0;
}

/*
 * XXX
 * This fails as unprivilegied, it should not: touch testa/testx/a
 * d-wx-wx-wx  2 root  wheel  512 Oct  5 04:32 testa/testx
 * -rwxrwxrwx  1 root  wheel  0   Oct  5 04:39 testa/testx/a
 */
int
perfuse_node_access(pu, opc, mode, pcr)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	int mode;
	const struct puffs_cred *pcr;
{
	perfuse_msg_t *pm;
	struct perfuse_state *ps;
	struct fuse_access_in *fai;
	int error;
	
	if (PERFUSE_NODE_DATA(opc)->pnd_flags & PND_REMOVED)
		return ENOENT;

	/* 
	 * If we previously detected the filesystem does not 
	 * implement access(), short-circuit the call and skip 
	 * to libpffs access() emulation.
	 */
	ps = puffs_getspecific(pu);
	if (ps->ps_flags & PS_NO_ACCESS) {
		error = ENOSYS;
	} else {
		pm = ps->ps_new_msg(pu, opc, FUSE_ACCESS, sizeof(*fai), pcr);
		fai = GET_INPAYLOAD(ps, pm, fuse_access_in);
		fai->mask = mode;

		error = xchg_msg(pu, opc, pm, NO_PAYLOAD_REPLY_LEN, wait_reply);
		ps->ps_destroy_msg(pm);
	}

	if (error == ENOSYS) {
		struct fuse_getattr_in *fgi;
		struct fuse_attr_out *fao;

		ps->ps_flags |= PS_NO_ACCESS;

		pm = ps->ps_new_msg(pu, opc, FUSE_GETATTR, 
			      sizeof(*fgi), NULL);
		fgi = GET_INPAYLOAD(ps, pm, fuse_getattr_in);
		fgi->getattr_flags = 0; 
		fgi->dummy = 0;
		fgi->fh = 0;

		if (PERFUSE_NODE_DATA(opc)->pnd_flags & PND_OPEN) {
			fgi->fh = perfuse_get_fh(opc, FREAD);
			fgi->getattr_flags |= FUSE_GETATTR_FH; 
		}

#ifdef PERFUSE_DEBUG
		if (perfuse_diagflags & PDF_FH)
			DPRINTF("%s: opc = %p, ino = %"PRId64", "
				"fh = 0x%"PRIx64"\n", __func__, (void *)opc,
				PERFUSE_NODE_DATA(opc)->pnd_ino, fgi->fh);
#endif
		if ((error = xchg_msg(pu, opc, pm, 
				      sizeof(*fao), wait_reply)) != 0) {
			ps->ps_destroy_msg(pm);
			goto out;
		}

		fao = GET_OUTPAYLOAD(ps, pm, fuse_attr_out);

		error = puffs_access(VREG, fao->attr.mode, fao->attr.uid,
				     fao->attr.gid, (mode_t)mode, pcr); 

		
		fuse_attr_to_vap(ps,
				 &((struct puffs_node *)opc)->pn_va,
				 &fao->attr);

		ps->ps_destroy_msg(pm);
	}

out:
	return error;
}

int
perfuse_node_getattr(pu, opc, vap, pcr)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	struct vattr *vap;
	const struct puffs_cred *pcr;
{
	perfuse_msg_t *pm;
	struct perfuse_state *ps;
	struct fuse_getattr_in *fgi;
	struct fuse_attr_out *fao;
	int error;
	
	if (PERFUSE_NODE_DATA(opc)->pnd_flags & PND_REMOVED)
		return ENOENT;

	/*
	 * getattr requires --X on the parent directory 
	 * no right is required on the object.
	 */
	if (no_access(PERFUSE_NODE_DATA(opc)->pnd_parent, pcr, PUFFS_VEXEC))
		return EACCES;

	ps = puffs_getspecific(pu);

	/*
	 * FUSE_GETATTR_FH must be set in fgi->flags 
	 * if we use for fgi->fh
	 */
	pm = ps->ps_new_msg(pu, opc, FUSE_GETATTR, sizeof(*fgi), pcr);
	fgi = GET_INPAYLOAD(ps, pm, fuse_getattr_in);
	fgi->getattr_flags = 0; 
	fgi->dummy = 0;
	fgi->fh = 0;

	if (PERFUSE_NODE_DATA(opc)->pnd_flags & PND_OPEN) {
		fgi->fh = perfuse_get_fh(opc, FREAD);
		fgi->getattr_flags |= FUSE_GETATTR_FH;
	}

	if ((error = xchg_msg(pu, opc, pm, sizeof(*fao), wait_reply)) != 0)
		goto out;

	fao = GET_OUTPAYLOAD(ps, pm, fuse_attr_out);

	/* 
	 * The message from filesystem has a cache timeout
	 * XXX this is ignored yet, is that right?
	 * 
	 * We also set birthtime, flags, filerev,vaflags to 0. 
	 * This seems the best bet, since the information is
	 * not available from filesystem.
	 */
	fuse_attr_to_vap(ps, vap, &fao->attr);

out:
	ps->ps_destroy_msg(pm);

	return error;
}

int
perfuse_node_setattr(pu, opc, vap, pcr)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	const struct vattr *vap;
	const struct puffs_cred *pcr;
{
	perfuse_msg_t *pm;
	uint64_t fh;
	struct perfuse_state *ps;
	struct perfuse_node_data *pnd;
	struct fuse_setattr_in *fsi;
	struct vattr *old_va;
	int error;

	ps = puffs_getspecific(pu);
	pnd = PERFUSE_NODE_DATA(opc);
	pm = NULL;

	/*
	 * The only operation we can do once the file is removed 
	 * is to resize it, and we can do it only if it is open.
	 * Do not even send the operation to the filesystem: the
	 * file is not there anymore.
	 */
	if (pnd->pnd_flags & PND_REMOVED) {
		if (!(pnd->pnd_flags & PND_OPEN))
			return ENOENT;
		
		error = 0;
		goto out;
	}

	old_va = puffs_pn_getvap((struct puffs_node *)opc);

	/*
	 * Check for permission to change size
	 */
	if ((vap->va_size != (u_quad_t)PUFFS_VNOVAL) &&
	    no_access(opc, pcr, PUFFS_VWRITE))
		return EACCES;

	/*
	 * Check for permission to change dates
	 */
	if (((vap->va_atime.tv_sec != (time_t)PUFFS_VNOVAL) ||
	     (vap->va_mtime.tv_sec != (time_t)PUFFS_VNOVAL)) &&
	    (puffs_access_times(old_va->va_uid, old_va->va_gid,
				old_va->va_mode, 0, pcr) != 0))
		return EACCES;

	/*
	 * Check for permission to change owner and group
	 */
	if (((vap->va_uid != (uid_t)PUFFS_VNOVAL) ||
	     (vap->va_gid != (gid_t)PUFFS_VNOVAL)) &&
	    (puffs_access_chown(old_va->va_uid, old_va->va_gid,
				vap->va_uid, vap->va_gid, pcr)) != 0)
		return EACCES;

	/*
	 * Check for permission to change permissions
	 */
	if ((vap->va_mode != (mode_t)PUFFS_VNOVAL) &&
	    (puffs_access_chmod(old_va->va_uid, old_va->va_gid,
				old_va->va_type, vap->va_mode, pcr)) != 0)
		return EACCES;
	
	/*
	 * It seems troublesome to resize a file while
	 * a write is just beeing done. Wait for
	 * it to finish.
	 */
	if (vap->va_size != (u_quad_t)PUFFS_VNOVAL)
		while (pnd->pnd_flags & PND_INWRITE)
			requeue_request(pu, opc, PCQ_AFTERWRITE);

	pm = ps->ps_new_msg(pu, opc, FUSE_SETATTR, sizeof(*fsi), pcr);
	fsi = GET_INPAYLOAD(ps, pm, fuse_setattr_in);
	fsi->valid = 0;

	/*
	 * Get a fh if the node is open for writing
	 */
	if (pnd->pnd_flags & PND_WFH) {
		fh = perfuse_get_fh(opc, FWRITE);
		fsi->fh = fh;
		fsi->valid |= FUSE_FATTR_FH;
	}

	if (vap->va_size != (u_quad_t)PUFFS_VNOVAL) {
		fsi->size = vap->va_size;
		fsi->valid |= FUSE_FATTR_SIZE;
	}

	/*
 	 * Setting mtime without atime or vice verse leads to
	 * dates being reset to Epoch on glusterfs. If one
	 * is missing, use the old value.
 	 */
	if ((vap->va_mtime.tv_sec != (time_t)PUFFS_VNOVAL) || 
	    (vap->va_atime.tv_sec != (time_t)PUFFS_VNOVAL)) {
		
		if (vap->va_atime.tv_sec != (time_t)PUFFS_VNOVAL) {
			fsi->atime = vap->va_atime.tv_sec;
			fsi->atimensec = (uint32_t)vap->va_atime.tv_nsec;
		} else {
			fsi->atime = old_va->va_atime.tv_sec;
			fsi->atimensec = (uint32_t)old_va->va_atime.tv_nsec;
		}

		if (vap->va_mtime.tv_sec != (time_t)PUFFS_VNOVAL) {
			fsi->mtime = vap->va_mtime.tv_sec;
			fsi->mtimensec = (uint32_t)vap->va_mtime.tv_nsec;
		} else {
			fsi->mtime = old_va->va_mtime.tv_sec;
			fsi->mtimensec = (uint32_t)old_va->va_mtime.tv_nsec;
		}

		fsi->valid |= (FUSE_FATTR_MTIME|FUSE_FATTR_ATIME);
	}

	if (vap->va_mode != (mode_t)PUFFS_VNOVAL) {
		fsi->mode = vap->va_mode; 
		fsi->valid |= FUSE_FATTR_MODE;
	}
	
	if (vap->va_uid != (uid_t)PUFFS_VNOVAL) {
		fsi->uid = vap->va_uid;
		fsi->valid |= FUSE_FATTR_UID;
	}

	if (vap->va_gid != (gid_t)PUFFS_VNOVAL) {
		fsi->gid = vap->va_gid;
		fsi->valid |= FUSE_FATTR_GID;
	}

	if (pnd->pnd_lock_owner != 0) {
		fsi->lock_owner = pnd->pnd_lock_owner;
		fsi->valid |= FUSE_FATTR_LOCKOWNER;
	}

	/*
	 * If nothing remain, discard the operation.
	 */
	if (!(fsi->valid & (FUSE_FATTR_SIZE|FUSE_FATTR_ATIME|FUSE_FATTR_MTIME|
			    FUSE_FATTR_MODE|FUSE_FATTR_UID|FUSE_FATTR_GID))) {
		error = 0;
		goto out;
	}

	/*
	 * A fuse_attr_out is returned, but we ignore it.
	 */
	error = xchg_msg(pu, opc, pm, sizeof(struct fuse_attr_out), wait_reply);

out:

	if (pm != NULL)
		ps->ps_destroy_msg(pm);

	return error;
}

int
perfuse_node_poll(pu, opc, events)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	int *events;
{
	struct perfuse_state *ps;
	perfuse_msg_t *pm;
	struct fuse_poll_in *fpi;
	struct fuse_poll_out *fpo;
	int error;

	ps = puffs_getspecific(pu);
	/*
	 * kh is set if FUSE_POLL_SCHEDULE_NOTIFY is set.
 	 */
	pm = ps->ps_new_msg(pu, opc, FUSE_POLL, sizeof(*fpi), NULL);
	fpi = GET_INPAYLOAD(ps, pm, fuse_poll_in);
	fpi->fh = perfuse_get_fh(opc, FREAD);
	fpi->kh = 0;
	fpi->flags = 0;

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_FH)
		DPRINTF("%s: opc = %p, ino = %"PRId64", fh = 0x%"PRIx64"\n",
			__func__, (void *)opc,	
			PERFUSE_NODE_DATA(opc)->pnd_ino, fpi->fh);
#endif
	if ((error = xchg_msg(pu, opc, pm, sizeof(*fpo), wait_reply)) != 0)
		goto out;

	fpo = GET_OUTPAYLOAD(ps, pm, fuse_poll_out);
	*events = fpo->revents;
out:
	ps->ps_destroy_msg(pm);

	return error;
}

/* ARGSUSED0 */
int
perfuse_node_mmap(pu, opc, flags, pcr)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	int flags;
	const struct puffs_cred *pcr;
{
	/* 
	 * Not implemented anymore in libfuse
	 */
	return ENOSYS;
}

/* ARGSUSED2 */
int
perfuse_node_fsync(pu, opc, pcr, flags, offlo, offhi)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	const struct puffs_cred *pcr;
	int flags;
	off_t offlo;
	off_t offhi;
{
	int op;
	perfuse_msg_t *pm;
	struct perfuse_state *ps;
	struct perfuse_node_data *pnd;
	struct fuse_fsync_in *ffi;
	uint64_t fh;
	int error;
	
	pm = NULL;
	ps = puffs_getspecific(pu);
	pnd = PERFUSE_NODE_DATA(opc);

	/*
	 * No need to sync a removed node
	 */
	if (pnd->pnd_flags & PND_REMOVED)
		return 0;

	/*
	 * We do not sync closed files. They have been 
	 * sync at inactive time already.
	 */
	if (!(pnd->pnd_flags & PND_OPEN))
		return 0;

	if (puffs_pn_getvap((struct puffs_node *)opc)->va_type == VDIR) 
		op = FUSE_FSYNCDIR;
	else 		/* VREG but also other types such as VLNK */
		op = FUSE_FSYNC;

	/*
	 * Do not sync if there are no change to sync
	 * XXX remove that test on files if we implement mmap
	 */
#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_SYNC)
		DPRINTF("%s: TEST opc = %p, file = \"%s\" is %sdirty\n", 
			__func__, (void*)opc, perfuse_node_path(opc),
			pnd->pnd_flags & PND_DIRTY ? "" : "not ");
#endif
	if (!(pnd->pnd_flags & PND_DIRTY))
		return 0;

	/*
	 * It seems NetBSD can call fsync without open first
	 * glusterfs complain in such a situation:
	 * "FSYNC() ERR => -1 (Invalid argument)"
	 * The file will be closed at inactive time.
	 * 
	 * We open the directory for reading in order to sync.
	 * This sounds rather counterintuitive, but it works.
	 */
	if (!(pnd->pnd_flags & PND_WFH)) {
		if ((error = perfuse_node_open(pu, opc, FREAD, pcr)) != 0)
			goto out;
	}

	if (op == FUSE_FSYNCDIR)
		fh = perfuse_get_fh(opc, FREAD);
	else
		fh = perfuse_get_fh(opc, FWRITE);
	
	/*
	 * If fsync_flags  is set, meta data should not be flushed.
	 */
	pm = ps->ps_new_msg(pu, opc, op, sizeof(*ffi), NULL);
	ffi = GET_INPAYLOAD(ps, pm, fuse_fsync_in);
	ffi->fh = fh;
	ffi->fsync_flags = (flags & FFILESYNC) ? 0 : 1;

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_FH)
		DPRINTF("%s: opc = %p, ino = %"PRId64", fh = 0x%"PRIx64"\n",
			__func__, (void *)opc,
			PERFUSE_NODE_DATA(opc)->pnd_ino, ffi->fh);
#endif

	if ((error = xchg_msg(pu, opc, pm, 
			      NO_PAYLOAD_REPLY_LEN, wait_reply)) != 0)
		goto out;	

	/*
	 * No reply beyond fuse_out_header: nothing to do on success
	 * just clear the dirty flag
	 */
	pnd->pnd_flags &= ~PND_DIRTY;

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_SYNC)
		DPRINTF("%s: CLEAR opc = %p, file = \"%s\"\n", 
			__func__, (void*)opc, perfuse_node_path(opc));
#endif

out:
	/*
	 * ENOSYS is not returned to kernel,
	 */
	if (error == ENOSYS)
		error = 0;

	if (pm != NULL)
		ps->ps_destroy_msg(pm);

	return error;
}

/* ARGSUSED0 */
int
perfuse_node_seek(pu, opc, oldoff, newoff,  pcr)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	off_t oldoff;
	off_t newoff;
	const struct puffs_cred *pcr;
{
	return 0;
}

int
perfuse_node_remove(pu, opc, targ, pcn)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	puffs_cookie_t targ;
	const struct puffs_cn *pcn;
{
	struct perfuse_state *ps;
	struct perfuse_node_data *pnd;
	perfuse_msg_t *pm;
	char *path;
	const char *name;
	size_t len;
	int error;
	
	pnd = PERFUSE_NODE_DATA(opc);

	if ((pnd->pnd_flags & PND_REMOVED) ||
	    (PERFUSE_NODE_DATA(targ)->pnd_flags & PND_REMOVED))
		return ENOENT;

#if notyet
	/*
	 * remove requires -WX on the parent directory 
	 * no right required on the object.
	 *
	 * XXX PUFFS interface gives no pcr here
	 */
	if (no_access((puffs_cookie_t)pnd->pnd_parent, pcr, 
		      PUFFS_VWRITE|PUFFS_VEXEC))
		return EACCES;
#endif

#ifdef PERFUSE_DEBUG
	if (targ == NULL)
		DERRX(EX_SOFTWARE, "%s: targ is NULL", __func__);

	if (perfuse_diagflags & (PDF_FH|PDF_FILENAME))
		DPRINTF("%s: opc = %p, remove opc = %p, file = \"%s\"\n",
			__func__, (void *)opc, (void *)targ, pcn->pcn_name);
#endif
	/*
	 * Await for all operations on the deleted node to drain, 
	 * as the filesystem may be confused to have it deleted
	 * during a getattr
	 */
	while (PERFUSE_NODE_DATA(targ)->pnd_flags & PND_INXCHG)
		requeue_request(pu, targ, PCQ_AFTERXCHG);

	ps = puffs_getspecific(pu);
	pnd = PERFUSE_NODE_DATA(opc);
	name = pcn->pcn_name;
	len = pcn->pcn_namelen + 1;

	pm = ps->ps_new_msg(pu, opc, FUSE_UNLINK, len, NULL);
	path = _GET_INPAYLOAD(ps, pm, char *);
	(void)strlcpy(path, name, len);

	if ((error = xchg_msg(pu, opc, pm, UNSPEC_REPLY_LEN, wait_reply)) != 0)
		goto out;

	PERFUSE_NODE_DATA(targ)->pnd_flags |= PND_REMOVED;
	if (!(PERFUSE_NODE_DATA(targ)->pnd_flags & PND_OPEN))
		puffs_setback(puffs_cc_getcc(pu), PUFFS_SETBACK_NOREF_N2);

	/*
	 * The parent directory needs a sync
	 */
	PERFUSE_NODE_DATA(opc)->pnd_flags |= PND_DIRTY;

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_FILENAME)
		DPRINTF("%s: remove nodeid = %"PRId64" file = \"%s\"\n",
			__func__, PERFUSE_NODE_DATA(targ)->pnd_ino,
			pcn->pcn_name);
#endif
out:
	ps->ps_destroy_msg(pm);

	return error;
}

int
perfuse_node_link(pu, opc, targ, pcn)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	puffs_cookie_t targ;
	const struct puffs_cn *pcn;
{
	struct perfuse_state *ps;
	perfuse_msg_t *pm;
	const char *name;
	size_t len;
	struct puffs_node *pn;
	struct fuse_link_in *fli;
	int error;

	if (PERFUSE_NODE_DATA(opc)->pnd_flags & PND_REMOVED)
		return ENOENT;
	
#if notyet
	/*
	 * Create an object require -W- permission in the parent directory
	 *
	 * XXX PUFFS interface gives no pcr here
	 */
	if (no_access(opc, pcr, PUFFS_VWRITE))
		return EACCES;
#endif


	ps = puffs_getspecific(pu);
	pn = (struct puffs_node *)targ;
	name = pcn->pcn_name;
	len =  sizeof(*fli) + pcn->pcn_namelen + 1;

	pm = ps->ps_new_msg(pu, opc, FUSE_LINK, len, NULL);
	fli = GET_INPAYLOAD(ps, pm, fuse_link_in);
	fli->oldnodeid = PERFUSE_NODE_DATA(pn)->pnd_ino;
	(void)strlcpy((char *)(void *)(fli + 1), name, len - sizeof(*fli));

	error = xchg_msg(pu, opc, pm, UNSPEC_REPLY_LEN, wait_reply);

	ps->ps_destroy_msg(pm);

	return error;
}

int
perfuse_node_rename(pu, opc, src, pcn_src, targ_dir, targ, pcn_targ)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	puffs_cookie_t src;
	const struct puffs_cn *pcn_src;
	puffs_cookie_t targ_dir;
	puffs_cookie_t targ;
	const struct puffs_cn *pcn_targ;
{
	struct perfuse_state *ps;
	perfuse_msg_t *pm;
	struct fuse_rename_in *fri;
	const char *newname;
	const char *oldname;
	char *np;
	int error;
	size_t len;
	size_t newname_len;
	size_t oldname_len;
	
	if ((PERFUSE_NODE_DATA(opc)->pnd_flags & PND_REMOVED) ||
	    (PERFUSE_NODE_DATA(src)->pnd_flags & PND_REMOVED) ||
	    (PERFUSE_NODE_DATA(targ_dir)->pnd_flags & PND_REMOVED))
		return ENOENT;

#if notyet
	/*
	 * move requires -WX on source and destination directory 
	 *
	 * XXX PUFFS interface gives no pcr here
	 */
	if (no_access(opc, pcr_src, PUFFS_VWRITE|PUFFS_VEXEC) ||
	    no_access(targ_dir,  pcr_targ, PUFFS_VWRITE|PUFFS_VEXEC))
		return EACCES;
#endif

	/*
	 * Await for all operations on the deleted node to drain, 
	 * as the filesystem may be confused to have it deleted
	 * during a getattr
	 */
	if ((struct puffs_node *)targ != NULL) {
		while (PERFUSE_NODE_DATA(targ)->pnd_flags & PND_INXCHG)
			requeue_request(pu, targ, PCQ_AFTERXCHG);
	} else {
		while (PERFUSE_NODE_DATA(src)->pnd_flags & PND_INXCHG)
			requeue_request(pu, src, PCQ_AFTERXCHG);
	}

	ps = puffs_getspecific(pu);
	newname =  pcn_targ->pcn_name;
	newname_len = pcn_targ->pcn_namelen + 1;
	oldname =  pcn_src->pcn_name;
	oldname_len = pcn_src->pcn_namelen + 1;

	len = sizeof(*fri) + oldname_len + newname_len;
	pm = ps->ps_new_msg(pu, opc, FUSE_RENAME, len, NULL);
	fri = GET_INPAYLOAD(ps, pm, fuse_rename_in);
	fri->newdir = PERFUSE_NODE_DATA(targ_dir)->pnd_ino;
	np = (char *)(void *)(fri + 1);
	(void)strlcpy(np, oldname, oldname_len);
	np += oldname_len;
	(void)strlcpy(np, newname, newname_len);

	if ((error = xchg_msg(pu, opc, pm, UNSPEC_REPLY_LEN, wait_reply)) != 0)
		goto out;

	if (opc != targ_dir) {
		struct perfuse_node_data *srcdir_pnd;
		struct perfuse_node_data *dstdir_pnd;
		struct perfuse_node_data *src_pnd;
		
		srcdir_pnd = PERFUSE_NODE_DATA(opc);
		dstdir_pnd = PERFUSE_NODE_DATA(targ_dir);
		src_pnd = PERFUSE_NODE_DATA(src);

		TAILQ_REMOVE(&srcdir_pnd->pnd_children, src_pnd, pnd_next);
		TAILQ_INSERT_TAIL(&dstdir_pnd->pnd_children, src_pnd, pnd_next);

		srcdir_pnd->pnd_childcount--;
		dstdir_pnd->pnd_childcount++;

		src_pnd->pnd_parent = targ_dir;

		PERFUSE_NODE_DATA(targ_dir)->pnd_flags |= PND_DIRTY;
	}

	(void)strlcpy(PERFUSE_NODE_DATA(src)->pnd_name, newname, MAXPATHLEN);

	PERFUSE_NODE_DATA(opc)->pnd_flags |= PND_DIRTY;

	if ((struct puffs_node *)targ != NULL)
		PERFUSE_NODE_DATA(targ)->pnd_flags |= PND_REMOVED;

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_FILENAME)
		DPRINTF("%s: nodeid = %"PRId64" file = \"%s\" renamed \"%s\" "
			"nodeid = %"PRId64" -> nodeid = %"PRId64" \"%s\"\n",
	 		__func__, PERFUSE_NODE_DATA(src)->pnd_ino,
			pcn_src->pcn_name, pcn_targ->pcn_name,
			PERFUSE_NODE_DATA(opc)->pnd_ino,
			PERFUSE_NODE_DATA(targ_dir)->pnd_ino,
			perfuse_node_path(targ_dir));
#endif

out:
	if (pm != NULL)
		ps->ps_destroy_msg(pm);

	return error;
}

int
perfuse_node_mkdir(pu, opc, pni, pcn, vap)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	struct puffs_newinfo *pni;
	const struct puffs_cn *pcn;
	const struct vattr *vap;
{
	struct perfuse_state *ps;
	perfuse_msg_t *pm;
	struct fuse_mkdir_in *fmi;
	const char *path;
	size_t len;
	
	if (PERFUSE_NODE_DATA(opc)->pnd_flags & PND_REMOVED)
		return ENOENT;

#if notyet
	/*
	 * Create an object require -WX permission in the parent directory
	 *
	 * XXX PUFFS interface gives no pcr here
	 */
	if (no_access(opc, pcrx, PUFFS_VWRITE|PUFFS_VEXEC))
		return EACCES;
#endif

	ps = puffs_getspecific(pu);
	path = pcn->pcn_name;
	len = sizeof(*fmi) + pcn->pcn_namelen + 1; 

	pm = ps->ps_new_msg(pu, opc, FUSE_MKDIR, len, NULL);
	fmi = GET_INPAYLOAD(ps, pm, fuse_mkdir_in);
	fmi->mode = vap->va_mode;
	fmi->umask = 0; 	/* Seems unused by libfuse? */
	(void)strlcpy((char *)(void *)(fmi + 1), path, len - sizeof(*fmi));

	return node_mk_common(pu, opc, pni, pcn, pm);
}


int
perfuse_node_rmdir(pu, opc, targ, pcn)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	puffs_cookie_t targ;
	const struct puffs_cn *pcn;
{
	struct perfuse_state *ps;
	struct perfuse_node_data *pnd;
	perfuse_msg_t *pm;
	char *path;
	const char *name;
	size_t len;
	int error;
	
	pnd = PERFUSE_NODE_DATA(opc);
	if (pnd->pnd_flags & PND_REMOVED)
		return ENOENT;


#if notyet
	/*
	 * remove requires -WX on the parent directory 
	 * no right required on the object.
	 *
	 * XXX PUFFS interface gives no pcr here
	 */
	if (no_access((puffs_cookie_t)pnd->pnd_parent, pcr,
		      PUFFS_VWRITE|PUFFS_VEXEC))
		return EACCES;
#endif

	/*
	 * Await for all operations on the deleted node to drain, 
	 * as the filesystem may be confused to have it deleted
	 * during a getattr
	 */
	while (PERFUSE_NODE_DATA(targ)->pnd_flags & PND_INXCHG)
		requeue_request(pu, targ, PCQ_AFTERXCHG);

	ps = puffs_getspecific(pu);
	name = pcn->pcn_name;
	len = pcn->pcn_namelen + 1;

	pm = ps->ps_new_msg(pu, opc, FUSE_RMDIR, len, NULL);
	path = _GET_INPAYLOAD(ps, pm, char *);
	(void)strlcpy(path, name, len);

	if ((error = xchg_msg(pu, opc, pm, UNSPEC_REPLY_LEN, wait_reply)) != 0)
		goto out;

	PERFUSE_NODE_DATA(targ)->pnd_flags |= PND_REMOVED;
	if (!(PERFUSE_NODE_DATA(targ)->pnd_flags & PND_OPEN))
		puffs_setback(puffs_cc_getcc(pu), PUFFS_SETBACK_NOREF_N2);

	/*
	 * The parent directory needs a sync
	 */
	PERFUSE_NODE_DATA(opc)->pnd_flags |= PND_DIRTY;

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_FILENAME)
		DPRINTF("%s: remove nodeid = %"PRId64" file = \"%s\"\n",
			__func__, PERFUSE_NODE_DATA(targ)->pnd_ino,
			perfuse_node_path(targ));
#endif
out:
	ps->ps_destroy_msg(pm);

	return error;
}

/* vap is unused */
/* ARGSUSED4 */
int
perfuse_node_symlink(pu, opc, pni, pcn_src, vap, link_target)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	struct puffs_newinfo *pni;
	const struct puffs_cn *pcn_src;
	const struct vattr *vap;
	const char *link_target;
{
	struct perfuse_state *ps;
	perfuse_msg_t *pm;
	char *np;
	const char *path;
	size_t path_len;
	size_t linkname_len;
	size_t len;
	
	if (PERFUSE_NODE_DATA(opc)->pnd_flags & PND_REMOVED)
		return ENOENT;

#if notyet
	/*
	 * Create an object require -W- permission in the parent directory
	 *
	 * XXX PUFFS interface gives no pcr here
	 */
	if (no_access(opc, pcr_src, PUFFS_VWRITE))
		return EACCES;
#endif

	ps = puffs_getspecific(pu);
	path = pcn_src->pcn_name;
	path_len = pcn_src->pcn_namelen + 1;
	linkname_len = strlen(link_target) + 1;
	len = path_len + linkname_len;

	pm = ps->ps_new_msg(pu, opc, FUSE_SYMLINK, len, NULL);
	np = _GET_INPAYLOAD(ps, pm, char *);
	(void)strlcpy(np, path, path_len);
	np += path_len;
	(void)strlcpy(np, link_target, linkname_len);

	return node_mk_common(pu, opc, pni, pcn_src, pm);
}

int 
perfuse_node_readdir(pu, opc, dent, readoff, 
		     reslen, pcr, eofflag, cookies, ncookies)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	struct dirent *dent;
	off_t *readoff;
	size_t *reslen;
	const struct puffs_cred *pcr;
	int *eofflag;
	off_t *cookies;
	size_t *ncookies;
{
	perfuse_msg_t *pm;
	uint64_t fh;
	struct perfuse_state *ps;
	struct perfuse_node_data *pnd;
	struct fuse_read_in *fri;
	struct fuse_out_header *foh;
	struct fuse_dirent *fd;
	size_t foh_len;
	int error;
	uint64_t fd_offset;
	size_t fd_maxlen;
	
	pm = NULL;
	error = 0;
	ps = puffs_getspecific(pu);

	/*
	 * readdir state is kept at node level, and several readdir
	 * requests can be issued at the same time on the same node.
	 * We need to queue requests so that only one is in readdir
	 * code at the same time.
	 */
	pnd = PERFUSE_NODE_DATA(opc);
	while (pnd->pnd_flags & PND_INREADDIR)
		requeue_request(pu, opc, PCQ_READDIR);
	pnd->pnd_flags |= PND_INREADDIR;

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_READDIR)
		DPRINTF("%s: READDIR opc = %p enter critical section\n",
			__func__, (void *)opc);
#endif
	/*
	 * Do we already have the data bufered?
	 */
	if (pnd->pnd_dirent != NULL)
		goto out;
	pnd->pnd_dirent_len = 0;
	
	/*
	 * It seems NetBSD can call readdir without open first
	 * libfuse will crash if it is done that way, hence open first.
	 */
	if (!(pnd->pnd_flags & PND_OPEN)) {
		if ((error = perfuse_node_open(pu, opc, FREAD, pcr)) != 0)
			goto out;
	}

	fh = perfuse_get_fh(opc, FREAD);

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_FH)
		DPRINTF("%s: opc = %p, ino = %"PRId64", rfh = 0x%"PRIx64"\n",
			__func__, (void *)opc,
			PERFUSE_NODE_DATA(opc)->pnd_ino, fh);
#endif

	pnd->pnd_all_fd = NULL;
	pnd->pnd_all_fd_len = 0;
	fd_offset = 0;
	fd_maxlen = ps->ps_max_readahead - sizeof(*foh);
	
	do {
		size_t fd_len;
		char *afdp;
		
		pm = ps->ps_new_msg(pu, opc, FUSE_READDIR, sizeof(*fri), pcr);

		/*
		 * read_flags, lock_owner and flags are unused in libfuse
		 */
		fri = GET_INPAYLOAD(ps, pm, fuse_read_in);
		fri->fh = fh;
		fri->offset = fd_offset;
		fri->size = fd_maxlen;
		fri->read_flags = 0;
		fri->lock_owner = 0;
		fri->flags = 0;

		if ((error = xchg_msg(pu, opc, pm, 	
				      UNSPEC_REPLY_LEN, wait_reply)) != 0)
			goto out;
		
		/* 
		 * There are many puffs_framebufs calls later,
		 * therefore foh will not be valid for a long time.
		 * Just get the length and forget it.
		 */
		foh = GET_OUTHDR(ps, pm);
		foh_len = foh->len;

		/*
		 * Empty read: we reached the end of the buffer.
		 */
		if (foh_len == sizeof(*foh))
			break;

		/*
		 * Corrupted message.
		 */
		if (foh_len < sizeof(*foh) + sizeof(*fd)) {
			DWARNX("readdir reply too short");
			error = EIO;
			goto out;
		}

		
		fd = GET_OUTPAYLOAD(ps, pm, fuse_dirent);
		fd_len = foh_len - sizeof(*foh);

		pnd->pnd_all_fd = realloc(pnd->pnd_all_fd, 
					  pnd->pnd_all_fd_len + fd_len);
		if (pnd->pnd_all_fd  == NULL)
			DERR(EX_OSERR, "malloc failed");

		afdp = (char *)(void *)pnd->pnd_all_fd + pnd->pnd_all_fd_len;
		(void)memcpy(afdp, fd, fd_len);

		pnd->pnd_all_fd_len += fd_len;
		fd_offset += fd_len;

		ps->ps_destroy_msg(pm);
		pm = NULL;

		/*
		 * If the buffer was not completely filled, 
		 * that is, if there is room for the biggest 
		 * struct dirent possible, then we are done:
		 * no need to issue another READDIR to see
		 * an empty reply.
		 */
	} while (foh_len >= fd_maxlen - (sizeof(*fd) + MAXPATHLEN));

	if (fuse_to_dirent(pu, opc, pnd->pnd_all_fd, pnd->pnd_all_fd_len) == -1)
		error = EIO;

out:
	if (pnd->pnd_all_fd != NULL) {
		free(pnd->pnd_all_fd);
		pnd->pnd_all_fd = NULL;
		pnd->pnd_all_fd_len = 0;
	}

	if (pm != NULL)
		ps->ps_destroy_msg(pm);

	if (error == 0)
		error = readdir_buffered(opc, dent, readoff,
			reslen, pcr, eofflag, cookies, ncookies);

	/*
	 * Schedule queued readdir requests
	 */
	pnd->pnd_flags &= ~PND_INREADDIR;
	(void)dequeue_requests(ps, opc, PCQ_READDIR, DEQUEUE_ALL);

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_READDIR)
		DPRINTF("%s: READDIR opc = %p exit critical section\n",
			__func__, (void *)opc);
#endif

	return error;
}

int
perfuse_node_readlink(pu, opc, pcr, linkname, linklen)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	const struct puffs_cred *pcr;
	char *linkname;
	size_t *linklen;
{
	struct perfuse_state *ps;
	perfuse_msg_t *pm;
	int error;
	size_t len;
	struct fuse_out_header *foh;
	
	if (PERFUSE_NODE_DATA(opc)->pnd_flags & PND_REMOVED)
		return ENOENT;

	/* 
	 * R-- required on parent, R-- required on link
	 */
	if (no_access((puffs_cookie_t)PERFUSE_NODE_DATA(opc)->pnd_parent,
	    pcr, PUFFS_VREAD) ||
	   no_access(opc, pcr, PUFFS_VREAD))
		return EACCES;

	ps = puffs_getspecific(pu);

	pm = ps->ps_new_msg(pu, opc, FUSE_READLINK, 0, pcr);

	if ((error = xchg_msg(pu, opc, pm, UNSPEC_REPLY_LEN, wait_reply)) != 0)
		goto out;

	foh = GET_OUTHDR(ps, pm);
	len = foh->len - sizeof(*foh);
	if (len > *linklen)
		DERRX(EX_PROTOCOL, "path len = %zd too long", len);
	if (len == 0)
		DERRX(EX_PROTOCOL, "path len = %zd too short", len);
		
	/*
	 * FUSE filesystems return a NUL terminated string, we 
	 * do not want to trailing \0
	 */
	*linklen = len - 1;
	(void)memcpy(linkname, _GET_OUTPAYLOAD(ps, pm, char *), len);
out:
	ps->ps_destroy_msg(pm);

	return error;
}

int 
perfuse_node_reclaim(pu, opc)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
{
	struct perfuse_state *ps;
	perfuse_msg_t *pm;
	struct perfuse_node_data *pnd;
	struct fuse_forget_in *ffi;
	struct puffs_node *pn;
	struct puffs_node *pn_root;
	
	ps = puffs_getspecific(pu);
	pnd = PERFUSE_NODE_DATA(opc);

	/*
	 * Never forget the root.
	 */
	if (pnd->pnd_ino == FUSE_ROOT_ID)
		return 0;

	pnd->pnd_flags |= PND_RECLAIMED;

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_RECLAIM)
		DPRINTF("%s (nodeid %"PRId64") reclaimed\n", 
			perfuse_node_path(opc), pnd->pnd_ino);
#endif

	pn_root = puffs_getroot(pu);
	pn = (struct puffs_node *)opc;
	while (pn != pn_root) {
		struct puffs_node *parent_pn;
	
		pnd = PERFUSE_NODE_DATA(pn);

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_RECLAIM)
		DPRINTF("%s (nodeid %"PRId64") is %sreclaimed, "
			"has childcount %d %s%s%s%s, pending ops:%s%s%s\n", 
		        perfuse_node_path((puffs_cookie_t)pn), pnd->pnd_ino,
		        pnd->pnd_flags & PND_RECLAIMED ? "" : "not ",
		        pnd->pnd_childcount,
			pnd->pnd_flags & PND_OPEN ? "open " : "not open",
			pnd->pnd_flags & PND_RFH ? "r" : "",
			pnd->pnd_flags & PND_WFH ? "w" : "",
			pnd->pnd_flags & PND_BUSY ? "" : " none",
			pnd->pnd_flags & PND_INREADDIR ? " readdir" : "",
			pnd->pnd_flags & PND_INWRITE ? " write" : "",
			pnd->pnd_flags & PND_INOPEN ? " open" : "");
#endif

		if (!(pnd->pnd_flags & PND_RECLAIMED) ||
		    (pnd->pnd_childcount != 0))
			return 0;

#ifdef PERFUSE_DEBUG
		if ((pnd->pnd_flags & PND_OPEN) ||
		       !TAILQ_EMPTY(&pnd->pnd_pcq))
			DERRX(EX_SOFTWARE, "%s: opc = %p: still open",
			      __func__, (void *)opc);

		if ((pnd->pnd_flags & PND_BUSY) ||
		       !TAILQ_EMPTY(&pnd->pnd_pcq))
			DERRX(EX_SOFTWARE, "%s: opc = %p: ongoing operations",
			      __func__, (void *)opc);
#endif

		/*
		 * Send the FORGET message
		 */
		pm = ps->ps_new_msg(pu, (puffs_cookie_t)pn, FUSE_FORGET, 
			      sizeof(*ffi), NULL);
		ffi = GET_INPAYLOAD(ps, pm, fuse_forget_in);
		ffi->nlookup = pnd->pnd_nlookup;

		/*
		 * No reply is expected, pm is freed in xchg_msg
		 */
		(void)xchg_msg(pu, (puffs_cookie_t)pn, 
			       pm, UNSPEC_REPLY_LEN, no_reply);

		parent_pn = pnd->pnd_parent;

		perfuse_destroy_pn(pu, pn);

		pn = parent_pn;
	}

	return 0;
}

int 
perfuse_node_inactive(pu, opc)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
{
	struct perfuse_state *ps;
	struct perfuse_node_data *pnd;

	ps = puffs_getspecific(pu);
	pnd = PERFUSE_NODE_DATA(opc);

	if (!(pnd->pnd_flags & (PND_OPEN|PND_REMOVED)))
		return 0;

	/*
	 * Make sure all operation are finished
	 * There can be an ongoing write. Other
	 * operation wait for all data before 
	 * the close/inactive.
	 */
	while (pnd->pnd_flags & PND_INWRITE)
		requeue_request(pu, opc, PCQ_AFTERWRITE);

	/*
	 * The inactive operation may be cancelled.
	 * If no open is in progress, set PND_INOPEN
	 * so that a new open will be queued.
	 */
	if (pnd->pnd_flags & PND_INOPEN)
		return 0;

	pnd->pnd_flags |= PND_INOPEN;

	/*
	 * Sync data
	 */
	if (pnd->pnd_flags & PND_DIRTY)
		(void)perfuse_node_fsync(pu, opc, NULL, 0, 0, 0);
	
	/*
	 * Close handles
	 */
	if (pnd->pnd_flags & PND_WFH)
		(void)perfuse_node_close_common(pu, opc, FWRITE);

	if (pnd->pnd_flags & PND_RFH)
		(void)perfuse_node_close_common(pu, opc, FREAD);

	/*
	 * This will cause a reclaim to be sent
	 */
	if (pnd->pnd_flags & PND_REMOVED)
		puffs_setback(puffs_cc_getcc(pu), PUFFS_SETBACK_NOREF_N1);

	/*
	 * Schedule awaiting operations
	 */
	pnd->pnd_flags &= ~PND_INOPEN;
	(void)dequeue_requests(ps, opc, PCQ_OPEN, DEQUEUE_ALL);

	return 0;
}


/* ARGSUSED0 */
int 
perfuse_node_print(pu, opc)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
{
	DERRX(EX_SOFTWARE, "%s: UNIMPLEMENTED (FATAL)", __func__);
	return 0;
}

/* ARGSUSED0 */
int
perfuse_node_pathconf(pu, opc, name, retval)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	int name;
	int *retval;
{
	DERRX(EX_SOFTWARE, "%s: UNIMPLEMENTED (FATAL)", __func__);
	return 0;
}

/* id is unused */
/* ARGSUSED2 */
int
perfuse_node_advlock(pu, opc, id, op, fl, flags)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	void *id;
	int op;
	struct flock *fl;
	int flags;
{
	struct perfuse_state *ps;
	int fop;
	perfuse_msg_t *pm;
	struct fuse_lk_in *fli;
	struct fuse_lk_out *flo;
	int error;
	
	ps = puffs_getspecific(pu);

	if (op == F_GETLK)
		fop = FUSE_GETLK;
	else
		fop = (flags & F_WAIT) ? FUSE_SETLKW : FUSE_SETLK;
			
	pm = ps->ps_new_msg(pu, opc, fop, sizeof(*fli), NULL);
	fli = GET_INPAYLOAD(ps, pm, fuse_lk_in);
	fli->fh = perfuse_get_fh(opc, FWRITE);
	fli->owner = fl->l_pid;
	fli->lk.start = fl->l_start;
	fli->lk.end = fl->l_start + fl->l_len;
	fli->lk.type = fl->l_type;
	fli->lk.pid = fl->l_pid;
	fli->lk_flags = (flags & F_FLOCK) ? FUSE_LK_FLOCK : 0;

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_FH)
		DPRINTF("%s: opc = %p, ino = %"PRId64", fh = 0x%"PRIx64"\n",
			__func__, (void *)opc,
			PERFUSE_NODE_DATA(opc)->pnd_ino, fli->fh);
#endif

	if ((error = xchg_msg(pu, opc, pm, sizeof(*flo), wait_reply)) != 0)
		goto out;

	flo = GET_OUTPAYLOAD(ps, pm, fuse_lk_out);
	fl->l_start = flo->lk.start;
	fl->l_len = flo->lk.end - flo->lk.start;
	fl->l_pid = flo->lk.pid;
	fl->l_type = flo->lk.type;
	fl->l_whence = SEEK_SET;	/* libfuse hardcodes it */

	/*
	 * Save or clear the lock
	 */
	switch (op) {
	case F_SETLK:
		PERFUSE_NODE_DATA(opc)->pnd_lock_owner = flo->lk.pid;
		break;
	case F_UNLCK:
		PERFUSE_NODE_DATA(opc)->pnd_lock_owner = 0;
		break;
	default:
		break;
	}
	
out:
	ps->ps_destroy_msg(pm);

	return error;
}

int
perfuse_node_read(pu, opc, buf, offset, resid, pcr, ioflag)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	uint8_t *buf;
	off_t offset;
	size_t *resid;
	const struct puffs_cred *pcr;
	int ioflag;
{
	struct perfuse_state *ps;
	struct perfuse_node_data *pnd;
	perfuse_msg_t *pm;
	struct fuse_read_in *fri;
	struct fuse_out_header *foh;
	size_t readen;
	int error;
	
	ps = puffs_getspecific(pu);
	pnd = PERFUSE_NODE_DATA(opc);
	pm = NULL;

	if (puffs_pn_getvap((struct puffs_node *)opc)->va_type == VDIR) 
		return EBADF;

	do {
		size_t max_read;

		max_read = ps->ps_max_readahead - sizeof(*foh);
		/*
		 * flags may be set to FUSE_READ_LOCKOWNER 
		 * if lock_owner is provided.
		 */
		pm = ps->ps_new_msg(pu, opc, FUSE_READ, sizeof(*fri), pcr);
		fri = GET_INPAYLOAD(ps, pm, fuse_read_in);
		fri->fh = perfuse_get_fh(opc, FREAD);
		fri->offset = offset;
		fri->size = (uint32_t)MIN(*resid, max_read);
		fri->read_flags = 0; /* XXX Unused by libfuse? */
		fri->lock_owner = pnd->pnd_lock_owner;
		fri->flags = 0;
		fri->flags |= (fri->lock_owner != 0) ? FUSE_READ_LOCKOWNER : 0;

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_FH)
		DPRINTF("%s: opc = %p, ino = %"PRId64", fh = 0x%"PRIx64"\n",
			__func__, (void *)opc, pnd->pnd_ino, fri->fh);
#endif
		error = xchg_msg(pu, opc, pm, UNSPEC_REPLY_LEN, wait_reply);

		if (error  != 0)
			goto out;

		foh = GET_OUTHDR(ps, pm);
		readen = foh->len - sizeof(*foh);

#ifdef PERFUSE_DEBUG
		if (readen > *resid)
			DERRX(EX_SOFTWARE, "%s: Unexpected big read %zd",
			      __func__, readen);
#endif

		(void)memcpy(buf,  _GET_OUTPAYLOAD(ps, pm, char *), readen);

		buf += readen;
		offset += readen;
		*resid -= readen;

		ps->ps_destroy_msg(pm);
		pm = NULL;
	} while ((*resid != 0) && (readen != 0));

	if (ioflag & (IO_SYNC|IO_DSYNC))
		ps->ps_syncreads++;
	else
		ps->ps_asyncreads++;

out:
	if (pm != NULL)
		ps->ps_destroy_msg(pm);

	return error;
}

int
perfuse_node_write(pu, opc, buf, offset, resid, pcr, ioflag)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	uint8_t *buf;
	off_t offset;
	size_t *resid;
	const struct puffs_cred *pcr;
	int ioflag;
{
	struct perfuse_state *ps;
	struct perfuse_node_data *pnd;
	perfuse_msg_t *pm;
	struct fuse_write_in *fwi;
	struct fuse_write_out *fwo;
	size_t data_len;
	size_t payload_len;
	size_t written;
	int error;
	
	ps = puffs_getspecific(pu);
	pnd = PERFUSE_NODE_DATA(opc);
	written = 0;

	if (puffs_pn_getvap((struct puffs_node *)opc)->va_type == VDIR) 
		return EBADF;

	/*
	 * We need to queue write requests in order to avoid
	 * dequeueing PCQ_AFTERWRITE when there are pending writes.
	 */
	while (pnd->pnd_flags & PND_INWRITE)
		requeue_request(pu, opc, PCQ_WRITE);
	pnd->pnd_flags |= PND_INWRITE;

	/*
	 * append flag: we may have to read file size first.
	 */
	if (ioflag & PUFFS_IO_APPEND)
		offset = ((struct puffs_node *)opc)->pn_va.va_size;

	pm = NULL;

	do {
		size_t max_write;
		/*
		 * There is a writepage flag when data
		 * is PAGE_SIZE-aligned. Use it for
		 * everything but the data after the last
		 * page boundary.
		 */
		max_write = ps->ps_max_write - sizeof(*fwi); 

		data_len = MIN(*resid, max_write);
		if (data_len > PAGE_SIZE)
			data_len = data_len & ~(PAGE_SIZE - 1);

		payload_len = data_len + sizeof(*fwi);

		/*
		 * flags may be set to FUSE_WRITE_CACHE (XXX usage?)
		 * or FUSE_WRITE_LOCKOWNER, if lock_owner is provided.
		 * write_flags is set to 1 for writepage.
		 */
		pm = ps->ps_new_msg(pu, opc, FUSE_WRITE, payload_len, pcr);
		fwi = GET_INPAYLOAD(ps, pm, fuse_write_in);
		fwi->fh = perfuse_get_fh(opc, FWRITE);
		fwi->offset = offset;
		fwi->size = (uint32_t)data_len;
		fwi->write_flags = (fwi->size % PAGE_SIZE) ? 0 : 1;
		fwi->lock_owner = pnd->pnd_lock_owner;
		fwi->flags = 0;
		fwi->flags |= (fwi->lock_owner != 0) ? FUSE_WRITE_LOCKOWNER : 0;
		fwi->flags |= (ioflag & IO_DIRECT) ? 0 : FUSE_WRITE_CACHE; 
		(void)memcpy((fwi + 1), buf, data_len);


#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_FH)
		DPRINTF("%s: opc = %p, ino = %"PRId64", fh = 0x%"PRIx64"\n",
			__func__, (void *)opc, pnd->pnd_ino, fwi->fh);
#endif
		if ((error = xchg_msg(pu, opc, pm, 
				      sizeof(*fwo), wait_reply)) != 0)
			goto out;

		fwo = GET_OUTPAYLOAD(ps, pm, fuse_write_out);
		written = fwo->size;
#ifdef PERFUSE_DEBUG
		if (written > *resid)
			DERRX(EX_SOFTWARE, "%s: Unexpected big write %zd",
			      __func__, written);
#endif
		*resid -= written;
		offset += written;
		buf += written;

		ps->ps_destroy_msg(pm);
		pm = NULL;
	} while (*resid != 0);

	/* 
	 * puffs_ops(3) says
	 *  "everything must be written or an error will be generated" 
	 */
	if (*resid != 0)
		error = EFBIG;

	if (ioflag & (IO_SYNC|IO_DSYNC))
		ps->ps_syncwrites++;
	else
		ps->ps_asyncwrites++;

	/*
	 * Remember to sync the file
	 */
	pnd->pnd_flags |= PND_DIRTY;

#ifdef PERFUSE_DEBUG
	if (perfuse_diagflags & PDF_SYNC)
		DPRINTF("%s: DIRTY opc = %p, file = \"%s\"\n", 
			__func__, (void*)opc, perfuse_node_path(opc));
#endif
out:
	if (pm != NULL)
		ps->ps_destroy_msg(pm);

	/*
	 * If there are no more queued write, we can resume
	 * an operation awaiting write completion.
	 */ 
	pnd->pnd_flags &= ~PND_INWRITE;
	if (dequeue_requests(ps, opc, PCQ_WRITE, 1) == 0)
		(void)dequeue_requests(ps, opc, PCQ_AFTERWRITE, DEQUEUE_ALL);

	return error;
}

/* ARGSUSED0 */
void
perfuse_cache_write(pu, opc, size, runs)
	struct puffs_usermount *pu;
	puffs_cookie_t opc;
	size_t size;
	struct puffs_cacherun *runs;
{
	return;
}
