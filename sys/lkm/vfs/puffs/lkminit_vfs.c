/*	$NetBSD: lkminit_vfs.c,v 1.4 2008/04/28 20:24:08 martin Exp $ */

/*-
 * Copyright (c) 1996 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Michael Graff <explorer@flame.org>.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Lubomir Kundrak <lkundrak@skosi.org>.
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
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: lkminit_vfs.c,v 1.4 2008/04/28 20:24:08 martin Exp $");

#include <sys/param.h>
#include <sys/lkm.h>

static int puffs_dispatch_vfs(struct lkm_table *, int, int);
int puffs_lkmentry(struct lkm_table *, int, int);

/*
 * The VFS part of module.
 */
static int
puffs_dispatch_vfs(struct lkm_table *lkmtp, int cmd, int ver)
{
	extern struct vfsops puffs_vfsops;

	/*
	 * declare the filesystem
	 */
	MOD_VFS("puffs", -1, &puffs_vfsops);
	lkmtp->private.lkm_any = (void *) &_module;	/* XXX */

	DISPATCH(lkmtp, cmd, ver, lkm_nofunc, lkm_nofunc, lkm_nofunc);
}

/*
 * entry point
 */
int
puffs_lkmentry(struct lkm_table *lkmtp, int cmd, int ver)
{
	int error = 0;

	switch (cmd) {
	case LKM_E_UNLOAD:
	case LKM_E_LOAD:
		error = puffs_dispatch_vfs(lkmtp, cmd, ver);
		break;
	case LKM_E_STAT:
		error = lkmdispatch(lkmtp, cmd);
		break;
	}
	return error;
}