/*	$NetBSD: conf.c,v 1.12 2005/12/11 12:17:06 christos Exp $	*/

/*
 * Copyright (c) 1994, 1995 Charles M. Hannum.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by Charles Hannum.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: conf.c,v 1.12 2005/12/11 12:17:06 christos Exp $");

#include <sys/param.h>
#include <sys/conf.h>

#include "scif.h"
#include "pvr.h"
#include "wskbd.h"

#include <dev/cons.h>

#define scifcnpollc	nullcnpollc

#if NPVR > 0
#if NWSKBD > 0
#define pvrcngetc wskbd_cngetc
#else /* NWSKBD > 0 */
static int
pvrcngetc(dev_t dev)
{
	return 0;
}
#endif /* NWSKBD > 0 */

#define pvrcnputc wsdisplay_cnputc
#define	pvrcnpollc nullcnpollc
#endif /* NPVR > 0 */

cons_decl(scif);
cons_decl(pvr);

struct consdev constab[] = {
#if NPVR > 0
	cons_init(pvr),
#endif
#if NSCIF > 0
	cons_init(scif),
#endif
	{ 0 },
};
