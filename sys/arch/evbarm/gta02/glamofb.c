/*-
 * Copyright (c) 2011, Peter Tworek
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Framebuffer driver SMedia GLAMO
 */

#include <sys/cdefs.h>

#include <sys/bus.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>

#include <dev/wscons/wsdisplayvar.h>
#include <dev/wscons/wsconsio.h>
#include <dev/wsfont/wsfont.h>
#include <dev/rasops/rasops.h>
#include <dev/wscons/wsdisplay_vconsvar.h>

#include <arm/s3c2xx0/s3c24x0var.h>

#include "locators.h"

#ifndef GLAMOFB_WIDTH
#define GLAMOFB_WIDTH	480
#endif /* !GLAMOFB_WIDTH */

#ifndef GLAMOFB_HEIGHT
#define GLAMOFB_HEIGHT	640
#endif /* !GLAMOFB_HEIGHT */

/* GLAMO only supports 16bit depth */
#define GLAMOFB_DEPTH	16

#define GLAMO_FB_ADDR	0x08800000

struct glamofb_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_iot;

	uint32_t		sc_width;
	uint32_t		sc_height;
	uint32_t		sc_stride;

	paddr_t			sc_fbaddr;
	size_t			sc_fbsize;
	void			*sc_fbmem;
	uint32_t 		sc_mode;

	struct wsscreen_descr		sc_defaultscreen_descr;
	const struct wsscreen_descr	*sc_screens[1];
	struct wsscreen_list		sc_screenlist;
	struct vcons_screen		sc_console_screen;
	struct vcons_data		sc_vd;
};

int     glamofb_match(device_t, cfdata_t, void *);
void    glamofb_attach(device_t, device_t, void *);
int     glamofb_ioctl(void *, void *, u_long, void *, int, struct lwp *);
paddr_t glamofb_mmap(void *, void *, off_t, int);
void    glamofb_init_screen(void *, struct vcons_screen *, int, long *);

CFATTACH_DECL_NEW(glamofb, sizeof(struct glamofb_softc),
	glamofb_match, glamofb_attach, NULL, NULL);

struct wsdisplay_accessops glamofb_accessops = {
	glamofb_ioctl,
	glamofb_mmap,
	NULL,	/* alloc_screen */
	NULL,	/* free_screen */
	NULL,	/* show_screen */
	NULL, 	/* load_font */
	NULL,	/* pollc */
	NULL	/* scroll */
};

int
glamofb_match(device_t parent, cfdata_t cf, void *aux)
{
	struct s3c2xx0_attach_args *sa = aux;

	if (sa->sa_addr == SSIOCF_ADDR_DEFAULT)
		sa->sa_addr = GLAMO_FB_ADDR;

	return (1);
}

void
glamofb_attach(device_t parent, device_t self, void *aux)
{
	struct wsemuldisplaydev_attach_args waa;
	struct glamofb_softc		*sc = device_private(self);
	struct s3c2xx0_attach_args	*aa = aux;
	struct rasops_info       	*ri;
	bus_space_handle_t		fbh;
	unsigned long            	defattr;

	sc->sc_dev = self;
	sc->sc_iot = aa->sa_iot;

	sc->sc_width = GLAMOFB_WIDTH;
	sc->sc_height = GLAMOFB_HEIGHT;
	sc->sc_stride = sc->sc_width << 1;
	sc->sc_fbsize = sc->sc_stride * sc->sc_height;
	sc->sc_fbaddr = aa->sa_addr;

	if (bus_space_map(sc->sc_iot, sc->sc_fbaddr, sc->sc_fbsize,
	                  BUS_SPACE_MAP_LINEAR, &fbh)) {
		aprint_error(": couldn't map framebuffer space!\n");
		return;
	}
	sc->sc_fbmem = (void *)bus_space_vaddr(sc->sc_iot, fbh);

	sc->sc_defaultscreen_descr = (struct wsscreen_descr) {
		"default",
		0, 0,
		NULL,
		8, 16,
		WSSCREEN_WSCOLORS | WSSCREEN_HILIT,
		NULL
	};
	sc->sc_screens[0] = &sc->sc_defaultscreen_descr;
	sc->sc_screenlist = (struct wsscreen_list){1, sc->sc_screens};
	sc->sc_mode = WSDISPLAYIO_MODE_EMUL;

	vcons_init(&sc->sc_vd, sc, &sc->sc_defaultscreen_descr, &glamofb_accessops);
	sc->sc_vd.init_screen = glamofb_init_screen;

	ri = &sc->sc_console_screen.scr_ri;

	vcons_init_screen(&sc->sc_vd, &sc->sc_console_screen, 1, &defattr);
	sc->sc_console_screen.scr_flags |= VCONS_SCREEN_IS_STATIC;

	sc->sc_defaultscreen_descr.textops = &ri->ri_ops;
	sc->sc_defaultscreen_descr.capabilities = ri->ri_caps;
	sc->sc_defaultscreen_descr.nrows = ri->ri_rows;
	sc->sc_defaultscreen_descr.ncols = ri->ri_cols;
	wsdisplay_cnattach(&sc->sc_defaultscreen_descr, ri, 0, 0, defattr);
	vcons_replay_msgbuf(&sc->sc_console_screen);

	waa.console = 1;
	waa.scrdata = &sc->sc_screenlist;
	waa.accessops = &glamofb_accessops;
	waa.accesscookie = &sc->sc_vd;

	aprint_normal(": %dx%d@16, fbaddr=%p\n", sc->sc_width,
	              sc->sc_height, sc->sc_fbmem);

	config_found(sc->sc_dev, &waa, wsemuldisplaydevprint);
}

void
glamofb_init_screen(void *cookie, struct vcons_screen *scr, int existing, long *defattr)
{
	struct glamofb_softc	*sc = cookie;
	struct rasops_info 	*ri = &scr->scr_ri;

	ri->ri_width = sc->sc_width;
	ri->ri_height = sc->sc_height;
	ri->ri_depth = 16;
	ri->ri_stride = sc->sc_stride;
	ri->ri_flg = RI_CENTER | RI_FULLCLEAR;

	ri->ri_bits = (char *)sc->sc_fbmem;

	scr->scr_flags |= VCONS_DONT_READ;

	if (existing) {
		ri->ri_flg |= RI_CLEAR;
	}

	rasops_init(ri, sc->sc_height / 8, sc->sc_width / 8);
	ri->ri_caps = WSSCREEN_WSCOLORS;

	rasops_reconfig(ri, sc->sc_height / ri->ri_font->fontheight,
	                sc->sc_width / ri->ri_font->fontwidth);

	ri->ri_hw = scr;
}

int
glamofb_ioctl(void *v, void *vs, u_long cmd, void *data, int flag, struct lwp *l)
{
	struct vcons_data	*vd = v;
	struct glamofb_softc	*sc = vd->cookie;
	struct wsdisplay_fbinfo	*wdf;
	struct vcons_screen	*ms = vd->active;
	int data_int;

	switch (cmd) {

		case WSDISPLAYIO_GTYPE:
			*(u_int *)data = WSDISPLAY_TYPE_PCIMISC;
			return 0;

		case WSDISPLAYIO_GINFO:
			if (ms == NULL)
				return (ENODEV);
			wdf = (void *)data;
			wdf->height = ms->scr_ri.ri_height;
			wdf->width = ms->scr_ri.ri_width;
			wdf->depth = ms->scr_ri.ri_depth;
			wdf->cmsize = 0;
			return 0;

		case WSDISPLAYIO_LINEBYTES:
			*(u_int *)data = sc->sc_stride;
			return 0;

		case WSDISPLAYIO_GMODE:
			*(u_int *)data = sc->sc_mode;
			return 0;

		case WSDISPLAYIO_SMODE:
			data_int = *(int*)data;
			/* notify the bus backend */
			if (data_int != sc->sc_mode) {
				sc->sc_mode = data_int;
				if(data_int == WSDISPLAYIO_MODE_EMUL) {
					vcons_redraw_screen(ms);
				}
			}
			return 0;
#if 0
		case WSDISPLAYIO_GVIDEO:
			*(u_int *)data = sc->sc_enabled ?
				WSDISPLAYIO_VIDEO_ON : WSDISPLAYIO_VIDEO_OFF;
			return 0;

		case WSDISPLAYIO_SVIDEO:
			data_int = *(int*)data;
			if (data_int == WSDISPLAYIO_VIDEO_ON) {
				jzfb_lcdc_enable(sc);
			} else if (data_int == WSDISPLAYIO_VIDEO_OFF) {
				jzfb_lcdc_disable(sc);
			}
			return 0;
#endif
        }
        return EPASSTHROUGH;
}

paddr_t
glamofb_mmap(void *v, void *vs, off_t offset, int prot)
{
	struct vcons_data *vd = v;
	struct glamofb_softc *sc = vd->cookie;

	/* 'regular' framebuffer mmap()ing */
	if (offset < sc->sc_fbsize) {
		return bus_space_mmap(sc->sc_iot, sc->sc_fbaddr + offset,
		                     0, prot, BUS_SPACE_MAP_LINEAR);
	}

	return -1;
}
