#	$NetBSD: Makefile,v 1.9 2011/08/11 23:15:36 joerg Exp $

.include <bsd.own.mk>

LIB=		mesa_dri
LIBISMODULE=	yes

SHLIB_MAJOR=	0

MESA_SRC_MODULES=	glapi main math vbo tnl shader swrast slang ss common

CPPFLAGS+=	-I${X11SRCDIR.MesaLib}/src/mesa/main \
		-I${X11SRCDIR.MesaLib}/src/mesa/glapi \
		-I${X11SRCDIR.MesaLib}/src/mesa/shader \
		-I${X11SRCDIR.MesaLib}/src/mesa/drivers/dri/common \
		-I${X11SRCDIR.MesaLib}/src/mesa \
		-I${X11SRCDIR.MesaLib}/include \
		-I${DESTDIR}${X11INCDIR}/X11


CPPFLAGS+=	${X11FLAGS.THREADLIB} \
		-DPTHREADS -D_NETBSD_SOURCE -DUSE_EXTERNAL_DXTN_LIB=1 \
		-DHAVE_ALIAS -DIN_DRI_DRIVER
#CPPFLAGS+=	-DGLX_DIRECT_RENDERING -DGLX_INDIRECT_RENDERING -DHAVE_POSIX_MEMALIGN
CFLAGS+=	-pthread

.include "../../libOSMesa/libmesa.mk"
.include "../../../tools/glsl/Makefile.glslsrcs"

SRCS+=  ${SRCS.pp} ${SRCS.cl}

.include <bsd.x11.mk>

LIBDIR=		${X11USRLIBDIR}/modules/dri

#LIBDPLIBS+=	GL	${.CURDIR}/../../libGL

CWARNFLAGS.clang+=	-Wno-format

.include <bsd.lib.mk>
