#	$NetBSD: Makefile,v 1.6 2011/10/11 13:53:57 joerg Exp $

PROG_CXX=	lli
NOMAN=		yes

.include <bsd.init.mk>

.PATH: ${LLVM_SRCDIR}/tools/lli

SRCS=	lli.cpp

LLVM_LIBS+= \
	X86CodeGen \
	X86TargetInfo \
	X86MCTargetDesc \
	X86AsmPrinter \
	X86Utils \
	AsmPrinter \
	AsmParser \
	MCParser \
	BitReader \
	JIT \
	MCJIT \
	RuntimeDyld \
	Interpreter \
	ExecutionEngine \
	Object \
	SelectionDAG \
	CodeGen \
	ScalarOpts \
	TransformsUtils \
	Analysis \
	Target \
	MC \
	Core \
	Support

.include "${.PARSEDIR}/../../link.mk"

.include <bsd.prog.mk>
