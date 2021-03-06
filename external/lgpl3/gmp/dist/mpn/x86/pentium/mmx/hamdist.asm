dnl  Intel P55 mpn_hamdist -- mpn hamming distance.

dnl  Copyright 2000, 2002 Free Software Foundation, Inc.
dnl
dnl  This file is part of the GNU MP Library.
dnl
dnl  The GNU MP Library is free software; you can redistribute it and/or
dnl  modify it under the terms of the GNU Lesser General Public License as
dnl  published by the Free Software Foundation; either version 3 of the
dnl  License, or (at your option) any later version.
dnl
dnl  The GNU MP Library is distributed in the hope that it will be useful,
dnl  but WITHOUT ANY WARRANTY; without even the implied warranty of
dnl  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
dnl  Lesser General Public License for more details.
dnl
dnl  You should have received a copy of the GNU Lesser General Public License
dnl  along with the GNU MP Library.  If not, see http://www.gnu.org/licenses/.

include(`../config.m4')


C P55: hamdist 12.0 cycles/limb

C For reference, this code runs at 11.5 cycles/limb for popcount, which is
C slower than the plain integer mpn/x86/pentium/popcount.asm.

MULFUNC_PROLOGUE(mpn_hamdist)
include_mpn(`x86/k6/mmx/popham.asm')
