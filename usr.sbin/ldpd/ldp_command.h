/* $NetBSD: ldp_command.h,v 1.1 2010/12/08 07:20:14 kefren Exp $ */

/*-
 * Copyright (c) 2010 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Mihai Chelaru <kefren@NetBSD.org>
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

#ifndef _LDP_COMMAND_H_
#define _LDP_COMMAND_H_

#define	MAX_COMMAND_SOCKETS 64
#define	MAX_COMMAND_SIZE 512

struct com_sock {
	int socket;
	int auth;	/* 1 if socket is authenticated */
};

struct com_func {
	char com[64];
	int (* func)(int, char *);
};

void	init_command_sockets(void);
int	create_command_socket(int);
struct com_sock *	is_command_socket(int);
void	command_accept(int);
int	add_command_socket(int);
void	command_dispatch(struct com_sock *);
void	command_close(int);

void	send_prompt(int);
void	send_pwd_prompt(int);
int	command_match(struct com_func*, int, char*, char*);

/* Main functions */
int	show_func(int, char *);
int	set_func(int, char *);
int	exit_func(int, char *);

/* Show functions */
int	show_neighbours(int, char *);
int	show_bindings(int, char *);
int	show_debug(int, char *);
int	show_hellos(int, char *);
int	show_parameters(int, char *);
int	show_version(int, char *);
int	show_warning(int, char *);

/* Set functions */
int	set_hello_time(int, char *);
int	set_debug(int, char *);
int	set_warning(int, char *);

#endif	/* !_LDP_COMMAND_H_ */