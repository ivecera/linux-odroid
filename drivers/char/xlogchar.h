/* Copyright (C) 2018 XiaoMi, Inc.
 * Copyright (C) 2020 XiaoMi, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef XLOGCHAR_H
#define XLOGCHAR_H

/* xlog alloc 1M bytes buffer used for audio event,
 * and use the ping pong buffer for write and read
 */

#define XLOGBUF_SIZE	(1024 * 1024)
#define XLOGPKG_SIZE	512
#define XLOGPKG_NUM	(XLOGBUF_SIZE / XLOGPKG_SIZE)

struct xlogchar {
	/* State for the char driver */
	struct device		*dev;
	struct cdev		cdev;
	size_t			read_idx;
	size_t			write_idx;
	size_t			used_size;
	struct mutex		lock;
	wait_queue_head_t	wait_q;
	char			*buf;
};

#endif
