#ifndef _MORSE_PAGESET_H_
#define _MORSE_PAGESET_H_

/*
 * Copyright 2017-2022 Morse Micro
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */

#include <linux/skbuff.h>
#include <linux/workqueue.h>
#include <linux/kfifo.h>
#include <linux/types.h>

#include "skbq.h"
#include "chip_if.h"

/*
 * A pageset uses a pair of pagers to implement the paging system
 * for transferring messages and data between the host and chip.
 * This module handles data from network queues on one end and multiple
 * pager interfaces on the other end.
 *
 * Paging works by requesting a page from the chip,
 * filling the page location on chip with data, then passing the page
 * back to the chip through a different pager. The reverse is true for rx.
 *
 * Typically one pageset would be used for chip to host communication and
 * a separate one would be used for host to chip communication.
 */

/* Number of HOST->CHIP pages to reserve for commands and beacons to avoid starvation */
#define CMD_RSVED_PAGES_MAX 2
/** Must be power ^2 and >= CMD_RSVED_PAGES_MAX */
#define CMD_RSVED_KFIFO_LEN 2

/* Number of HOST->CHIP pages to reserve exclusively for commands to avoid starvation */
#define CMD_RSVED_CMD_PAGES_MAX 1

/* Number of HOST->CHIP pages to reserve exclusively for beacons to avoid starvation */
#define CMD_RSVED_BEACON_PAGES_MAX 1

/**
 * Number of CHIP->HOST returned pages to cache in the host to speed up TX
 *
 * Nominally, this should be >= the amount of pages allocated to the
 * FROM_HOST pager
 */
#define CACHED_PAGES_MAX 32
/** Must be power ^2 and >= CACHED_PAGES_MAX */
#define CACHED_PAGES_KFIFO_LEN 32

/**
 * Number of TX queues used to store different priority packets
 *
 * Nominally, this should be equal to the number of QoS queues the chip supports
 *
 */
#define PAGESET_TX_SKBQ_MAX 4

extern const struct chip_if_ops morse_pageset_ops;

struct morse_page {
	u32 addr;	/* Address of page in chip memory */
	u32 size_bytes; /* Number of bytes in the page */
};

struct morse_pager_pkt_memory {
	u32 base_addr;
	u16 page_len;
	u8 page_len_reserved;
	u8 num;
};

struct morse_pageset {
	struct morse *mors;
	struct morse_skbq data_qs[PAGESET_TX_SKBQ_MAX];
	struct morse_skbq beacon_q;
	struct morse_skbq mgmt_q;
	struct morse_skbq cmd_q;
	u8 flags;

	/* @lock: Protects access to the populated and return pagers that make up this pageset */
	struct mutex lock;
	struct morse_pager *populated_pager;
	struct morse_pager *return_pager;

	DECLARE_KFIFO(reserved_pages, struct morse_page, CMD_RSVED_KFIFO_LEN);
	DECLARE_KFIFO(cached_pages, struct morse_page, CACHED_PAGES_KFIFO_LEN);
};

/**
 * Prints info about the pageset instance to a file.
 *
 * @mors: Morse chip instance
 * @pageset: Pointer to pageset struct to print
 * @file: Pointer to file to print to
 */
void morse_pageset_show(struct morse *mors, struct morse_pageset *pageset, struct seq_file *file);

#endif /* !_MORSE_PAGESET_H_ */
