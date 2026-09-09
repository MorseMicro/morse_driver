/*
 * Copyright 2021-2023 Morse Micro
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */
#include <linux/gpio.h>
#include "morse.h"
#include "debug.h"
#include "pageset.h"
#include "skb_header.h"
#include "skbq.h"
#include "ps.h"
#include "hw.h"
#include "bus.h"
#include "ipmon.h"
#include "pager_hw.h"
#include "trace.h"

/* Defined as the most number of MPDUs per AMPDU */
#ifndef MAX_PAGES_PER_TX_TXN
#define MAX_PAGES_PER_TX_TXN	16
#endif

/* 2 full AMPDUs (and also more than the number of RX pages in chip) */
#ifndef MAX_PAGES_PER_RX_TXN
#define MAX_PAGES_PER_RX_TXN	32
#endif

/* How frequently to notify the chip when RX pages are returned */
#ifndef PAGE_RETURN_NOTIFY_INT
#define PAGE_RETURN_NOTIFY_INT	4
#endif

/* Time in milliseconds to wait for the beacon tasklet to queue the beacon to skbq */
#define BEACON_TASKLET_WAITQ_TIMEOUT 1

/* Value to use when ACI is not applicable to queue lookup */
#define TX_Q_NO_ACI (-1)

static void pageset_tx_status_irq_enable(struct morse *mors, bool enable)
{
	int ret = morse_hw_irq_enable(mors, MORSE_PAGER_BYPASS_TX_STATUS_IRQ_NUM, enable);

	if (ret)
		MORSE_ERR(mors, "%s: failed (ret:%d)\n", __func__, ret);
}

static void pageset_cmd_resp_irq_enable(struct morse *mors, bool enable)
{
	int ret = morse_hw_irq_enable(mors, MORSE_PAGER_BYPASS_CMD_RESP_IRQ_NUM, enable);

	if (ret)
		MORSE_ERR(mors, "%s: failed (ret:%d)\n", __func__, ret);
}

static void pageset_pager_irq_enable(const struct morse_pager *pager, bool enable)
{
	int ret = morse_hw_irq_enable(pager->mors, pager->id, enable);

	if (ret)
		MORSE_ERR(pager->mors, "%s: [%d] failed (ret:%d)\n", __func__, pager->id, ret);
}

/* Mappings between sk_buff, skbq and pageset */
static inline struct morse_skbq *skbq_pageset_tc_q_from_aci(struct morse *mors, int aci)
{
	struct morse_pageset *pageset = mors->chip_if->to_chip_pageset;

	if (!pageset)
		return NULL;

	if (aci == TX_Q_NO_ACI)
		return NULL;

	if (aci < 0 || aci >= ARRAY_SIZE(pageset->data_qs))
		return NULL;

	return &pageset->data_qs[aci];
}

static inline struct morse_skbq *pageset2cmdq(struct morse_pageset *pageset)
{
	return &pageset->cmd_q;
}

static struct morse_skbq *skbq_pageset_cmd_tc_q(struct morse *mors)
{
	return (mors->chip_if->to_chip_pageset) ? &mors->chip_if->to_chip_pageset->cmd_q : NULL;
}

static struct morse_skbq *skbq_pageset_bcn_tc_q(struct morse *mors)
{
	return (mors->chip_if->to_chip_pageset) ? &mors->chip_if->to_chip_pageset->beacon_q : NULL;
}

static struct morse_skbq *skbq_pageset_mgmt_tc_q(struct morse *mors)
{
	return (mors->chip_if->to_chip_pageset) ? &mors->chip_if->to_chip_pageset->mgmt_q : NULL;
}

static struct morse_skbq *skbq_pageset_get_q(struct morse *mors,
					     enum morse_skb_channel channel,
					     int aci)
{
	switch (channel) {
	case MORSE_SKB_CHAN_DATA:
	case MORSE_SKB_CHAN_WIPHY:
	case MORSE_SKB_CHAN_LOOPBACK:
	case MORSE_SKB_CHAN_DATA_NOACK:
		return skbq_pageset_tc_q_from_aci(mors, aci);
	case MORSE_SKB_CHAN_MGMT:
		return skbq_pageset_mgmt_tc_q(mors);
	case MORSE_SKB_CHAN_BEACON:
		return skbq_pageset_bcn_tc_q(mors);
	case MORSE_SKB_CHAN_COMMAND:
		return mors->chip_if->to_chip_pageset ?
		       pageset2cmdq(mors->chip_if->to_chip_pageset) : NULL;
	default:
		return NULL;
	}
}

static void skbq_pageset_get_tx_qs(struct morse *mors, struct morse_skbq **qs, int *num_qs)
{
	struct morse_pageset *pageset = mors->chip_if->to_chip_pageset;
	*qs = pageset->data_qs;
	*num_qs = PAGESET_TX_SKBQ_MAX;
}

static struct morse_skbq *skbq_pageset_get_rx_data_q(struct morse *mors)
{
	/* On rx, all data frames go through data_q[0] */
	const int rx_data_queue = 0;

	return (mors->chip_if && mors->chip_if->from_chip_pageset) ?
	    &mors->chip_if->from_chip_pageset->data_qs[rx_data_queue] : NULL;
}

static void morse_pageset_flush_cache(struct morse *mors, struct morse_pageset *pageset)
{
	mutex_lock(&pageset->lock);
	kfifo_reset(&pageset->cached_pages);
	kfifo_reset(&pageset->reserved_pages);

	morse_pager_hw_flush_cache(pageset->populated_pager);
	morse_pager_hw_flush_cache(pageset->return_pager);
	mutex_unlock(&pageset->lock);
}

static void pagesets_flush_all_caches(struct morse *mors)
{
	morse_pageset_flush_cache(mors, mors->chip_if->to_chip_pageset);
	morse_pageset_flush_cache(mors, mors->chip_if->from_chip_pageset);
}

static int morse_pageset_irq_handler(struct morse *mors, u32 status)
{
	int count;
	int ret;
	struct morse_pager *pager;
	struct morse_chip_if_state *chip_if = mors->chip_if;
	bool rx_pend = false;
	bool tx_buffer_return_pend = false;
	bool is_tx_status_bypass = false;
	bool is_cmd_resp_bypass = false;

	for (count = 0; count < chip_if->pager_count; count++) {
		if (!(status & MORSE_PAGER_IRQ_MASK(count)))
			continue;

		pager = &chip_if->pagers[count];

		if (pager->flags & MORSE_PAGER_FLAGS_POPULATED)
			rx_pend = true;
		else
			tx_buffer_return_pend = true;
	}

	/* Check the bypass locations for 'RX' pending */
	is_tx_status_bypass = !!(status & MORSE_PAGER_IRQ_BYPASS_TX_STATUS_AVAILABLE);
	is_cmd_resp_bypass = !!(status & MORSE_PAGER_IRQ_BYPASS_CMD_RESP_AVAILABLE);

	if (is_tx_status_bypass && chip_if->bypass.tx_sts.location) {
		u32 page;

		ret = morse_reg32_read(mors, chip_if->bypass.tx_sts.location, &page);
		if (ret == 0) {
			ret = kfifo_put(&chip_if->bypass.tx_sts.to_process, page);
			MORSE_WARN_ON(FEATURE_ID_DEFAULT, ret == 0);
			rx_pend = true;
		}
	}

	if (is_cmd_resp_bypass && chip_if->bypass.cmd_resp.location) {
		u32 page;

		ret = morse_reg32_read(mors, chip_if->bypass.cmd_resp.location, &page);
		if (ret == 0) {
			ret = kfifo_put(&chip_if->bypass.cmd_resp.to_process, page);
			MORSE_WARN_ON(FEATURE_ID_DEFAULT, ret == 0);
			rx_pend = true;
		}
	}

	if (rx_pend || tx_buffer_return_pend) {
		if (rx_pend)
			set_bit(MORSE_RX_PEND, &chip_if->event_flags);

		if (tx_buffer_return_pend)
			set_bit(MORSE_PAGE_RETURN_PEND, &chip_if->event_flags);

		queue_work(mors->chip_wq, &mors->chip_if_work);
	}

	return 0;
}

static bool morse_pageset_page_is_cached(struct morse_pageset *pageset, struct morse_page *page)
{
	int i;
	int n_pages;
	struct morse_page pages[CACHED_PAGES_MAX];

	lockdep_assert_held(&pageset->lock);

	BUILD_BUG_ON(CMD_RSVED_PAGES_MAX > CACHED_PAGES_MAX);
	MORSE_WARN_ON(FEATURE_ID_PAGER, !pageset || !page);
	if (!pageset || !page)
		return false;

	n_pages = kfifo_out_peek(&pageset->reserved_pages, pages, CACHED_PAGES_MAX);
	for (i = 0; i < n_pages; i++) {
		if (pages[i].addr == page->addr)
			return true;
	}

	n_pages = kfifo_out_peek(&pageset->cached_pages, pages, CACHED_PAGES_MAX);
	for (i = 0; i < n_pages; i++) {
		if (pages[i].addr == page->addr)
			return true;
	}

	return false;
}

static void morse_pageset_to_chip_return_handler(struct morse *mors, struct morse_pageset *pageset)
{
	struct morse_pager *pager = pageset->return_pager;
	struct morse_page page;
	int ret;
	bool pager_empty = false;
	unsigned int popped = 0;
	unsigned int max_expected_pops;

	lockdep_assert_held(&pageset->lock);

	/* Continue to pop until either the pager is exhausted or more than
	 * double the amount of possible cache entries have been popped.
	 * (to max bound this loop in the event of hardware failure)
	 */
	max_expected_pops = kfifo_size(&pageset->cached_pages) * 2;

	while (kfifo_len(&pageset->reserved_pages) < CMD_RSVED_PAGES_MAX) {
		if (morse_pager_hw_pop(pager, &page)) {
			pager_empty = true;
			break;
		}

		popped++;
		if (morse_pageset_page_is_cached(pageset, &page))
			continue;

		ret = kfifo_put(&pageset->reserved_pages, page);
		MORSE_WARN_ON(FEATURE_ID_PAGER, !ret);
	}

	if (pager_empty)
		goto exit;

	while (popped < max_expected_pops) {
		if (morse_pager_hw_pop(pager, &page))
			break;

		popped++;
		if (morse_pageset_page_is_cached(pageset, &page))
			continue;

		ret = kfifo_put(&pageset->cached_pages, page);
		MORSE_WARN_ON(FEATURE_ID_PAGER, !ret);
	}

	if (popped >= max_expected_pops)
		MORSE_DBG(mors, "%s: more page returns than expected\n", __func__);

exit:
	trace_pagesets_tx_buffers_avail(popped);
	if (popped)
		morse_pager_hw_notify(pager);
}

static void tx_page_unavailable_for_channel(struct morse *mors, enum morse_skb_channel channel)
{
	if (channel == MORSE_SKB_CHAN_BEACON)
		mors->debug.page_stats.bcn_no_page++;
	else if (channel == MORSE_SKB_CHAN_COMMAND)
		mors->debug.page_stats.cmd_no_page++;
}

static int tx_page_get_for_channel(struct morse_pageset *pageset,
				   enum morse_skb_channel channel,
				   struct morse_page *page)
{
	int ret;

	lockdep_assert_held(&pageset->lock);

	if (channel == MORSE_SKB_CHAN_BEACON || channel == MORSE_SKB_CHAN_COMMAND) {
		if (kfifo_get(&pageset->reserved_pages, page) > 0)
			return 0;
	}

	ret = kfifo_get(&pageset->cached_pages, page);

	return ret == 0 ? -ENOMEM : 0;
}

static uint tx_pages_available_for_channel(struct morse_pageset *pageset,
					  enum morse_skb_channel channel)
{
	int available;
	int reserved;
	int any;

	lockdep_assert_held(&pageset->lock);

	any = kfifo_len(&pageset->cached_pages);
	reserved = kfifo_len(&pageset->reserved_pages);

	if (channel == MORSE_SKB_CHAN_BEACON) {
		/* A backlog of beacons should be cleared ASAP. They need to make
		 * the round trip to the HW and back to keep the beacon SM in sync.
		 */
		available = any + reserved;
	} else if (channel == MORSE_SKB_CHAN_COMMAND) {
		/* The driver ensures CMDs are serialized (one in flight at a time). At
		 * least one page must be kept in reserve for beacon channel
		 * servicing.
		 */
		available = min_t(int, 1, (any + reserved) - 1);
	} else {
		/* Artificially reduce the number of pages available for other channels
		 * if the reserved pool is not holding the maximum it should be.
		 * Channels that use the reserved pool will also take from the other pool when
		 * empty.
		 */
		MORSE_WARN_ON(FEATURE_ID_PAGER, reserved > CMD_RSVED_PAGES_MAX);
		any -= (CMD_RSVED_PAGES_MAX - reserved);
		available = min_t(int, MAX_PAGES_PER_TX_TXN, any);
	}

	return max(available, 0);
}

static int morse_pageset_write(struct morse_pageset *pageset,
			       enum morse_skb_channel channel,
			       struct sk_buff *skb)
{
	int ret = 0;
	struct morse *mors = pageset->mors;
	struct morse_pager *populated_pager = pageset->populated_pager;
	struct morse_page page;
	struct morse_buff_skb_header *hdr = (struct morse_buff_skb_header *)skb->data;
	size_t end_of_skb_pad = (skb->len & 0x03) ? (4 - (unsigned long)(skb->len & 3)) : 0;
	size_t write_len = skb->len + end_of_skb_pad;

	lockdep_assert_held(&pageset->lock);

	if (tx_page_get_for_channel(pageset, channel, &page)) {
		MORSE_ERR(mors, "%s no pages available\n", __func__);
		return -ENOSPC;
	}

	if (write_len > page.size_bytes) {
		MORSE_ERR(mors, "%s Data larger than pagesize: [%d:%d]\n",
			  __func__, skb->len, page.size_bytes);
		return -ENOSPC;
	}

	if (write_len > (skb->len + skb_tailroom(skb))) {
		/* SKB should be big enough to copy from */
		MORSE_WARN_ON(FEATURE_ID_SKB, 1);
		return -ENOSPC;
	}

	morse_debug_fw_hostif_log_record(mors, true, skb, hdr);

	ret = morse_pager_hw_page_write(populated_pager, &page, 0, skb->data, write_len);
	if (ret) {
		MORSE_ERR(mors, "%s failed to write page: %d\n", __func__, ret);
		/* Put the page back into the cache */
		kfifo_put(&pageset->cached_pages, page);
		return ret;
	}

	/* Put the filled page to send it to the chip */
	ret = morse_pager_hw_put(populated_pager, &page);
	if (ret) {
		MORSE_ERR(mors, "%s failed to return page: %d\n", __func__, ret);
		/* Return page to avoid page leak.
		 * Write sync word as 0 so the chip discards it.
		 * Don't not putting this in the return pager to avoid
		 * reading and writing from the same pager, as this would require
		 * additional synchronisation.
		 */
		hdr->sync = 0;
		morse_pager_hw_page_write(populated_pager,
					  &page,
					  0,
					  (const char *)hdr,
					  sizeof(*hdr));
		morse_pager_hw_put(populated_pager, &page);
		return ret;
	}

	return ret;
}

static int morse_pageset_read(struct morse_pageset *pageset, struct sk_buff_head *rx_list)
{
	int ret = 0;
	struct morse *mors = pageset->mors;
	struct sk_buff *skb = NULL;
	struct morse_pager *return_pager = pageset->return_pager;
	struct morse_pager *populated_pager = pageset->populated_pager;
	struct morse_chip_if_state *chip_if = mors->chip_if;
	struct morse_page page = { .addr = 0, .size_bytes = 0 };
	struct morse_buff_skb_header *hdr;
	int skb_len;
	int max_checksum_rounds = 2;
	int count = 0;
	bool checksum_valid = !(mors->chip_if->validate_skb_checksum);

	lockdep_assert_held(&pageset->lock);

	/* Preference CMD responses, then TX statuses from the pager bypass
	 * locations before processing other RX. We treat these page types as
	 * 'first-class' citizens.
	 */
	if (kfifo_len(&chip_if->bypass.cmd_resp.to_process) > 0) {
		ret = kfifo_get(&chip_if->bypass.cmd_resp.to_process, &page.addr);
		WARN_ON(ret == 0);
		page.size_bytes = populated_pager->page_size_bytes;
	} else if (kfifo_len(&chip_if->bypass.tx_sts.to_process) > 0) {
		ret = kfifo_get(&chip_if->bypass.tx_sts.to_process, &page.addr);
		WARN_ON(ret == 0);
		page.size_bytes = populated_pager->page_size_bytes;
	} else {
		/* Pop one page from pager */
		ret = morse_pager_hw_pop(populated_pager, &page);
		if (ret) {
			/* No pages left */
			page.addr = 0;
			ret = -ENOENT;
			goto exit;
		}
	}

	skb_len = round_up(page.addr >> 20, 4);
	page.addr = ((page.addr & 0xFFFFF) | mors->cfg->regs->pager_base_address);

	/* Allocate an skb for the page data, copy header to it */
	skb = dev_alloc_skb(skb_len);
	if (!skb) {
		ret = -ENOMEM;
		goto exit;
	}
	skb_put(skb, skb_len);

	/* Read page data */
	ret = morse_pager_hw_page_read(populated_pager, &page, 0, skb->data, skb_len);
	if (ret) {
		MORSE_ERR(mors, "%s failed to read page: %d\n", __func__, ret);
		/* Error is considered catastrophic, pass up error to stop
		 * more page pops.
		 */
		goto exit;
	}

	hdr = (struct morse_buff_skb_header *)skb->data;

	morse_debug_fw_hostif_log_record(mors, false, skb, hdr);

	/* Validate header */
	if (hdr->sync != MORSE_SKB_HEADER_SYNC) {
		bool chip_owned = (hdr->sync == MORSE_SKB_HEADER_CHIP_OWNED_SYNC);

		MORSE_DBG(mors, "%s sync error:0x%02X page[addr:0x%08x len:%d]\n",
			  __func__, hdr->sync, page.addr, hdr->len);

		if (chip_owned) {
			/* Chip already owns the page, clear page address
			 * to indicate that it should not be returned
			 */
			mors->debug.page_stats.page_owned_by_chip++;
			page.addr = 0;
		}

		/* Not considered catastrophic, continue to read pages out of the
		 * pager.
		 */
		ret = -EAGAIN;
		goto exit;
	}

	while (!checksum_valid && count < max_checksum_rounds) {
		checksum_valid = morse_validate_skb_checksum(skb->data);
		if (checksum_valid)
			break;

		mors->debug.page_stats.invalid_checksum++;
		/* Read tx status again if the first read is corrupted. There is a tput degradation
		 * if continue to read pages from the pager.
		 */
		if (hdr->channel != MORSE_SKB_CHAN_TX_STATUS)
			break;
		ret = morse_pager_hw_page_read(populated_pager, &page, 0, skb->data, skb_len);
		if (ret)
			break;
		count++;
	}

	if (!checksum_valid) {
		MORSE_DBG(mors,
			  "%s: SKB checksum is invalid, page:[a:0x%08x len:%d] hdr:[c:%02X s:%02X]",
			  __func__, page.addr, skb_len, hdr->channel, hdr->sync);
		if (hdr->channel == MORSE_SKB_CHAN_TX_STATUS)
			mors->debug.page_stats.invalid_tx_status_checksum++;
		ret = -EAGAIN;
		goto exit;
	}

	/* SW-3875: seems like sdio read can sometimes go wrong and read first 4-bytes word twice,
	 * overwriting second word (hence, offset will be overwritten with 'sync' byte). Anyway, we
	 * should not expect offset value to be larger than word alignment (max 3 bytes)
	 */
	if (hdr->offset > 3) {
		MORSE_ERR(mors,
			  "%s: corrupted skb header offset [offset=%u], hdr.len %d, page addr: 0x%08x\n",
			  __func__, hdr->offset, hdr->len, page.addr);

		/* Should we actually do that, or just fail the page and go to exit_return_page? */
		hdr->offset = (le16_to_cpu(hdr->len) & 0x03) ?
			(4 - (unsigned long)(le16_to_cpu(hdr->len) & 3)) : 0;
	}

	/* Read of page can be greater than actual size of data - so trim */
	skb_len = sizeof(*hdr) + hdr->offset + le16_to_cpu(hdr->len);
	skb_trim(skb, skb_len);

#ifdef CONFIG_MORSE_IPMON
	if (hdr->channel == MORSE_SKB_CHAN_DATA) {
		static u64 time_start;

		morse_ipmon(&time_start, skb, skb->data + sizeof(*hdr),
			    le16_to_cpu(hdr->len), IPMON_LOC_SERVER_DRV, 0);
	}
#endif

	/* Ownership transferred to rx_list; routing and mq enqueue happen
	 * after the read loop completes.
	 */
	__skb_queue_tail(rx_list, skb);
	skb = NULL;

exit:
	/* If the SKB did not successfully make it into the rx_list, it must be freed */
	dev_kfree_skb(skb);

	if (page.addr) {
		/* Put the emptied page to send it back to the chip */
		ret = morse_pager_hw_put(return_pager, &page);
		if (ret)
			MORSE_ERR(mors, "%s: return page failed: %d\n", __func__, ret);
	}

	return ret;
}

/* Returns: less than zero (error), 0 (all tx transmitted), greater than zero (more to tx) */
static int morse_pageset_tx(struct morse_pageset *pageset, enum morse_skb_channel channel, int aci)
{
	int n_pages_avail;
	int n_to_tx;
	int n_to_dequeue_and_tx;
	struct sk_buff_head skbq_to_send;
	struct sk_buff_head skbq_sent;
	struct sk_buff_head skbq_failed;
	struct sk_buff *pfirst, *pnext;
	struct morse *mors = pageset->mors;
	struct morse_buff_skb_header *hdr;
	struct morse_skbq *mq;
	uint count = 0;

	lockdep_assert_held(&pageset->lock);

	mq = skbq_pageset_get_q(mors, channel, aci);
	if (!mq)
		return -EPERM;

	n_to_tx = morse_skbq_count(mq);
	if (!n_to_tx)
		return 0; /* Nothing to transmit from this mq */

	n_pages_avail = tx_pages_available_for_channel(pageset, channel);
	if (!n_pages_avail) {
		tx_page_unavailable_for_channel(mors, channel);
		return -ENOMEM; /* No pages to transmit at all! */
	}

	__skb_queue_head_init(&skbq_to_send);
	__skb_queue_head_init(&skbq_sent);
	__skb_queue_head_init(&skbq_failed);

	if (channel == MORSE_SKB_CHAN_COMMAND) {
		/* Purge timed-out commands (this should not happen) */
		morse_skbq_purge(mq, &mq->pending);
	} else if (channel == MORSE_SKB_CHAN_MGMT && n_to_tx) {
		/* Purge old mgmt frames that have not been sent due to congestion */
		morse_skbq_purge_aged(mors, mq);
	}

	n_to_dequeue_and_tx = morse_skbq_deq_num_items(mq, &skbq_to_send, n_pages_avail);
	skb_queue_walk_safe(&skbq_to_send, pfirst, pnext) {
		int ret;

		if (n_pages_avail) {
			ret = morse_pageset_write(pageset, channel, pfirst);
		} else {
			mors->debug.page_stats.no_page++;
			MORSE_ERR(mors, "%s no pages available\n", __func__);
			ret = -ENOMEM;
		}

		if (ret == 0) {
			hdr = (struct morse_buff_skb_header *)pfirst->data;
			switch (hdr->channel) {
			case MORSE_SKB_CHAN_COMMAND:
				mors->debug.page_stats.cmd_tx++;
				break;
			case MORSE_SKB_CHAN_BEACON:
				mors->debug.page_stats.bcn_tx++;
				break;
			case MORSE_SKB_CHAN_MGMT:
				mors->debug.page_stats.mgmt_tx++;
				break;
			default:
				mors->debug.page_stats.data_tx++;
				break;
			}
			n_pages_avail--;
			__skb_unlink(pfirst, &skbq_to_send);
			__skb_queue_tail(&skbq_sent, pfirst);
			count++;
		} else {
			__skb_unlink(pfirst, &skbq_to_send);
			__skb_queue_tail(&skbq_failed, pfirst);
		}
	}

	if (skbq_failed.qlen > 0) {
		mors->debug.page_stats.write_fail += skbq_failed.qlen;
		MORSE_ERR(mors,
			  "%s: could not write %d pkts - dequeued=%d transmitted=%d remaining=%d\n",
			  __func__, skbq_failed.qlen, n_to_dequeue_and_tx, count, n_pages_avail);
		morse_skbq_purge(NULL, &skbq_failed);
	}

	if (skbq_sent.qlen > 0)
		morse_skbq_tx_complete(mq, &skbq_sent);

	trace_pagesets_tx(channel, count);
	return (n_to_tx - n_to_dequeue_and_tx);
}

/* Returns true if there are TX data pages waiting to be sent */
static bool pageset_tx_from_data_qs(struct morse *mors)
{
	s16 aci;
	uint remaining = 0;
	struct morse_pageset *pageset = mors->chip_if->to_chip_pageset;
	unsigned long stop_mask = BIT(MORSE_TX_BEACON_PEND);
	unsigned long flags;

	mutex_lock(&pageset->lock);
	for (aci = MORSE_ACI_VO; aci >= 0; aci--) {
		int ret;

		if (!morse_is_data_tx_allowed(mors))
			break;

		/* Check for other high-priority tx traffic */
		flags = READ_ONCE(mors->chip_if->event_flags);
		if (flags & stop_mask) {
			/* +1 to signal that there may be more data left */
			remaining += 1;
			break;
		}

		ret = morse_pageset_tx(pageset, MORSE_SKB_CHAN_DATA, aci);
		if (ret > 0) {
			remaining += ret;
		} else if (ret == -ENOMEM) {
			/* +1 to signal that there is more data left */
			remaining += 1;
			break;
		}
	}

	morse_pager_hw_notify(pageset->populated_pager);
	mutex_unlock(&pageset->lock);

	trace_pagesets_tx_remaining(MORSE_SKB_CHAN_DATA, remaining);

	/* Data has potentially been transmitted from the data SKBQs.
	 * If the mac80211 TX data Qs were previously stopped,
	 * now would be a good time to check if they can be started again.
	 */
	morse_skbq_may_wake_tx_queues(mors);
	if (mors->custom_configs.enable_airtime_fairness &&
	    !test_bit(MORSE_STATE_FLAG_DATA_QS_STOPPED, &mors->state_flags))
		tasklet_schedule(&mors->tasklet_txq);

	return (remaining > 0) && morse_is_data_tx_allowed(mors);
}

/* Returns true if there is more of this channel type pending after transmit */
static bool pageset_tx_from_skb_channel(struct morse *mors, enum morse_skb_channel channel)
{
	bool more_tx;

	if (channel == MORSE_SKB_CHAN_DATA) {
		/* data channel is handled differently to other cases */
		more_tx = pageset_tx_from_data_qs(mors);
	} else {
		int ret;
		uint remaining = 0;
		struct morse_pageset *pageset = mors->chip_if->to_chip_pageset;

		mutex_lock(&pageset->lock);
		ret = morse_pageset_tx(pageset, channel, TX_Q_NO_ACI);
		morse_pager_hw_notify(pageset->populated_pager);
		mutex_unlock(&pageset->lock);

		if (ret > 0)
			remaining += ret;
		else if (ret == -ENOMEM)
			remaining += 1;

		trace_pagesets_tx_remaining(channel, remaining);
		more_tx = (remaining > 0);
	}

	return more_tx;
}

static void pageset_route_rx_skbs(struct morse *mors,
				  struct sk_buff_head *rx_list,
				  bool *rxd_cmd,
				  bool *rxd_data,
				  bool *rxd_tx_sts)
{
	struct sk_buff_head cmd_list;
	struct sk_buff_head data_list;
	struct sk_buff *skb, *next;
	int leftover;

	__skb_queue_head_init(&cmd_list);
	__skb_queue_head_init(&data_list);

	skb_queue_walk_safe(rx_list, skb, next) {
		struct morse_buff_skb_header *hdr =
			(struct morse_buff_skb_header *)skb->data;

		__skb_unlink(skb, rx_list);

		switch (hdr->channel) {
		case MORSE_SKB_CHAN_COMMAND:
			*rxd_cmd = true;
			__skb_queue_tail(&cmd_list, skb);
			break;
		case MORSE_SKB_CHAN_TX_STATUS:
			*rxd_tx_sts = true;
			__skb_queue_tail(&data_list, skb);
			break;
		case MORSE_SKB_CHAN_DATA:
		case MORSE_SKB_CHAN_NDP_FRAMES:
		case MORSE_SKB_CHAN_DATA_NOACK:
		case MORSE_SKB_CHAN_BEACON:
		case MORSE_SKB_CHAN_MGMT:
		case MORSE_SKB_CHAN_LOOPBACK:
			*rxd_data = true;
			__skb_queue_tail(&data_list, skb);
			break;
		default:
			MORSE_ERR(mors, "%s: unknown channel %d\n",
				  __func__, hdr->channel);
			dev_kfree_skb_any(skb);
			break;
		}
	}

	if (!skb_queue_empty(&cmd_list))
		morse_skbq_enq(pageset2cmdq(mors->chip_if->from_chip_pageset), &cmd_list);

	if (!skb_queue_empty(&data_list))
		morse_skbq_enq(skbq_pageset_get_rx_data_q(mors), &data_list);

	leftover = skb_queue_len(&cmd_list);
	if (leftover) {
		MORSE_ERR(mors, "%s: Failed to enqueue %d RX CMD SKBs\n", __func__, leftover);
		trace_pagesets_rx_skbq_enqueue_fail(MORSE_SKB_CHAN_COMMAND, leftover);
		morse_skbq_purge(NULL, &cmd_list);
	}

	leftover = skb_queue_len(&data_list);
	if (leftover) {
		MORSE_ERR(mors, "%s: Failed to enqueue %d RX data SKBs\n", __func__, leftover);
		trace_pagesets_rx_skbq_enqueue_fail(MORSE_SKB_CHAN_DATA, leftover);
		morse_skbq_purge(NULL, &data_list);
	}
}

/* Returns true if there are populated RX pages left in the device */
static bool morse_pageset_rx_handler(struct morse *mors,
				     bool *rxd_data,
				     bool *rxd_cmd,
				     bool *rxd_tx_sts)
{
	int ret = 0;
	int count = 0;
	bool return_notify_req = false;
	bool more_rx;
	struct morse_pageset *pageset = mors->chip_if->from_chip_pageset;
	bool break_early = false;
	unsigned long immediate_stop_mask = BIT(MORSE_TX_BEACON_PEND);
	struct sk_buff_head rx_list;

	__skb_queue_head_init(&rx_list);

	mutex_lock(&pageset->lock);

	do {
		unsigned long flags;

		ret = morse_pageset_read(pageset, &rx_list);
		return_notify_req = true;

		if (ret == 0)
			count++;

		if (ret == -EAGAIN)
			ret = 0;

		if ((count % PAGE_RETURN_NOTIFY_INT) == 0) {
			morse_pager_hw_notify(pageset->return_pager);
			return_notify_req = false;
		}

		flags = READ_ONCE(mors->chip_if->event_flags);
		break_early |= !!(flags & immediate_stop_mask);
		break_early |= (count >= MAX_PAGES_PER_RX_TXN);
	} while (ret == 0 && !break_early);

	trace_pagesets_rx(count);
	if (return_notify_req)
		morse_pager_hw_notify(pageset->return_pager);

	more_rx = (ret == -ENOMEM ||
		   break_early ||
		   kfifo_len(&pageset->mors->chip_if->bypass.tx_sts.to_process) ||
		   kfifo_len(&pageset->mors->chip_if->bypass.cmd_resp.to_process));

	mutex_unlock(&pageset->lock);

	pageset_route_rx_skbs(mors, &rx_list, rxd_cmd, rxd_data, rxd_tx_sts);
	if (*rxd_cmd)
		queue_work(mors->net_wq, &pageset2cmdq(pageset)->dispatch_work);

	if (*rxd_data || *rxd_tx_sts)
		queue_work(mors->net_wq, &skbq_pageset_get_rx_data_q(mors)->dispatch_work);

	return more_rx;
}

static int pagesets_get_tx_buffered_count(struct morse *mors)
{
	int i = 0;
	int count = 0;
	struct morse_pageset *tx_pageset;

	if (!mors->chip_if || !mors->chip_if->to_chip_pageset)
		return 0;

	tx_pageset = mors->chip_if->to_chip_pageset;
	if (!(tx_pageset->flags & MORSE_CHIP_IF_FLAGS_DIR_TO_CHIP)) {
		MORSE_WARN_ON_ONCE(FEATURE_ID_PAGER, 1);
		return 0;
	}

	count += tx_pageset->beacon_q.skbq.qlen + tx_pageset->beacon_q.pending.qlen;
	count += tx_pageset->mgmt_q.skbq.qlen + tx_pageset->mgmt_q.pending.qlen;
	count += tx_pageset->cmd_q.skbq.qlen + tx_pageset->cmd_q.pending.qlen;

	for (i = 0; i < ARRAY_SIZE(tx_pageset->data_qs); i++)
		count += morse_skbq_count_tx_ready(&tx_pageset->data_qs[i]) +
		    tx_pageset->data_qs[i].pending.qlen;

	return count;
}

static void morse_pagesets_stale_tx_work(struct work_struct *work)
{
	int i;
	int flushed = 0;
	struct morse *mors = container_of(work, struct morse, tx_stale_work);
	struct morse_pageset *tx_pageset;

	if (!mors->chip_if || !mors->chip_if->to_chip_pageset || !mors->stale_status.enabled)
		return;

	tx_pageset = mors->chip_if->to_chip_pageset;

	mutex_lock(&tx_pageset->lock);
	flushed += morse_skbq_check_for_stale_tx(mors, &tx_pageset->beacon_q);
	flushed += morse_skbq_check_for_stale_tx(mors, &tx_pageset->mgmt_q);
	for (i = 0; i < ARRAY_SIZE(tx_pageset->data_qs); i++)
		flushed += morse_skbq_check_for_stale_tx(mors, &tx_pageset->data_qs[i]);
	mutex_unlock(&tx_pageset->lock);

	if (flushed) {
		trace_pagesets_stale_tx_flushed(flushed);
		MORSE_DBG(mors, "%s: Flushed %d stale TX SKBs\n", __func__, flushed);

		if (pagesets_get_tx_buffered_count(mors) == 0) {
			/* Evaluate ps to check if it was gated on a stale tx status */
			morse_ps_queue_eval(mors);
		}
	}

	/* Re-arm if frames still pending - the timer was not re-armed by tx_complete because it
	 * was already running when those frames arrived, so we must schedule the next check here.
	 */
	if (mors->cfg->ops->skbq_get_tx_status_pending_count(mors) > 0) {
		spin_lock_bh(&mors->stale_status.lock);
		if (mors->stale_status.enabled)
			mod_timer(&mors->stale_status.timer,
				  jiffies + msecs_to_jiffies(morse_skbq_tx_status_lifetime_ms()));
		spin_unlock_bh(&mors->stale_status.lock);
	}
}

static void morse_pagesets_work(struct work_struct *work)
{
	struct morse *mors = container_of(work, struct morse, chip_if_work);
	int ps_bus_timeout_ms = 0;
	bool rxd_data = false;
	bool rxd_cmd = false;
	bool rxd_tx_sts = false;
	unsigned long flags_on_entry;
	unsigned long flags_on_exit = 0;
	unsigned long *flags = &mors->chip_if->event_flags;

	flags_on_entry = READ_ONCE(mors->chip_if->event_flags);
	trace_pagesets_work_enter(flags_on_entry);
	if (!flags_on_entry)
		goto exit;

	/* Don't attempt to interact with device once it becomes unresponsive */
	if (!morse_hw_is_on(mors))
		goto exit;

	/* Once the system starts suspend process, prevent chip communication */
	if (test_bit(MORSE_STATE_FLAG_SYSTEM_IN_SUSPEND, &mors->state_flags))
		goto exit;

	/* Ensure chip is awake while communicating with it */
	morse_ps_wakers_inc(mors);
	morse_ps_force_eval(mors);

	if (test_and_clear_bit(MORSE_TX_BEACON_PEND, flags)) {
		struct morse_pageset *pageset = mors->chip_if->to_chip_pageset;

		mutex_lock(&pageset->lock);

		/* Attempt to refill the TX cache if there are no pages available for beacon TX */
		if (!tx_pages_available_for_channel(pageset, MORSE_SKB_CHAN_BEACON) &&
		    test_and_clear_bit(MORSE_PAGE_RETURN_PEND, flags))
			morse_pageset_to_chip_return_handler(mors, pageset);

		mutex_unlock(&pageset->lock);

		if (pageset_tx_from_skb_channel(mors, MORSE_SKB_CHAN_BEACON))
			set_bit(MORSE_TX_BEACON_PEND, flags);
	}

	/* Prior to processing the remaining TX channel types, first service RX to
	 * get free RX buffers back into the chip ASAP.
	 */
	if (test_and_clear_bit(MORSE_RX_PEND, flags)) {
		if (morse_pageset_rx_handler(mors, &rxd_data, &rxd_cmd, &rxd_tx_sts))
			set_bit(MORSE_RX_PEND, flags);
	}

	if (test_and_clear_bit(MORSE_PAGE_RETURN_PEND, flags)) {
		struct morse_pageset *pageset = mors->chip_if->to_chip_pageset;

		mutex_lock(&pageset->lock);
		morse_pageset_to_chip_return_handler(mors, pageset);
		mutex_unlock(&pageset->lock);
	}

	/* TX any commands before anything else */
	if (test_and_clear_bit(MORSE_TX_COMMAND_PEND, flags)) {
		if (pageset_tx_from_skb_channel(mors, MORSE_SKB_CHAN_COMMAND))
			set_bit(MORSE_TX_COMMAND_PEND, flags);
	}

	/* TX mgmt before considering data */
	if (test_and_clear_bit(MORSE_TX_MGMT_PEND, flags)) {
		if (pageset_tx_from_skb_channel(mors, MORSE_SKB_CHAN_MGMT))
			set_bit(MORSE_TX_MGMT_PEND, flags);
	}

	/* Pause TX data Qs */
	if (test_and_clear_bit(MORSE_DATA_TRAFFIC_PAUSE_PEND, flags)) {
		if (test_and_clear_bit(MORSE_DATA_TRAFFIC_RESUME_PEND, flags))
			MORSE_ERR_RATELIMITED(mors,
					      "Latency to handle traffic pause is too great\n");
		else
			morse_skbq_data_traffic_pause(mors);
	}

	/* Resume TX data Qs  */
	if (test_and_clear_bit(MORSE_DATA_TRAFFIC_RESUME_PEND, flags)) {
		if (test_bit(MORSE_DATA_TRAFFIC_PAUSE_PEND, flags))
			MORSE_ERR_RATELIMITED(mors,
					      "Latency to handle traffic resume is too great\n");

		morse_skbq_data_traffic_resume(mors);
	}

	/* Finally TX any data */
	if (test_and_clear_bit(MORSE_TX_DATA_PEND, flags)) {
		if (pageset_tx_from_skb_channel(mors, MORSE_SKB_CHAN_DATA))
			set_bit(MORSE_TX_DATA_PEND, flags);
	}

	/* Calculate timeout to disable the shared bus with the chip. For any TX that was
	 * pushed down, or non-CMD RX that came up from the chip - increase the timeout as
	 * 'network activity' was seen.
	 */
	if (test_bit(MORSE_TX_DATA_PEND, &flags_on_entry) ||
	    test_bit(MORSE_TX_MGMT_PEND, &flags_on_entry) ||
	    rxd_data)
		ps_bus_timeout_ms = max(ps_bus_timeout_ms, mors_ps_get_net_timeout_ms(mors));

	if (test_and_clear_bit(MORSE_UPDATE_HW_CLOCK_REFERENCE, flags)) {
		morse_claim_bus(mors);
		morse_hw_clock_update(mors);
		morse_release_bus(mors);
	}

	if (ps_bus_timeout_ms)
		morse_ps_bus_activity(mors, ps_bus_timeout_ms);

	morse_ps_wakers_dec(mors);
	morse_ps_force_eval(mors);

	flags_on_exit = READ_ONCE(mors->chip_if->event_flags);
	if (flags_on_exit)
		queue_work(mors->chip_wq, &mors->chip_if_work);

exit:
	trace_pagesets_work_exit(flags_on_exit);
}

void morse_pageset_show(struct morse *mors, struct morse_pageset *pageset, struct seq_file *file)
{
	int i;

	mutex_lock(&pageset->lock);
	seq_printf(file, "flags:0x%01x reserved=%d cached=%d\n",
		   pageset->flags,
		   kfifo_len(&pageset->reserved_pages), kfifo_len(&pageset->cached_pages));
	seq_printf(file, "flags:0x%01x\n", pageset->populated_pager->flags);
	seq_printf(file, "flags:0x%01x\n", pageset->return_pager->flags);

	for (i = 0; i < ARRAY_SIZE(pageset->data_qs); i++)
		morse_skbq_show(&pageset->data_qs[i], file);
	morse_skbq_show(&pageset->mgmt_q, file);
	morse_skbq_show(&pageset->beacon_q, file);
	morse_skbq_show(&pageset->cmd_q, file);
	mutex_unlock(&pageset->lock);
}

static int pageset_init(struct morse *mors,
			struct morse_pageset *pageset,
			u8 flags,
			struct morse_pager *populated_pager,
			struct morse_pager *return_pager)
{
	int i;
	u16 chip_if_direction_flag;

	pageset->mors = mors;
	pageset->flags = flags;
	pageset->populated_pager = populated_pager;
	pageset->return_pager = return_pager;
	mors->chip_if->active_chip_if = MORSE_CHIP_IF_PAGESET;

	mutex_init(&pageset->lock);
	INIT_KFIFO(pageset->reserved_pages);
	INIT_KFIFO(pageset->cached_pages);

	chip_if_direction_flag = pageset->flags & MORSE_PAGER_FLAGS_DIR_TO_HOST ?
				 MORSE_CHIP_IF_FLAGS_DIR_TO_HOST :
				 MORSE_CHIP_IF_FLAGS_DIR_TO_CHIP;

	if (pageset->flags & MORSE_CHIP_IF_FLAGS_DATA) {
		morse_skbq_init(mors,
				&pageset->beacon_q,
				MORSE_CHIP_IF_FLAGS_DATA | chip_if_direction_flag);

		morse_skbq_init(mors,
				&pageset->mgmt_q,
				MORSE_CHIP_IF_FLAGS_DATA | chip_if_direction_flag);

		for (i = 0; i < ARRAY_SIZE(pageset->data_qs); i++)
			morse_skbq_init(mors,
					&pageset->data_qs[i],
					MORSE_CHIP_IF_FLAGS_DATA | chip_if_direction_flag);
	}

	if (pageset->flags & MORSE_CHIP_IF_FLAGS_COMMAND)
		morse_skbq_init(mors,
				&pageset->cmd_q,
				MORSE_CHIP_IF_FLAGS_COMMAND | chip_if_direction_flag);

	populated_pager->parent = pageset;
	return_pager->parent = pageset;
	return 0;
}

static void pageset_finish(struct morse_pageset *pageset)
{
	int i;

	pageset->return_pager = NULL;
	pageset->populated_pager = NULL;

	if (pageset->flags & MORSE_CHIP_IF_FLAGS_DATA) {
		morse_skbq_finish(&pageset->beacon_q);
		morse_skbq_finish(&pageset->mgmt_q);
		for (i = 0; i < ARRAY_SIZE(pageset->data_qs); i++)
			morse_skbq_finish(&pageset->data_qs[i]);
	}

	if (pageset->flags & MORSE_CHIP_IF_FLAGS_COMMAND)
		morse_skbq_finish(&pageset->cmd_q);
}

static int pagesets_get_tx_status_pending_count(struct morse *mors)
{
	int i = 0;
	int count = 0;
	struct morse_pageset *tx_pageset;

	if (!mors->chip_if || !mors->chip_if->to_chip_pageset)
		return 0;

	tx_pageset = mors->chip_if->to_chip_pageset;
	if (!(tx_pageset->flags & MORSE_CHIP_IF_FLAGS_DIR_TO_CHIP)) {
		MORSE_WARN_ON_ONCE(FEATURE_ID_PAGER, 1);
		return 0;
	}

	count += tx_pageset->beacon_q.pending.qlen;
	count += tx_pageset->mgmt_q.pending.qlen;
	count += tx_pageset->cmd_q.pending.qlen;

	for (i = 0; i < ARRAY_SIZE(tx_pageset->data_qs); i++)
		count += tx_pageset->data_qs[i].pending.qlen;

	return count;
}

static void pagesets_flush_tx_data(struct morse *mors)
{
	int aci;
	struct morse_skbq *q;

	q = skbq_pageset_get_q(mors, MORSE_SKB_CHAN_BEACON, TX_Q_NO_ACI);
	if (q)
		morse_skbq_tx_flush(q);

	q = skbq_pageset_get_q(mors, MORSE_SKB_CHAN_MGMT, TX_Q_NO_ACI);
	if (q)
		morse_skbq_tx_flush(q);

	for (aci = MORSE_ACI_BE; aci <= MORSE_ACI_VO; aci++) {
		q = skbq_pageset_get_q(mors, MORSE_SKB_CHAN_DATA, aci);
		if (q)
			morse_skbq_tx_flush(q);
	}
}

static void pagesets_flush_cmds(struct morse *mors)
{
	struct morse_skbq *q = skbq_pageset_get_q(mors, MORSE_SKB_CHAN_COMMAND, TX_Q_NO_ACI);

	if (!q)
		return;

	morse_skbq_tx_flush(q);
}

static void pagesets_interrupts_set(struct morse *mors, bool is_init)
{
	struct morse_pager *rx_data = mors->chip_if->from_chip_pageset->populated_pager;
	struct morse_pager *tx_return = mors->chip_if->to_chip_pageset->return_pager;

	pageset_pager_irq_enable(tx_return, is_init);
	pageset_pager_irq_enable(rx_data, is_init);
	pageset_tx_status_irq_enable(mors, is_init);
	pageset_cmd_resp_irq_enable(mors, is_init);
	morse_hw_enable_stop_notifications(mors, is_init);

	/* Always initialise headless interrupt to false (init or finish flow) */
	morse_hw_headless_done_irq_enable(mors, false);
}

static void pagesets_interrupts_suspend(struct morse *mors, bool suspend)
{
	struct morse_pager *rx_data = mors->chip_if->from_chip_pageset->populated_pager;
	struct morse_pager *tx_return = mors->chip_if->to_chip_pageset->return_pager;

	/* Unlike init/finish - not all interrupts are disabled while
	 * quiescing HW to enter suspend
	 */
	pageset_pager_irq_enable(tx_return, !suspend);
	pageset_pager_irq_enable(rx_data, !suspend);
	pageset_tx_status_irq_enable(mors, !suspend);
	pageset_cmd_resp_irq_enable(mors, !suspend);
}

static void pagesets_free(struct morse *mors)
{
	int i;
	struct morse_pageset *pageset;

	if (!mors->chip_if || !mors->chip_if->pagesets)
		return;

	for (pageset = mors->chip_if->pagesets, i = 0;
	     i < mors->chip_if->pageset_count; pageset++, i++)
		pageset_finish(pageset);

	kfree(mors->chip_if->pagesets);
	mors->chip_if->pagesets = NULL;
	mors->chip_if->to_chip_pageset = NULL;
	mors->chip_if->from_chip_pageset = NULL;
}

static int pagesets_init(struct morse *mors)
{
	int i;
	int ret;
	struct morse_pager *pager;
	struct morse_pager *rx_data = NULL;
	struct morse_pager *rx_return = NULL;
	struct morse_pager *tx_data = NULL;
	struct morse_pager *tx_return = NULL;

	ret = morse_pager_hw_pagers_init(mors);
	if (ret)
		return ret;

	mors->chip_if->pagesets = NULL;
	mors->chip_if->pageset_count = 0;

	/* Tie pagers to the pageset */
	for (pager = mors->chip_if->pagers, i = 0; i < mors->chip_if->pager_count; pager++, i++) {
		if ((pager->flags & MORSE_PAGER_FLAGS_DIR_TO_HOST) &&
		    (pager->flags & MORSE_PAGER_FLAGS_POPULATED)) {
			rx_data = pager;
		} else if ((pager->flags & MORSE_PAGER_FLAGS_DIR_TO_HOST) &&
			   (pager->flags & MORSE_PAGER_FLAGS_FREE)) {
			rx_return = pager;
		} else if ((pager->flags & MORSE_PAGER_FLAGS_DIR_TO_CHIP) &&
			   (pager->flags & MORSE_PAGER_FLAGS_POPULATED)) {
			tx_data = pager;
		} else if ((pager->flags & MORSE_PAGER_FLAGS_DIR_TO_CHIP) &&
			   (pager->flags & MORSE_PAGER_FLAGS_FREE)) {
			tx_return = pager;
		} else {
			MORSE_WARN(mors, "%s: unknown pager flags 0x%x\n", __func__, pager->flags);
		}
	}

	if (!rx_data || !rx_return || !tx_data || !tx_return) {
		MORSE_ERR(mors, "%s: insufficient pagers available\n", __func__);
		ret = -EFAULT;
		goto err_exit;
	}

	/* Setup pagesets */
	mors->chip_if->pageset_count = 2;
	mors->chip_if->pagesets = kcalloc(mors->chip_if->pageset_count,
					  sizeof(struct morse_pageset),
					  GFP_KERNEL);

	ret = pageset_init(mors, &mors->chip_if->pagesets[0],
			   (MORSE_CHIP_IF_FLAGS_DIR_TO_CHIP |
			    MORSE_CHIP_IF_FLAGS_COMMAND |
			    MORSE_CHIP_IF_FLAGS_DATA), tx_data, tx_return);
	if (ret)
		goto err_exit;

	ret = pageset_init(mors, &mors->chip_if->pagesets[1],
			   (MORSE_CHIP_IF_FLAGS_DIR_TO_HOST |
			    MORSE_CHIP_IF_FLAGS_COMMAND |
			    MORSE_CHIP_IF_FLAGS_DATA), rx_data, rx_return);
	if (ret)
		goto err_exit;

	/* Only valid while we only have 2 pagesets */
	mors->chip_if->to_chip_pageset = &mors->chip_if->pagesets[0];
	mors->chip_if->from_chip_pageset = &mors->chip_if->pagesets[1];
	INIT_WORK(&mors->recovery.hw_stop, morse_hw_stop_work);
	INIT_WORK(&mors->chip_if_work, morse_pagesets_work);
	INIT_WORK(&mors->tx_stale_work, morse_pagesets_stale_tx_work);
	INIT_KFIFO(mors->chip_if->bypass.tx_sts.to_process);
	INIT_KFIFO(mors->chip_if->bypass.cmd_resp.to_process);

	/* Set page return flag now to force eval on chip wq eval */
	set_bit(MORSE_PAGE_RETURN_PEND, &mors->chip_if->event_flags);
	pagesets_interrupts_set(mors, true);
	return 0;

err_exit:
	pagesets_free(mors);
	morse_pager_hw_pagers_finish(mors);
	return ret;
}

static int pagesets_hw_restarted(struct morse *mors)
{
	set_bit(MORSE_PAGE_RETURN_PEND, &mors->chip_if->event_flags);
	pagesets_flush_all_caches(mors);
	pagesets_interrupts_set(mors, true);
	return 0;
}

static void pagesets_finish(struct morse *mors)
{
	/* Disable all interrupts */
	pagesets_interrupts_set(mors, false);

	/* Ordering of cancelling work and freeing the pageset matters */
	cancel_work_sync(&mors->chip_if_work);
	pagesets_free(mors);
	cancel_work_sync(&mors->tx_stale_work);
	cancel_work_sync(&mors->recovery.hw_stop);

	/* Free underlying pager hw */
	morse_pager_hw_pagers_finish(mors);
}

static void pagesets_suspend(struct morse *mors)
{
	const bool is_suspend = true;

	/* Disable pageset specific interrupts while quiescing */
	pagesets_interrupts_suspend(mors, is_suspend);

	/* Temporarily disable global hw irq to flush work */
	morse_bus_set_irq(mors, false);
	morse_hw_irq_clear(mors);

	/* Cancel pending chip work and flush all in-flight data/cmds */
	cancel_work_sync(&mors->chip_if_work);
	mors->cfg->ops->flush_tx_data(mors);
	mors->cfg->ops->flush_cmds(mors);

	/* Enable global interrupts again */
	morse_bus_set_irq(mors, true);
}

static void pagesets_resume(struct morse *mors)
{
	const bool is_suspend = false;

	/* Set page return flag now to force eval on chip wq */
	set_bit(MORSE_PAGE_RETURN_PEND, &mors->chip_if->event_flags);
	queue_work(mors->chip_wq, &mors->chip_if_work);

	/* Enable pageset specific interrupts upon resume */
	pagesets_interrupts_suspend(mors, is_suspend);
}

const struct chip_if_ops morse_pageset_ops = {
	.init = pagesets_init,
	.hw_restarted = pagesets_hw_restarted,
	.flush_cache = pagesets_flush_all_caches,
	.flush_tx_data = pagesets_flush_tx_data,
	.flush_cmds = pagesets_flush_cmds,
	.skbq_get_tx_status_pending_count = pagesets_get_tx_status_pending_count,
	.skbq_get_tx_buffered_count = pagesets_get_tx_buffered_count,
	.finish = pagesets_finish,
	.suspend = pagesets_suspend,
	.resume = pagesets_resume,
	.skbq_get_tx_qs = skbq_pageset_get_tx_qs,
	.skbq_bcn_tc_q = skbq_pageset_bcn_tc_q,
	.skbq_mgmt_tc_q = skbq_pageset_mgmt_tc_q,
	.skbq_cmd_tc_q = skbq_pageset_cmd_tc_q,
	.skbq_tc_q_from_aci = skbq_pageset_tc_q_from_aci,
	.skbq_rx_data_q = skbq_pageset_get_rx_data_q,
	.chip_if_handle_irq = morse_pageset_irq_handler,
};
