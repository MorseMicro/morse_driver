/*
 * Copyright 2021-2023 Morse Micro
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <linux/bitops.h>
#include "skbq.h"
#include "pager_hw.h"
#include "bus.h"
#include "debug.h"
#include "chip_if.h"
#include "trace.h"

/* Split 32-bits into 2 parts: block + bitmap */
#define MORSE_PAGER_BITS_BLOCK_LEN	(1)
#define MORSE_PAGER_BITS_BITMAP_LEN	(32 - MORSE_PAGER_BITS_BLOCK_LEN)

#define MORSE_PAGER_NUM_BLOCKS		BIT(MORSE_PAGER_BITS_BLOCK_LEN)

struct morse_pager_hw_table {
	u32 addr;	/* location of the pager table */
	u32 count;	/* Number of entries in the table */
};

struct morse_pager_cache {
	/* Encoded bitmap of pages in pkt memory */
	u32 bitmap[MORSE_PAGER_NUM_BLOCKS];
};

struct morse_pager_hw_aux_data {
	/* These addresses are be the put and pop
	 * for the same instance of the pager hw.
	 */
	u32 put_addr;
	u32 pop_addr;

	struct morse_pager_cache cache;
};

static int pager_hw_init(struct morse *mors, struct morse_pager *pager, u32 put_addr, u32 pop_addr)
{
	struct morse_pager_hw_aux_data *aux_data;

	pager->aux_data = kzalloc(sizeof(*aux_data), GFP_KERNEL);
	if (!pager->aux_data)
		return -ENOMEM;

	aux_data = (struct morse_pager_hw_aux_data *)pager->aux_data;
	aux_data->put_addr = put_addr;
	aux_data->pop_addr = pop_addr;

	return 0;
}

static void pager_hw_finish(struct morse *mors, struct morse_pager *pager)
{
	kfree(pager->aux_data);
	pager->aux_data = NULL;
}

static int pager_hw_read_table(struct morse *mors, struct morse_pager_hw_table *tbl_ptr)
{
	int ret;
	const u32 pager_count_addr = mors->cfg->host_table_ptr +
	    offsetof(struct host_table, chip_if) +
	    offsetof(struct morse_chip_if_host_table, pager_count);

	tbl_ptr->addr = mors->cfg->host_table_ptr +
	    offsetof(struct host_table, chip_if) +
	    offsetof(struct morse_chip_if_host_table, pager_table);

	morse_claim_bus(mors);
	ret = morse_reg32_read(mors, pager_count_addr, &tbl_ptr->count);
	morse_release_bus(mors);

	if (ret != 0 && (tbl_ptr->count == 0 || tbl_ptr->addr == 0))
		ret = -EIO;

	return ret;
}

static int pager_hw_get_index_from_page(struct morse_pager *pager,
					struct morse_page *page,
					u8 *index)
{
	struct morse_pager_pkt_memory *pkt_memory = &pager->mors->chip_if->pkt_memory;

	if (page->addr < pkt_memory->base_addr ||
			page->addr > pkt_memory->base_addr + pkt_memory->page_len * pkt_memory->num)
		return -EINVAL;

	*index = (page->addr - pkt_memory->page_len_reserved - pkt_memory->base_addr) /
			pkt_memory->page_len;

	return 0;
}

static int pager_hw_get_page_from_index(struct morse_pager *pager,
					u8 index,
					struct morse_page *page)
{
	struct morse_pager_pkt_memory *pkt_memory = &pager->mors->chip_if->pkt_memory;

	if (index >= pkt_memory->num)
		return -EINVAL;

	page->addr = pkt_memory->base_addr + (pkt_memory->page_len * index) +
			pkt_memory->page_len_reserved;
	page->size_bytes = pkt_memory->page_len - pkt_memory->page_len_reserved;

	return 0;
}

static void pager_hw_cache_pages(struct morse_pager *pager, struct morse_page *page)
{
	struct morse_pager_hw_aux_data *aux_data =
		(struct morse_pager_hw_aux_data *)pager->aux_data;
	u32 block = page->addr >> MORSE_PAGER_BITS_BITMAP_LEN;

	aux_data->cache.bitmap[block] = page->addr & ~BIT(MORSE_PAGER_BITS_BITMAP_LEN);

	trace_pager_hw_cache_put_pages(pager, hweight32(aux_data->cache.bitmap[block]));
}

static int pager_hw_get_page_from_cache(struct morse_pager *pager, struct morse_page *page)
{
	int ret;
	struct morse_pager_hw_aux_data *aux_data =
		(struct morse_pager_hw_aux_data *)pager->aux_data;
	int block;
	int index;

	for (block = 0; block < ARRAY_SIZE(aux_data->cache.bitmap); ++block) {
		if (aux_data->cache.bitmap[block])
			break;
	}

	if (block >= ARRAY_SIZE(aux_data->cache.bitmap)) {
		ret = -ENOENT;
		goto exit;
	}

	index = ffs(aux_data->cache.bitmap[block]);
	if (!index) {
		ret = -ENOENT;
		goto exit;
	}
	index -= 1;

	aux_data->cache.bitmap[block] &= ~BIT(index);

	ret = pager_hw_get_page_from_index(pager,
		(block * MORSE_PAGER_BITS_BITMAP_LEN) + index, page);

exit:
	trace_pager_hw_cache_get(pager, (ret) ? 0 : page->addr);
	return ret;
}

static int pager_hw_pop(struct morse_pager *pager, struct morse_page *page)
{
	struct morse_pager_hw_aux_data *aux_data =
	    (struct morse_pager_hw_aux_data *)pager->aux_data;
	int ret = 0;
	u32 pop_val;

	morse_claim_bus(pager->mors);
	ret = morse_reg32_read(pager->mors, aux_data->pop_addr, &pop_val);
	morse_release_bus(pager->mors);

	if (!ret) {
		/* Pager has no pages left */
		if (pop_val == 0) {
			ret = -EAGAIN;
			goto exit;
		}

		page->addr = pop_val;
		page->size_bytes = pager->page_size_bytes;
	}

exit:
	trace_pager_hw_pop(pager, (ret) ? 0 : page->addr);
	return ret;
}

static int pager_hw_put(struct morse_pager *pager, struct morse_page *page)
{
	struct morse_pager_hw_aux_data *aux_data =
	    (struct morse_pager_hw_aux_data *)pager->aux_data;
	int ret;

	trace_pager_hw_put(pager, page->addr);
	morse_claim_bus(pager->mors);
	ret = morse_reg32_write(pager->mors, aux_data->put_addr, page->addr);
	morse_release_bus(pager->mors);

	if (!ret) {
		page->addr = 0;
		page->size_bytes = 0;
	}
	return ret;
}

static void pager_hw_notify_pager(const struct morse_pager *pager)
{
	struct morse_pager_hw_aux_data *aux_data =
		(struct morse_pager_hw_aux_data *)pager->aux_data;
	u32 block;
	struct morse_page page;

	page.size_bytes = 0;

	for (block = 0; block < ARRAY_SIZE(aux_data->cache.bitmap); ++block) {
		if (!aux_data->cache.bitmap[block])
			continue;

		page.addr = (block << MORSE_PAGER_BITS_BITMAP_LEN) | aux_data->cache.bitmap[block];

		trace_pager_hw_notify(pager, hweight32(aux_data->cache.bitmap[block]));
		pager_hw_put((struct morse_pager *)pager, &page);
		aux_data->cache.bitmap[block] = 0;
	}
}

int morse_pager_hw_notify(const struct morse_pager *pager)
{
	lockdep_assert_held(&pager->parent->lock);
	/* Put the cached pages to to_host free HW pager */
	if (pager->parent->mors->chip_if->pkt_memory.num &&
	    (pager->flags & (MORSE_PAGER_FLAGS_DIR_TO_HOST | MORSE_PAGER_FLAGS_FREE)))
		pager_hw_notify_pager(pager);

	return 0;
}

int morse_pager_hw_pop(struct morse_pager *pager, struct morse_page *page)
{
	int ret;

	lockdep_assert_held(&pager->parent->lock);
	if (!(pager->flags & MORSE_PAGER_FLAGS_FREE) ||
	    !pager->parent->mors->chip_if->pkt_memory.num)
		return pager_hw_pop(pager, page);

	/* Try to return a page from the cache. If unavailable, pop from the HW pager, cache the
	 * pages and return a page from it
	 */
	ret = pager_hw_get_page_from_cache(pager, page);
	if (!ret)
		return ret;

	ret = pager_hw_pop(pager, page);
	if (ret)
		return ret;

	pager_hw_cache_pages(pager, page);

	ret = pager_hw_get_page_from_cache(pager, page);

	return ret;
}

int morse_pager_hw_put(struct morse_pager *pager, struct morse_page *page)
{
	struct morse_pager_hw_aux_data *aux_data =
		(struct morse_pager_hw_aux_data *)pager->aux_data;
	int ret;
	u8 index;
	u32 block;

	lockdep_assert_held(&pager->parent->lock);
	if (!(pager->flags & MORSE_PAGER_FLAGS_FREE) ||
	    !pager->parent->mors->chip_if->pkt_memory.num)
		return pager_hw_put(pager, page);

	/* Cache the pages for now. Actual "put" to the pager HW is delayed to notify() */
	ret = pager_hw_get_index_from_page(pager, page, &index);
	if (ret)
		return ret;

	block = index / MORSE_PAGER_BITS_BITMAP_LEN;
	aux_data->cache.bitmap[block] |= BIT(index - (block * MORSE_PAGER_BITS_BITMAP_LEN));
	trace_pager_hw_store_bit_bulk(pager, BIT(index - (block * MORSE_PAGER_BITS_BITMAP_LEN)));
	return 0;
}

int morse_pager_hw_page_write(struct morse_pager *pager,
			      struct morse_page *page,
			      int offset,
			      const char *buff,
			      int num_bytes)
{
	int ret;

	lockdep_assert_held(&pager->parent->lock);
	if (offset < 0)
		return -EINVAL;

	if (num_bytes > page->size_bytes)
		return -EMSGSIZE;

	if (page->addr == 0)
		return -EFAULT;

	trace_pager_hw_write_page(pager, page->addr + offset);
	morse_claim_bus(pager->mors);
	ret = morse_dm_write(pager->mors, page->addr + offset, buff, num_bytes);
	morse_release_bus(pager->mors);

	return ret;
}

int morse_pager_hw_page_read(struct morse_pager *pager,
			     struct morse_page *page,
			     int offset,
			     char *buff,
			     int num_bytes)
{
	int ret;

	lockdep_assert_held(&pager->parent->lock);
	if (offset < 0)
		return -EINVAL;

	if (num_bytes > page->size_bytes)
		return -EMSGSIZE;

	if (page->addr == 0)
		return -EFAULT;

	trace_pager_hw_read_page(pager, page->addr + offset);
	morse_claim_bus(pager->mors);
	ret = morse_dm_read(pager->mors, page->addr + offset, buff, num_bytes);
	morse_release_bus(pager->mors);

	return ret;
}

void morse_pager_hw_flush_cache(struct morse_pager *pager)
{
	struct morse_pager_hw_aux_data *aux_data =
		(struct morse_pager_hw_aux_data *)pager->aux_data;

	lockdep_assert_held(&pager->parent->lock);
	memset(&aux_data->cache.bitmap, 0, sizeof(aux_data->cache.bitmap));
}

/* Will free the memory allocated in morse_pager_hw_pagers_init */
static void pager_hw_pagers_free(struct morse *mors)
{
	if (mors->chip_if && mors->chip_if->pagers) {
		int i;
		struct morse_pager *pager;

		for (pager = mors->chip_if->pagers, i = 0;
		     i < mors->chip_if->pager_count;
		     pager++, i++)
			pager_hw_finish(mors, pager);

		mors->chip_if->pager_count = 0;
		kfree(mors->chip_if->pagers);
		mors->chip_if->pagers = NULL;
	}

	kfree(mors->chip_if);
	mors->chip_if = NULL;
}

int morse_pager_hw_pagers_init(struct morse *mors)
{
	int i;
	int ret;
	struct morse_pager *pager;
	struct morse_pager_hw_table tbl_ptr;
	struct morse_pager_hw_entry *entries = NULL;

	ret = pager_hw_read_table(mors, &tbl_ptr);
	if (ret)
		return ret;

	mors->chip_if = kzalloc(sizeof(*mors->chip_if), GFP_KERNEL);
	if (!mors->chip_if) {
		ret = -ENOMEM;
		goto exit;
	}

	mors->chip_if->pagers = kcalloc(tbl_ptr.count, sizeof(struct morse_pager), GFP_KERNEL);
	if (!mors->chip_if->pagers) {
		ret = -ENOMEM;
		goto exit;
	}

	mors->chip_if->pager_count = tbl_ptr.count;
	entries = kcalloc(tbl_ptr.count, sizeof(*entries), GFP_KERNEL);
	if (!entries) {
		ret = -ENOMEM;
		goto exit;
	}

	morse_claim_bus(mors);
	ret = morse_dm_read(mors, tbl_ptr.addr, (u8 *)entries, sizeof(*entries) * tbl_ptr.count);
	morse_release_bus(mors);
	if (ret)
		goto exit;

	/* First initialise implementation specific data */
	for (pager = mors->chip_if->pagers, i = 0; i < mors->chip_if->pager_count; pager++, i++) {
		ret = pager_hw_init(mors,
				    pager,
				    __le32_to_cpu(entries[i].push_addr),
				    __le32_to_cpu(entries[i].pop_addr));
		if (ret)
			goto exit;

		pager->mors = mors;
		pager->flags = entries[i].flags;
		pager->page_size_bytes = __le16_to_cpu(entries[i].page_size);
		pager->parent = NULL;
		pager->id = i;
	}

	MORSE_INFO(mors, "%s: %d pagers detected\n", __func__, mors->chip_if->pager_count);
	ret = 0;
exit:
	if (ret) {
		MORSE_ERR(mors, "%s: failed (ret:%d)\n", __func__, ret);
		pager_hw_pagers_free(mors);
	}

	kfree(entries);
	return ret;
}

void morse_pager_hw_pagers_finish(struct morse *mors)
{
	pager_hw_pagers_free(mors);
}
