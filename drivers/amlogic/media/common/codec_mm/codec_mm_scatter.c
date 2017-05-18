/*
 * drivers/amlogic/media/common/codec_mm/codec_mm_scatter.c
 *
 * Copyright (C) 2016 Amlogic, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/dma-contiguous.h>
#include <linux/cma.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/libfdt_env.h>
#include <linux/of_reserved_mem.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/genalloc.h>
#include <linux/dma-mapping.h>
#include <linux/dma-contiguous.h>
#include <linux/amlogic/media/codec_mm/codec_mm.h>
#include <linux/amlogic/media/codec_mm/codec_mm_scatter.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/mm.h>

#include "codec_mm_priv.h"
#include "codec_mm_scatter_priv.h"

#define SCATTER_MEM "SCATTER_MEM"

/*#define PHY_ADDR_NEED_64BITS*/
#if PAGE_SHIFT >= 12
#define SID_MASK 0xfff
#elif PAGE_SHIFT >= 8
#define SID_MASK 0xff
#else
#error "unsupport PAGE_SHIFT PAGE_SHIFT must >= 8" ## PAGE_SHIFT
#endif

#define MAX_ADDR_SHIFT PAGE_SHIFT
#define MAX_SID (SID_MASK - 1)
#define MAX_HASH_SID (MAX_SID - 1)

#define ADDR_SEED(paddr) (((paddr) >> (MAX_ADDR_SHIFT << 1)) +\
					((paddr) >> MAX_ADDR_SHIFT) +\
					(paddr))

/*
*hash ID: 0-MAX_HASH_SID
*ONE_PAGE_SID: MAX_SID
*/
#define HASH_PAGE_ADDR(paddr) (ADDR_SEED(paddr) % MAX_SID)
#define SLOT_TO_SID(slot)	HASH_PAGE_ADDR(((slot->phy_addr)>>PAGE_SHIFT))

#define ONE_PAGE_SID (SID_MASK)
#define PAGE_SID(mm_page)  (page_sid_type)((mm_page) & SID_MASK)
#define PAGE_ADDR(mm_page) (ulong)(((mm_page) >> MAX_ADDR_SHIFT)\
			<< MAX_ADDR_SHIFT)

#define ADDR2PAGE(addr, sid)	(((addr) & (~SID_MASK)) | (sid))

#define PAGE_SID_OF_MMS(mms, id) PAGE_SID((mms)->pages_list[id])
#define PAGE_ADDR_OF_MMS(mms, id) PAGE_ADDR((mms)->pages_list[id])

#define INVALID_ID(mms, id) (!(mms) ||\
					!(mms)->pages_list ||\
					(mms)->page_cnt <= (id) ||\
					(id) < 0)

#define SID_OF_ONEPAGE(sid)	   ((sid) == ONE_PAGE_SID)
#define ADDR2BIT(base, addr)  (((addr) - (base)) >> PAGE_SHIFT)
#define BIT2ADDR(base, bit)   ((base) + (1<<PAGE_SHIFT) * (bit))
#define VALID_SID(sid) (((sid) < MAX_SID) || SID_OF_ONEPAGE(sid))
#define VALID_BIT(slot, bit) (bit >= 0 && ((slot->pagemap_size << 3) > bit))
#define CODEC_MM_S_ERR(x) ((-1000) - (x))

/*#define SCATTER_DEBUG*/
#define ERR_LOG(args...) pr_err(args)
#define WAR_LOG(args...) pr_warn(args)
#define INFO_LOG(args...) pr_info(args)

#ifdef SCATTER_DEBUG
#define DBG_LOG(args...) pr_info(args)
#else
#define DBG_LOG(args...)
#endif
#define MAX_SYS_BLOCK_PAGE 128
#define MIN_SYS_BLOCK_PAGE 8

/*#define USE_KMALLOC_FOR_SCATTER*/
#ifdef USE_KMALLOC_FOR_SCATTER
#define SC_ALLOC(s, f) kmalloc(s, f)
#define SC_FREE(p) kfree(p)
#else
#define SC_ALLOC(s, f) vmalloc(s)
#define SC_FREE(p) vfree(p)

#endif
#define MAX_SC_LIST 64
struct codec_mm_scatter_mgt {
	struct codec_mm_slot *slot_list_map[MAX_SID];
	int codec_mm_num;
	int total_page_num;
	int alloced_page_num;
	int max_alloced;
	int try_alloc_in_cma_page_cnt;
	int try_alloc_in_sys_page_cnt;
	int try_alloc_in_sys_page_cnt_max;
	int try_alloc_in_sys_page_cnt_min;
	int alloc_from_cma_first;
	int enable_slot_from_sys;
	int no_cache_size_M;
	int support_from_slot_sys;
	int one_page_cnt;
	int scatters_cnt;
	int slot_cnt;
	int reserved_block_mm_M;
	int keep_size_PAGE;

	int alloc_from_sys_sc_cnt;
	int alloc_from_sys_page_cnt;
	int alloc_from_sys_max_page_cnt;

	int delay_free_on;
	int force_cache_on;
	int force_cache_page_cnt;
	u64 delay_free_timeout_jiffies64;

/*time states*/
	int alloc_max_us;
	u64 alloc_total_us;
	int alloc_cnt;
	int alloc_10us_less_cnt;
	int alloc_10_50us_cnt;
	int alloc_50_100us_cnt;
	int alloc_100_1000us_cnt;
	int alloc_1_10ms_cnt;
	int alloc_10_100ms_cnt;
	int alloc_100ms_up_cnt;

	struct delayed_work dealy_work;
	int scatter_task_run_num;
	struct codec_mm_scatter *cache_sc;
	int cached_pages;
	spinlock_t list_lock;
	struct mutex monitor_lock;
	struct list_head free_list;	/*slot */
	struct list_head scatter_list;	/*scatter list */
	struct codec_mm_scatter *scmap[MAX_SC_LIST];/*used for valid check. */
};

static struct codec_mm_scatter_mgt *scatter_mgt;

static struct codec_mm_scatter_mgt *codec_mm_get_scatter_mgt(void)
{
	return scatter_mgt;
}

/*#define MY_MUTEX_DEBUG*/
#ifdef MY_MUTEX_DEBUG
#define codec_mm_scatter_lock(s) \
	codec_mm_scatter_lock_debug(s, __LINE__)

#define codec_mm_list_lock(s) \
	codec_mm_list_lock_debug(s, __LINE__)

static inline int mutex_trylock_time(
	struct mutex *lock, int wait)
{
	unsigned long timeout = jiffies + wait;
	int locked = mutex_trylock(lock);

	while (!locked && time_before(jiffies, timeout)) {
		msleep(20);
		locked = mutex_trylock(lock);
	}
	return locked;
}

#define TRY_MLOCK_INFO(lock, line, time, info)\
		static int last_lock_line;\
		while (!mutex_trylock_time((lock), time)) {\
			pr_err(info " mutex has lock on %d,new lock on %d\n",\
			last_lock_line, line);\
		} \
		last_lock_line = line;\


static inline int spin_trylock_time(
			spinlock_t *lock, int wait)
{
	unsigned long timeout = jiffies + wait;
	int locked = spin_trylock(lock);

	while (!locked && time_before(jiffies, timeout)) {
		msleep(20);
		locked = spin_trylock(lock);
	}
	return locked;
}

#define TRY_SLOCK_INFO(lock, line, time, info)\
		static int last_lock_line;\
		while (!spin_trylock_time((lock), time)) {\
			pr_err(info " spin has lock on %d,new lock on %d\n",\
			last_lock_line, line);\
		} \
		last_lock_line = line;\


static inline void codec_mm_scatter_lock_debug(
	struct codec_mm_scatter *mms,
	int line)
{
	TRY_MLOCK_INFO(&mms->mutex, line, 10 * HZ, "mms");
}

static inline void codec_mm_list_lock_debug(
	struct codec_mm_scatter_mgt *smgt, int line)
{
	TRY_SLOCK_INFO(&smgt->list_lock, line, 10 * HZ, "list");
}
#else
static inline void codec_mm_scatter_lock(
	struct codec_mm_scatter *mms)
{
	mutex_lock(&mms->mutex);
}
static inline void codec_mm_list_lock(
	struct codec_mm_scatter_mgt *smgt)
{
	spin_lock(&smgt->list_lock);
}
#endif
static inline void codec_mm_scatter_unlock(
	struct codec_mm_scatter *mms)
{
	mutex_unlock(&mms->mutex);
}

static inline void codec_mm_list_unlock(
	struct codec_mm_scatter_mgt *smgt)
{
	spin_unlock(&smgt->list_lock);
}

static int codec_mm_scatter_alloc_want_pages_in(
		struct codec_mm_scatter *mms,
		int want_pages);

static struct workqueue_struct *codec_mm_scatter_wq_get(void)
{
	static struct workqueue_struct *codec_mm_scatter;

	if (!codec_mm_scatter)
		codec_mm_scatter = create_singlethread_workqueue("codec_mm_sc");
	return codec_mm_scatter;
}


static int codec_mm_schedule_delay_work(int delay_ms, int for_update)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	bool ret;
	if (!for_update && delayed_work_pending(&smgt->dealy_work))
		return 0;
	if (delayed_work_pending(&smgt->dealy_work))
		cancel_delayed_work(&smgt->dealy_work);
	if (codec_mm_scatter_wq_get()) {
		ret = queue_delayed_work(codec_mm_scatter_wq_get(),
			&smgt->dealy_work, delay_ms * HZ / 1000);
	} else
		ret = schedule_delayed_work(&smgt->dealy_work,
			delay_ms * HZ / 1000);
	return ret;
}

static inline u64 codec_mm_get_current_us(void)
{
	struct timeval  tv;

	do_gettimeofday(&tv);
	return div64_u64(timeval_to_ns(&tv), 1000);
}

static void codec_mm_update_alloc_time(u64 startus)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	int spend_time_us;

	spend_time_us = (int)(codec_mm_get_current_us() - startus);
	if (spend_time_us > 0 && spend_time_us < 100000000) {
		/*	>0 && less than 100s*/
			/*else think time base changed.*/
		smgt->alloc_cnt++;
		if (spend_time_us < 10)
			smgt->alloc_10us_less_cnt++;
		else if (spend_time_us < 50)
			smgt->alloc_10_50us_cnt++;
		else if (spend_time_us < 100)
			smgt->alloc_50_100us_cnt++;
		else if (spend_time_us < 1000)
			smgt->alloc_100_1000us_cnt++;
		else if (spend_time_us < 10000)
			smgt->alloc_1_10ms_cnt++;
		else if (spend_time_us < 100000)
			smgt->alloc_10_100ms_cnt++;
		else
			smgt->alloc_100ms_up_cnt++;

		smgt->alloc_total_us += spend_time_us;
		if (spend_time_us > smgt->alloc_max_us) {
			/*..*/
			smgt->alloc_max_us  = spend_time_us;
		}
	}
}

void codec_mm_clear_alloc_infos(void)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();

	smgt->alloc_cnt = 0;
	smgt->alloc_10us_less_cnt = 0;
	smgt->alloc_10_50us_cnt = 0;
	smgt->alloc_50_100us_cnt = 0;
	smgt->alloc_100_1000us_cnt = 0;
	smgt->alloc_1_10ms_cnt = 0;
	smgt->alloc_10_100ms_cnt = 0;
	smgt->alloc_100ms_up_cnt = 0;
	smgt->alloc_total_us = 0;
	smgt->alloc_max_us = 0;
}


static int codec_mm_set_slot_in_hash(struct codec_mm_slot *slot)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();

	page_sid_type sid = SLOT_TO_SID(slot);

	if (sid < 0 || sid > MAX_SID) {
		ERR_LOG("ERROR sid %d", sid);
		return -1;
	}
	slot->sid = sid;
	INIT_LIST_HEAD(&slot->sid_list);
	INIT_LIST_HEAD(&slot->free_list);
	codec_mm_list_lock(smgt);
	if (!smgt->slot_list_map[sid]) {
		smgt->slot_list_map[sid] = slot;
		slot->isroot = 1;
	} else {
		struct codec_mm_slot *f_slot = smgt->slot_list_map[sid];

		list_add_tail(&slot->sid_list, &f_slot->sid_list);
		slot->isroot = 0;
	}
	smgt->total_page_num += slot->page_num;
	smgt->slot_cnt++;
	if (slot->from_type == SLOT_FROM_GET_FREE_PAGES) {
		smgt->alloc_from_sys_sc_cnt++;
		smgt->alloc_from_sys_page_cnt += slot->page_num;
		if (smgt->alloc_from_sys_page_cnt >
			smgt->alloc_from_sys_max_page_cnt)
			smgt->alloc_from_sys_max_page_cnt =
				smgt->alloc_from_sys_page_cnt;
	}
	list_add_tail(&slot->free_list, &smgt->free_list);
	codec_mm_list_unlock(smgt);
	return 0;
}

static struct codec_mm_slot *codec_mm_find_slot_in_hash(
	page_sid_type sid, ulong addr)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	struct codec_mm_slot *fslot, *slot;

	if (!VALID_SID(sid))
		return NULL;
	codec_mm_list_lock(smgt);
	fslot = smgt->slot_list_map[sid];
	if (!fslot) {
		ERR_LOG("not valid sid %d\n",
			(int)sid);
		goto err;
	}
	slot = fslot;
	while (!(addr >= slot->phy_addr &&	/*optimization with hash? */
			 addr <
			 (slot->phy_addr +
			(slot->page_num << PAGE_SHIFT)))) {
		/*not in range. */

		slot = list_entry(slot->sid_list.prev,
			struct codec_mm_slot, sid_list);
		/*
		*pr_err("Slot range from =%p->%p\n",
		*(void *)slot->phy_addr,
		*(void *)slot->phy_addr +
		*(slot->page_num << PAGE_SHIFT));
		*/
		if (slot == fslot) {
			ERR_LOG("can't find valid slot, for addr =%p\n",
				(void *)addr);
			goto err;
		}
	}
	codec_mm_list_unlock(smgt);
	return slot;
err:
	codec_mm_list_unlock(smgt);
	return NULL;
}

static int codec_mm_slot_free(struct codec_mm_slot *slot)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	int ret = 0;

	codec_mm_list_lock(smgt);
	if (slot->alloced_page_num > 0 || slot->on_alloc_free) {
		codec_mm_list_unlock(smgt);
		return -1;
	}
	if (!list_empty(&slot->free_list))
		list_del(&slot->free_list);
	if (!list_empty(&slot->sid_list)) {
		if (slot->isroot) {
			struct codec_mm_slot *next_slot;

			next_slot = list_entry(slot->sid_list.next,
				struct codec_mm_slot, sid_list);
			next_slot->isroot = 1;
			smgt->slot_list_map[slot->sid] = next_slot;
		}
		list_del(&slot->sid_list);
	} else {		/*no sid list,clear map */
		smgt->slot_list_map[slot->sid] = NULL;
	}
	smgt->slot_cnt--;
	smgt->total_page_num -= slot->page_num;
	if (slot->from_type == SLOT_FROM_GET_FREE_PAGES) {
		smgt->alloc_from_sys_sc_cnt--;
		smgt->alloc_from_sys_page_cnt -= slot->page_num;
	}
	codec_mm_list_unlock(smgt);
	switch (slot->from_type) {
	case SLOT_FROM_CODEC_MM:
		if (slot->mm)
			codec_mm_release(slot->mm, SCATTER_MEM);
		else
			ERR_LOG("ERR:slot->mm is ERROR:%p\n", slot->mm);
		break;
	case SLOT_FROM_GET_FREE_PAGES:
		if (slot->page_header != 0)
			free_pages(slot->page_header,
				get_order(PAGE_SIZE * slot->page_num));
		else
			ERR_LOG("ERR:slot->page_header is ERROR:%p\n",
				(void *)slot->page_header);
		break;
		/*other */
	default:
		ERR_LOG("unknown from type:%d\n", slot->from_type);
		ret = -1;
	}

	kfree(slot->pagemap);
	kfree(slot);
	return 0;
}

static int codec_mm_slot_try_free(struct codec_mm_slot *slot)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();

	if (smgt->keep_size_PAGE > 0) {
		/*delay free, when size < delay_free_M MB */
		int free_pages = smgt->total_page_num - smgt->alloced_page_num;

		if (free_pages < smgt->keep_size_PAGE)
			return -1;
	}
	return codec_mm_slot_free(slot);
}

static inline int codec_mm_slot_init_bitmap(struct codec_mm_slot *slot)
{
	slot->alloced_page_num = 0;
	/*bytes = 8bits for 8 pages.
	*1 more for less than 8 page.
	*another for reserved.
	*/
	slot->pagemap_size = (slot->page_num >> 3) + 2;
	slot->pagemap = kmalloc(slot->pagemap_size, GFP_KERNEL);
	if (!slot->pagemap) {
		ERR_LOG("ERROR.init pagemap failed\n");
		return -1;
	}
	memset(slot->pagemap, 0, slot->pagemap_size);
	slot->next_bit = 0;
	return 0;
}

/*
*flags : 1. don't used codecmm.
*/
static struct codec_mm_slot *codec_mm_slot_alloc(int size, int flags)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	struct codec_mm_slot *slot;
	struct codec_mm_s *mm;
	int try_alloc_size = size;
	int have_alloced = 0;

	if (try_alloc_size > 0 && try_alloc_size <= PAGE_SIZE)
		return NULL;	/*don't alloc less than one PAGE. */
	slot = kmalloc(sizeof(struct codec_mm_slot), GFP_KERNEL);
	if (!slot)
		return NULL;
	memset(slot, 0, sizeof(struct codec_mm_slot));
	do {
		if (flags & 1)
			break;	/*ignore codec_mm */
		if ((try_alloc_size <= 0 ||
			try_alloc_size > 64 * 1024) &&	/*must > 512K. */
			codec_mm_get_free_size() >
			smgt->reserved_block_mm_M * SZ_1M) {
			/*try from codec_mm */
			if (try_alloc_size <= 0) {
				try_alloc_size =
					smgt->try_alloc_in_cma_page_cnt *
					PAGE_SIZE;
			}
			if (codec_mm_get_free_size() < try_alloc_size)
				try_alloc_size = codec_mm_get_free_size();
			mm = codec_mm_alloc(SCATTER_MEM, try_alloc_size, 0,
				CODEC_MM_FLAGS_CMA_FIRST |
				CODEC_MM_FLAGS_FOR_VDECODER |
				CODEC_MM_FLAGS_FOR_SCATTER);
			if (mm != NULL) {
				slot->from_type = SLOT_FROM_CODEC_MM;
				slot->mm = mm;
				slot->page_num = mm->page_count;
				slot->phy_addr = mm->phy_addr;
				codec_mm_slot_init_bitmap(slot);
				if (slot->pagemap == NULL) {
					codec_mm_release(mm, SCATTER_MEM);
					break;	/*try next. */
				}
				have_alloced = 1;
				DBG_LOG("alloced from codec mm %d!!!\n",
					slot->page_num);
			}
		}
	} while (0);
	if (!have_alloced && !smgt->support_from_slot_sys) {
		/*not enabled from sys */
		goto error;
	}
	if (!have_alloced) {	/*init for sys alloc */
		if (size <= 0)
			try_alloc_size =
				smgt->try_alloc_in_sys_page_cnt << PAGE_SHIFT;
		else
			try_alloc_size = PAGE_ALIGN(size);
		if (try_alloc_size <= PAGE_SIZE << 1) {
			DBG_LOG("try too small %d, try one page now,\n",
				try_alloc_size);
			goto error;	/*don't alloc 1 page with slot. */
		}
	}
	if (!have_alloced && codec_mm_video_tvp_enabled()) {
		/*tvp not support alloc from sys. */
		goto error;
	}
	while (!have_alloced) {
		/*don't alloc 1 page with slot. */
		/*try alloc from sys. */
		int page_order = get_order(try_alloc_size);
		slot->page_header = __get_free_pages(
			__GFP_IO | __GFP_NOWARN | __GFP_NORETRY,
			page_order);
		if (!slot->page_header) {
			if ((try_alloc_size >> (PAGE_SHIFT + 1)) >=
				smgt->try_alloc_in_sys_page_cnt_min) {
				try_alloc_size = try_alloc_size >> 1;
				smgt->try_alloc_in_sys_page_cnt =
					try_alloc_size >> PAGE_SHIFT;
				continue;	/*try less block memory. */
			} else {
				/*disabled from sys
				   *may auto enabled when free more.
				 */
				smgt->support_from_slot_sys = 0;
				ERR_LOG("alloc sys failed size =%d!!!\n",
					try_alloc_size);
				goto error;
			}
		}
		slot->from_type = SLOT_FROM_GET_FREE_PAGES;
		slot->mm = NULL;
		slot->page_num = 1 << page_order;
		slot->phy_addr =
			virt_to_phys((unsigned long *)slot->page_header);
		codec_mm_slot_init_bitmap(slot);
		if (slot->pagemap == NULL) {
			free_pages(slot->page_header, page_order);
			goto error;
		}
		have_alloced = 1;
		break;
	}
	codec_mm_set_slot_in_hash(slot);

	return slot;
error:
	kfree(slot);
	return NULL;
}

static int codec_mm_slot_free_page(struct codec_mm_slot *slot, ulong phy_addr)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	int bit;

	if (!slot || !slot->pagemap || slot->page_num <= 0)
		return CODEC_MM_S_ERR(4);

	bit = ADDR2BIT(slot->phy_addr, phy_addr);
	if (!VALID_BIT(slot, bit))
		return CODEC_MM_S_ERR(5);	/*!!!out of page map.!! */
	codec_mm_list_lock(smgt);
	slot->on_alloc_free++;
	if (!test_and_clear_bit(bit, slot->pagemap)) {
		ERR_LOG("ERROR,page is ready free before!!! page=%p\n",
			(void *)phy_addr);
		slot->on_alloc_free--;
		codec_mm_list_unlock(smgt);
		return CODEC_MM_S_ERR(6);
	}
	slot->alloced_page_num--;
	slot->on_alloc_free--;
	codec_mm_list_unlock(smgt);
	return 0;
}

static int codec_mm_slot_alloc_pages(struct codec_mm_slot *slot,
	phy_addr_type *pages, int num)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	int alloced = 0;
	int need = num;
	int can_alloced;
	phy_addr_type page;
	int tryn = slot->page_num;
	int i;

	if (!slot || !slot->pagemap)
		return -1;
	if (slot->alloced_page_num >= slot->page_num)
		return -2;
	can_alloced = slot->page_num - slot->alloced_page_num;
	need = need > can_alloced ? can_alloced : need;
	i = slot->next_bit;
	/*if not one alloc free. quit this one */
	while (need > 0 && (slot->on_alloc_free == 1)) {
		if (!VALID_BIT(slot, i)) {
			ERR_LOG("ERROR alloc in slot %p\n",
					slot);
			ERR_LOG("\ti=%d,slot->pagemap=%p\n",
					i,
					slot->pagemap);
			break;
		}
		codec_mm_list_lock(smgt);
		if (!test_and_set_bit(i, slot->pagemap)) {
			page = ADDR2PAGE(BIT2ADDR(slot->phy_addr, i),
				slot->sid);
			slot->alloced_page_num++;
			pages[alloced] = page;
			alloced++;
			need--;
		}
		codec_mm_list_unlock(smgt);
		i++;
		if (i >= slot->page_num)
			i = 0;
		if (--tryn <= 0)
			break;
	}
	slot->next_bit = i;
	if (i >= slot->page_num)
		slot->next_bit = 0;
	DBG_LOG("alloced from %p, %d,%d,%d\n",
			slot, slot->page_num,
			slot->alloced_page_num, alloced);
	return alloced;
}

static inline phy_addr_type codec_mm_get_page_addr(
		struct codec_mm_scatter *mms,
		int id)
{
	phy_addr_type page;

	if (INVALID_ID(mms, id))
		return 0;
	page = mms->pages_list[id];
	return PAGE_ADDR(page);
}

static inline page_sid_type codec_mm_get_page_sid(
		struct codec_mm_scatter *mms,
		int id)
{
	phy_addr_type page;

	if (INVALID_ID(mms, id))
		return 0;
	page = mms->pages_list[id];
	return PAGE_SID(page);
}

static int codec_mm_page_free_to_slot(page_sid_type sid, ulong addr)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	struct codec_mm_slot *slot;
	int ret;
	int slot_free = 0;

	slot = codec_mm_find_slot_in_hash(sid, addr);
	if (!slot)
		return -1;
	ret = codec_mm_slot_free_page(slot, addr);
	if (ret != 0)
		ERR_LOG("free slot addr error =%p ret=%d\n",
			(void *)addr, ret);

	if (slot->alloced_page_num == 0)
		slot_free = (codec_mm_slot_try_free(slot) == 0);

	if (!slot_free) {	/*move to have free list. */
		codec_mm_list_lock(smgt);
		if (list_empty(&slot->free_list) &&
			(slot->alloced_page_num < slot->page_num)) {
			DBG_LOG("add to  free %p, %d,%d,%d\n",
				slot, slot->page_num,
				slot->alloced_page_num, ret);
			list_add_tail(&slot->free_list, &smgt->free_list);
		}
		codec_mm_list_unlock(smgt);
	}

	return 0;
}

static int codec_mm_page_alloc_from_one_pages(phy_addr_type *pages, int num)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	int neednum = num;
	int alloced = 0;

	while (neednum > 0) {	/*one page  alloc */
		void *vpage = (void *)__get_free_page(GFP_KERNEL);
		ulong page;
		page_sid_type sid;

		if (vpage != NULL) {
			page = virt_to_phys(vpage);
			sid = ONE_PAGE_SID;
			page |= sid;
			pages[alloced++] = page;
			neednum--;
		} else {
			/*can't alloced memofy from ONEPAGE alloc */
			WAR_LOG("Out of memory OnePage alloc =%d,%d\n",
				alloced, num);
			break;
		}
	}
	codec_mm_list_lock(smgt);
	smgt->total_page_num += alloced;
	smgt->one_page_cnt += alloced;
	smgt->alloced_page_num += alloced;
	codec_mm_list_unlock(smgt);
	return alloced;
}

static int codec_mm_page_alloc_from_slot(phy_addr_type *pages, int num)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	struct codec_mm_slot *slot = NULL;
	int alloced = 0;
	int neednum = num;
	int n;

	codec_mm_list_lock(smgt);
	if (list_empty(&smgt->free_list) &&
		(codec_mm_get_free_size() <	/*no codec mm */
			smgt->reserved_block_mm_M * SZ_1M) &&
			!smgt->support_from_slot_sys) {	/*no sys */
		codec_mm_list_unlock(smgt);
		return 0;
	}
	codec_mm_list_unlock(smgt);

	do {
		slot = NULL;
		if (smgt->total_page_num <= 0 ||	/*no codec mm. */
			smgt->alloced_page_num == smgt->total_page_num ||
			list_empty(&smgt->free_list)) {
			/*codec_mm_scatter_info_dump(NULL, 0); */
			slot = codec_mm_slot_alloc(0, 0);
			if (!slot) {
				/*
				   *ERR_LOG("can't alloc slot from system\n");
				 */
				break;
			}
		}
		codec_mm_list_lock(smgt);
		if (slot && slot->on_alloc_free != 0) {
			ERR_LOG("the slot on alloc/free1: %d\n",
				slot->on_alloc_free);
			slot = NULL;	/*slot used on another alloc. */
		}
		if (!slot && !list_empty(&smgt->free_list)) {
			slot = list_entry(smgt->free_list.next,
				struct codec_mm_slot, free_list);
			if (!slot)
				ERR_LOG("ERROR!!!!.slot is NULL!!!!\n");
			else if (slot->on_alloc_free != 0) {
				ERR_LOG("the slot on alloc/free: %d\n",
					slot->on_alloc_free);
				slot = NULL;
			}
		}
		if (slot && slot->on_alloc_free == 0) {	/*del from free list. */
			slot->on_alloc_free++;
			list_del_init(&slot->free_list);
		} else {
			slot = NULL;
		}
		codec_mm_list_unlock(smgt);
		if (slot) {
			n = codec_mm_slot_alloc_pages(slot,
				&pages[alloced], neednum);
			codec_mm_list_lock(smgt);
			slot->on_alloc_free--;	/*alloc use end */
			if (slot->alloced_page_num < slot->page_num &&
				list_empty(&smgt->free_list)) {
				DBG_LOG("slot have free: %p, t:%d,a:%d,%d\n",
					slot, slot->page_num,
					slot->alloced_page_num, alloced);
				list_add_tail(&slot->free_list,
					&smgt->free_list);
				/*no free now, del from free list.,*/
				   /*and init to empty */
			}
			codec_mm_list_unlock(smgt);
			if (n > 0) {
				alloced += n;
				neednum -= n;
			} else {
				DBG_LOG("alloced fail neednum:%d n=%d\n",
					neednum, n);
				DBG_LOG("smgt->free_list.next:%p\n",
					smgt->free_list.next);
				DBG_LOG("smgt->free_list.prev:%p\n",
					smgt->free_list.prev);
				DBG_LOG("slot->free_list:%p\n",
					&slot->free_list);
				DBG_LOG("slot->free_list.next:%p\n",
					slot->free_list.next);
				/*codec_mm_dump_slot(slot, NULL, 0); */
				continue;	/*try next. */
			}
		} else {
			/*not alloced enough in block mode, try one page next */
			/*ERR_LOG("get free slot failed neednum: %d\n",
			   *neednum);
			 */
			break;
		}
	} while (neednum > 0);
	if (neednum > 0 && 0) {
		WAR_LOG("1can't alloc enough pages!!alloced=%d,need=%d\n",
			alloced, num);
		WAR_LOG("2can't alloc enough pages!!alloced=%d,need=%d\n",
			alloced, num);
	}
	codec_mm_list_lock(smgt);
	smgt->alloced_page_num += alloced;
	if (smgt->max_alloced < smgt->alloced_page_num)
		smgt->max_alloced = smgt->alloced_page_num;
	codec_mm_list_unlock(smgt);
	return alloced;
}


static int codec_mm_page_alloc_from_free_scatter(
	phy_addr_type *pages, int num)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	struct codec_mm_scatter *mms;
	int need = num;
	int alloced = 0;

	mms = smgt->cache_sc;
	if (!mms)
		return 0;
	codec_mm_scatter_lock(mms);
	alloced = min(mms->page_cnt, need);
	if (alloced > 0) {
		mms->page_cnt -= alloced;
		mms->page_tail -= alloced;
#if 1
		memcpy(pages, &mms->pages_list[mms->page_tail + 1],
			alloced * sizeof(phy_addr_type));
#else
	/*alloc from first*/
		memcpy(pages, &mms->pages_list[0],
			alloced * sizeof(phy_addr_type));
		memmove(&mms->pages_list[0],
			&mms->pages_list[alloced],
			mms->page_cnt * sizeof(phy_addr_type));
#endif
		memset(&mms->pages_list[mms->page_tail + 1], 0,
			alloced * sizeof(phy_addr_type));
	}
	codec_mm_list_lock(smgt);
	smgt->cached_pages = mms->page_cnt;
	codec_mm_list_unlock(smgt);
	codec_mm_scatter_unlock(mms);
	return alloced;
}

static int codec_mm_page_alloc_all_locked(
		phy_addr_type *pages, int num, int iscache)
{
	int alloced = 0;
	int can_from_scatter = iscache ? 0 : 1;
	int can_from_slot = 1;
	int new_alloc;

	while (alloced < num) {
		new_alloc = 0;
		if (can_from_scatter) {
			new_alloc = codec_mm_page_alloc_from_free_scatter(
				pages + alloced,
				num - alloced);
			if (new_alloc <= 0)
				can_from_scatter = 0;
		} else if (can_from_slot) {
			new_alloc = codec_mm_page_alloc_from_slot(
				pages + alloced,
				num - alloced);
			if (new_alloc <= 0)
				can_from_slot = 0;
		} else if (!codec_mm_video_tvp_enabled()) {
			new_alloc = codec_mm_page_alloc_from_one_pages(
				pages + alloced,
				num - alloced);
			if (new_alloc <= 0)
				break;
		} else {
			break;
		}
		alloced += new_alloc;
	}
	return alloced;
}

static int codec_mm_pages_free_to_scatter(
	struct codec_mm_scatter *src_mms)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	struct codec_mm_scatter *dst_mms;
	int moved = 0;
	int left;

	if (src_mms->page_used >= src_mms->page_cnt)
		return -1;	/*no need free. */
	dst_mms = smgt->cache_sc;
	if (!dst_mms)
		return 0;
	codec_mm_scatter_lock(dst_mms);
	moved = min(src_mms->page_cnt - src_mms->page_used,
		dst_mms->page_max_cnt - dst_mms->page_cnt);
	left = src_mms->page_cnt - moved;
	if (moved > 0) {
		memcpy(&dst_mms->pages_list[dst_mms->page_tail + 1],
			&src_mms->pages_list[left],
			moved * sizeof(phy_addr_type));
		memset(&src_mms->pages_list[left], 0,
			sizeof(phy_addr_type) * moved);
	}
	dst_mms->page_cnt += moved;
	dst_mms->page_tail += moved;
	src_mms->page_cnt -= moved;
	src_mms->page_tail -= moved;
	codec_mm_list_lock(smgt);
	smgt->cached_pages = dst_mms->page_cnt;
	codec_mm_list_unlock(smgt);
	codec_mm_scatter_unlock(dst_mms);
	return moved;
}

#if 0

/*
*return:
*1:for valid;
*0:no valid mms.
*must check in
*codec_mm_list_lock
*/
static int codec_mm_scatter_valid_check_inlock(struct codec_mm_scatter *mms)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	struct list_head *pos, *to_check_list;

	if (!mms)
		return 0;
	to_check_list = &mms->list;
	if (list_empty(&smgt->scatter_list))
		return 0;
	list_for_each(pos, &smgt->scatter_list) {
		if (pos == to_check_list)
			return 1;
	}
	return 0;
}
#endif

/*
*free one page in mms;
*/
static int codec_mm_scatter_free_page_id_locked(
	struct codec_mm_scatter *mms, int id)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	page_sid_type sid;
	int ret;

	if (INVALID_ID(mms, id))
		return CODEC_MM_S_ERR(1);
	sid = PAGE_SID_OF_MMS(mms, id);
	if (!VALID_SID(sid))
		return CODEC_MM_S_ERR(2);
	if (SID_OF_ONEPAGE(sid)) {
		ulong phy_addr = PAGE_ADDR_OF_MMS(mms, id);

		free_page((unsigned long)phys_to_virt(phy_addr));
		smgt->one_page_cnt--;
		smgt->total_page_num--;
		if (id == mms->page_tail)
			mms->page_tail--;
		mms->page_cnt--;
		return 0;
	}
	ret = codec_mm_page_free_to_slot(sid, PAGE_ADDR_OF_MMS(mms, id));
	if (!ret) {
		mms->page_cnt--;
		mms->page_tail--;
	}
	return ret;
}

/*
*free one page in mms;
*/
static int codec_mm_scatter_free_pages_in_locked(struct codec_mm_scatter *mms,
	int start_id)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	int i;
	int ret;
	int id = start_id;
	int freeNum = 0;
	int not_continue_print = 1;

	for (i = mms->page_tail; i >= id; i--) {
		ret = codec_mm_scatter_free_page_id_locked(mms, i);
		if (ret < 0) {
			if (not_continue_print) {
				ERR_LOG("page free error.%d,id=%d, addr:%d\n",
					ret, i,
					(int)mms->pages_list[i]);
				codec_mm_dump_scatter(mms, NULL, 0);
			}
			not_continue_print = 0;
		} else {
			not_continue_print = 1;
		}
		freeNum++;
		mms->pages_list[i] = (phy_addr_type) 0;
	}
	codec_mm_list_lock(smgt);
	smgt->alloced_page_num -= freeNum;
	codec_mm_list_unlock(smgt);
	return 0;
}

/*
*free pages from id(include id);
*/
static int codec_mm_scatter_free_tail_pages_in(
	struct codec_mm_scatter *mms,
	int start_free_id,
	int fast)
{
	int id = start_free_id;

	if (!mms || id < 0 || id >= mms->page_cnt || mms->page_tail < 0) {
		if (mms)
			ERR_LOG("free mm scatters error id %d,page_cnt=%d\n",
				id, mms->page_cnt);
		return -1;
	}
	codec_mm_scatter_lock(mms);
	mms->page_used = start_free_id;

	if (fast == 1) {
		codec_mm_scatter_unlock(mms);
		return 0;
	}
	if (fast == 2 || fast == 3) {
		codec_mm_pages_free_to_scatter(mms);
		if (fast == 2 || mms->page_used == mms->page_cnt) {
			codec_mm_scatter_unlock(mms);
			return 0;
		}
	}
	codec_mm_scatter_free_pages_in_locked(mms, start_free_id);
	codec_mm_scatter_unlock(mms);
	return 0;
}

int codec_mm_scatter_free_tail_pages(struct codec_mm_scatter *mms,
	int start_free_id)
{
	int ret = 0;

	if (start_free_id < mms->page_cnt)
		ret = codec_mm_scatter_free_tail_pages_in(mms,
			start_free_id, 0);
	return ret;
}
EXPORT_SYMBOL(codec_mm_scatter_free_tail_pages);

int codec_mm_scatter_free_tail_pages_fast(struct codec_mm_scatter *mms,
	int start_free_id)
{
	int ret = 0;

	if (start_free_id < mms->page_cnt)
		ret = codec_mm_scatter_free_tail_pages_in(mms,
			start_free_id, 2);
	codec_mm_schedule_delay_work(100, 0);
	return ret;
}
EXPORT_SYMBOL(codec_mm_scatter_free_tail_pages_fast);

int codec_mm_scatter_free_unused_pages(struct codec_mm_scatter *mms)
{
	int ret = 0;

	if (mms->page_used < mms->page_cnt)
		ret = codec_mm_scatter_free_tail_pages_in(mms,
			mms->page_used, 3);
	return ret;
}
EXPORT_SYMBOL(codec_mm_scatter_free_unused_pages);

/*free all pages only
*don't free scatter
*/
int codec_mm_scatter_free_all_pages(struct codec_mm_scatter *mms)
{
	int ret;

	ret = codec_mm_scatter_free_tail_pages(mms, 0);
	return ret;
}
EXPORT_SYMBOL(codec_mm_scatter_free_all_pages);

static inline int codec_mm_scatter_map_add_locked(
	struct codec_mm_scatter *mms)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	int i;

	for (i = 0; i < MAX_SC_LIST; i++) {
		if (smgt->scmap[i] == NULL) {
			smgt->scmap[i] = mms;
			return i;
		}
	}
	return -1;
}

static inline int codec_mm_scatter_map_del_locked(
	struct codec_mm_scatter *mms)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	int i;

	for (i = 0; i < MAX_SC_LIST; i++) {
		if (smgt->scmap[i] == mms) {
			smgt->scmap[i] = NULL;
			return i;
		}
	}
	return 0;
}

int codec_mm_scatter_valid_locked(struct codec_mm_scatter *mms)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	int i;
	int valid = 0;

	for (i = 0; i < MAX_SC_LIST; i++) {
		if (smgt->scmap[i] == mms) {
			valid = 1;
			break;
		}
	}
	return valid;
}
EXPORT_SYMBOL(codec_mm_scatter_valid_locked);

/*free scatter's all */
static int codec_mm_scatter_free_on_nouser(struct codec_mm_scatter *mms)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	int ret = 0;
	int free;

	codec_mm_scatter_lock(mms);
	codec_mm_list_lock(smgt);
	ret = atomic_read(&mms->user_cnt);
	if (ret > 0) {
		codec_mm_list_unlock(smgt);
		codec_mm_scatter_unlock(mms);
		/*>0 have another user. */
		/*pr_err("ERROR, scatter is not free.cnt:%d\n", ret); */
		return 0;
	}
	/*to free now */
	free = mms->page_cnt;
	if (!list_empty(&mms->list))
		list_del(&mms->list);
	smgt->scatters_cnt--;
	codec_mm_scatter_map_del_locked(mms);
	codec_mm_list_unlock(smgt);
	codec_mm_scatter_unlock(mms);
	if (mms->page_cnt > 0)
		ret = codec_mm_scatter_free_tail_pages_in(mms, 0, 0);
	if (free >= 256 &&
		(smgt->try_alloc_in_sys_page_cnt <
			smgt->try_alloc_in_sys_page_cnt_max) &&
		(smgt->total_page_num < (1024 * 1024 * 32 >> PAGE_SHIFT))) {
		smgt->try_alloc_in_sys_page_cnt *= 2;
		if (!smgt->support_from_slot_sys) {
			smgt->support_from_slot_sys =
				smgt->enable_slot_from_sys;
		}
	}

	SC_FREE(mms);
	return ret;
}

/*
*mask for other use it.
*/
static int codec_mm_scatter_inc_user_in(struct codec_mm_scatter *mms,
	int cnt)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	int ret = -1;
	int old_user;

	if (!mms)
		return -1;
	codec_mm_list_lock(smgt);
	if (!codec_mm_scatter_valid_locked(mms)) {
		codec_mm_list_unlock(smgt);
		return -1;
	}
	old_user = atomic_read(&mms->user_cnt);
	if (old_user >= 0) {
		ret = atomic_add_return(cnt, &mms->user_cnt);
		if (old_user == 1)
			mms->tofree_jiffies = 0;
	}
	codec_mm_list_unlock(smgt);
	return ret <= 0 ? ret : 0;	/*must add before user cnt >= 0 */
}

/*mask scatter's to free.*/
static int codec_mm_scatter_dec_user_in(struct codec_mm_scatter *mms,
	int delay_free_ms, int cnt)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	int after_users = 1;

	if (!mms)
		return -1;
	codec_mm_list_lock(smgt);
	if (!codec_mm_scatter_valid_locked(mms)) {
		codec_mm_list_unlock(smgt);
		return -1;
	}
	if (atomic_read(&mms->user_cnt) >= 1) {
		after_users = atomic_sub_return(cnt, &mms->user_cnt);
		if (after_users == 0) {
			/*is free time */
			if (delay_free_ms > 0)
				mms->tofree_jiffies = jiffies +
					delay_free_ms * HZ / 1000;
			else {
				mms->page_used = 0;
				mms->tofree_jiffies = 0;
			}
		}
	}
	codec_mm_list_unlock(smgt);
	if (after_users == 0)
		codec_mm_schedule_delay_work(0, 1);
	return 0;
}

/*
*maybe a render/sink.video/osd/
*/
int codec_mm_scatter_inc_for_keeper(void *sc_mm)
{
	struct codec_mm_scatter *mms = sc_mm;

	return codec_mm_scatter_inc_user_in(mms, 100);
}
EXPORT_SYMBOL(codec_mm_scatter_inc_for_keeper);

/*
*maybe a render/sink.video/osd/
*/
int codec_mm_scatter_dec_keeper_user(void *sc_mm, int delay_ms)
{
	struct codec_mm_scatter *mms = sc_mm;

	return codec_mm_scatter_dec_user_in(mms, delay_ms, 100);
}
EXPORT_SYMBOL(codec_mm_scatter_dec_keeper_user);

int codec_mm_scatter_dec_owner_user(void *sc_mm, int delay_ms)
{
	struct codec_mm_scatter *mms = sc_mm;
	int ret = codec_mm_scatter_dec_user_in(mms, delay_ms, 1000);

	if (ret)
		ERR_LOG("dec_owner_user error %p\n", mms);
	return ret;
}
EXPORT_SYMBOL(codec_mm_scatter_dec_owner_user);

/*
*max pages:
*want pages now,
*maybe:
*	max pages == support 4k,need pages;
*	page num = current size need pages;
*/
struct codec_mm_scatter *codec_mm_scatter_alloc_new(int max_page, int page_num)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	struct codec_mm_scatter *mms;
	int ret;

	if (max_page < page_num)
		return NULL;

	mms = SC_ALLOC(sizeof(struct codec_mm_scatter) + sizeof(phy_addr_type) *
		max_page, GFP_KERNEL);
	if (!mms) {
		ERR_LOG("no enough for mm scatter!!!!\n");
		return NULL;
	}
	memset(mms, 0, sizeof(struct codec_mm_scatter));
	mms->pages_list = (phy_addr_type *) (mms + 1);
	mms->page_max_cnt = max_page;
	INIT_LIST_HEAD(&mms->list);
	memset(mms->pages_list, 0, sizeof(phy_addr_type) * max_page);
	mms->page_cnt = 0;
	mms->page_tail = -1;
	atomic_set(&mms->user_cnt, 0);
	mutex_init(&mms->mutex);
	if (page_num > 0) {
		ret = codec_mm_page_alloc_all_locked(mms->pages_list, page_num,
			mms == smgt->cache_sc);
		if (ret <= 0)
			goto error;
		mms->page_cnt = ret;
		mms->page_tail = mms->page_cnt - 1;
	}
	atomic_set(&mms->user_cnt, 1000);
	codec_mm_list_lock(smgt);
	mms->page_used = mms->page_cnt;
	list_add_tail(&mms->list, &smgt->scatter_list);
	smgt->scatters_cnt++;
	codec_mm_scatter_map_add_locked(mms);
	codec_mm_list_unlock(smgt);
	return mms;
error:
	codec_mm_scatter_free_on_nouser(mms);
	return NULL;
}
EXPORT_SYMBOL(codec_mm_scatter_alloc_new);

/*
*max pages:
*want pages now,
*maybe:
*	max pages == support 4k,need pages;
*	page num = current size need pages;
*/
struct codec_mm_scatter *codec_mm_scatter_alloc(int max_page, int page_num)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	struct codec_mm_scatter *mms, *alloced_mms;
	struct list_head *pos, *next;
	int ret;
	u64 startus;

	startus = codec_mm_get_current_us();
	alloced_mms = NULL;
	codec_mm_list_lock(smgt);
	if (!list_empty(&smgt->scatter_list)) {	/*try find a free scatter. */
		pos = smgt->scatter_list.prev;	/*free on prev. */
		while (pos != &smgt->scatter_list) {
			next = pos->prev;
			mms = list_entry(pos, struct codec_mm_scatter, list);
			if (mms->page_max_cnt >= max_page &&
				atomic_read(&mms->user_cnt) == 0) {
				if (atomic_add_return(1000,
						&mms->user_cnt) == 1000) {
					mms->page_used = mms->page_cnt;
					alloced_mms = mms;
					break;
				} else
					atomic_sub(1000, &mms->user_cnt);
			}
			pos = next;
		}
	}
	codec_mm_list_unlock(smgt);
	if (!alloced_mms) {
		/*
		*just alloc mms first,
		*alloc pages later.
		*/
		alloced_mms = codec_mm_scatter_alloc_new(max_page, 0);
	}
	if (alloced_mms) {
		ret = codec_mm_scatter_alloc_want_pages_in(alloced_mms,
			page_num);
		if (ret < 0) {
			atomic_sub(1000, &alloced_mms->user_cnt);
			return NULL;
		}
		/*pr_info("reused old mms! %p\n", alloced_mms);*/
		codec_mm_update_alloc_time(startus);
		return alloced_mms;
	}
	return NULL;
}
EXPORT_SYMBOL(codec_mm_scatter_alloc);

static int codec_mm_scatter_alloc_want_pages_in(
		struct codec_mm_scatter *mms,
		int want_pages)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	int ret;

	if (want_pages > mms->page_max_cnt)
		return CODEC_MM_S_ERR(100);
	codec_mm_scatter_lock(mms);
	mms->page_used = want_pages;
	if (want_pages > mms->page_cnt) {
		ret = codec_mm_page_alloc_all_locked(
				&mms->pages_list[mms->page_tail + 1],
					want_pages - mms->page_cnt,
				mms == smgt->cache_sc);
		if (ret <= 0) {
			codec_mm_scatter_unlock(mms);
			ERR_LOG("can't alloc want pages %d\n", want_pages);
			return ret;
		}
		mms->page_cnt += ret;
		mms->page_tail += ret;
	}
	if (mms == smgt->cache_sc) {
		codec_mm_list_lock(smgt);
		if (smgt->cache_sc)/*update cache pages*/
			smgt->cached_pages = smgt->cache_sc->page_cnt;
		codec_mm_list_unlock(smgt);
	}
	codec_mm_scatter_unlock(mms);
	if (smgt->cached_pages < smgt->keep_size_PAGE / 2) {
		/*try alloc more cache.*/
		codec_mm_schedule_delay_work(0, 1);
	}
	return 0;
}

int codec_mm_scatter_alloc_want_pages(
		struct codec_mm_scatter *mms,
		int want_pages)
{
	int ret;
	u64 startus;

	startus = codec_mm_get_current_us();
	ret = codec_mm_scatter_alloc_want_pages_in(mms, want_pages);
	codec_mm_update_alloc_time(startus);
	return ret;
}
EXPORT_SYMBOL(codec_mm_scatter_alloc_want_pages);

int codec_mm_free_all_free_slots(void)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	struct codec_mm_slot *slot, *to_free;

	do {
		to_free = NULL;
		codec_mm_list_lock(smgt);
		{
			struct list_head *header, *list;

			header = &smgt->free_list;
			list = header->prev;
			while (list != header) {
				slot = list_entry(list, struct codec_mm_slot,
					free_list);
				if (slot->alloced_page_num == 0) {
					list_del_init(&slot->free_list);
					to_free = slot;
					break;
				}
				list = list->prev;
			};
		}
		codec_mm_list_unlock(smgt);
		if (!to_free)
			break;
		codec_mm_slot_free(to_free);
	} while (1);
	return 0;
}
EXPORT_SYMBOL(codec_mm_free_all_free_slots);

int codec_mm_scatter_info_dump(void *buf, int size)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	char *pbuf = buf;
	char sbuf[512];
	int tsize = 0;
	int s;
	int n;

	if (!pbuf)
		pbuf = sbuf;

#define BUFPRINT(args...) \
		do {\
			s = sprintf(pbuf, args);\
			tsize += s;\
			pbuf += s; \
		} while (0)

	BUFPRINT("codec scattered memory info:\n");
	BUFPRINT("\ttotal size:%dM, %d Bytes,pages:%d\n",
			 (smgt->total_page_num << PAGE_SHIFT) / SZ_1M,
			 smgt->total_page_num << PAGE_SHIFT,
			 smgt->total_page_num);
	n = smgt->alloced_page_num;
	BUFPRINT("\talloced size:%dM, %d Bypes,pages:%d\n",
			 (n << PAGE_SHIFT) / SZ_1M,
			 n << PAGE_SHIFT,
			 n);
	BUFPRINT("\tmax alloced:%d M | %d pages\n",
		(smgt->max_alloced << PAGE_SHIFT) / SZ_1M,
		smgt->max_alloced);
	BUFPRINT("\tscatter cached:%d M |%d pages\n",
		(smgt->cached_pages << PAGE_SHIFT) / SZ_1M,
		smgt->cached_pages);

	BUFPRINT("\talloc from sys size:%d\n",
		(smgt->alloc_from_sys_sc_cnt +
		smgt->one_page_cnt) << PAGE_SHIFT);

	BUFPRINT("\talloc from sys sc cnt:%d\n",
		smgt->alloc_from_sys_sc_cnt);
	BUFPRINT("\talloc from sys pages cnt:%d pages\n",
		smgt->alloc_from_sys_page_cnt);
	BUFPRINT("\talloc from sys max pages cnt:%d pages\n",
		smgt->alloc_from_sys_max_page_cnt);
	BUFPRINT("\tscatter_task_run:%d\n",
		smgt->scatter_task_run_num);
	BUFPRINT("\tone_page_cnt:%d\n",
		smgt->one_page_cnt);
	BUFPRINT("\tcatters cnt:%d\n", smgt->scatters_cnt);
	BUFPRINT("\tslot cnt:%d\n", smgt->slot_cnt);
	BUFPRINT("\tcma alloc block size:%d\n",
		smgt->try_alloc_in_cma_page_cnt);
	BUFPRINT("\tsys alloc block size:%d\n",
			smgt->try_alloc_in_sys_page_cnt);
	BUFPRINT("\tdelay_free_on:%d\n",
			smgt->delay_free_on);
	BUFPRINT("\tdelay_free_on time:%lld jiffies\n",
			smgt->delay_free_timeout_jiffies64);
	BUFPRINT("\tcurrent time:%lld\n",
			get_jiffies_64());
	BUFPRINT("\talloc time max us:%d\n",
			smgt->alloc_max_us);
	BUFPRINT("\talloc cnt:%d step:%d:%d:%d:%d:%d:%d:%d\n",
			smgt->alloc_cnt,
			smgt->alloc_10us_less_cnt,
			smgt->alloc_10_50us_cnt,
			smgt->alloc_50_100us_cnt,
			smgt->alloc_100_1000us_cnt,
			smgt->alloc_1_10ms_cnt,
			smgt->alloc_10_100ms_cnt,
			smgt->alloc_100ms_up_cnt
			);
	{
		int average_timeus = smgt->alloc_cnt == 0 ?
			0 : (int)(smgt->alloc_total_us/smgt->alloc_cnt);
		BUFPRINT("\talloc time average us:%d\n",
			average_timeus);
	}

#undef BUFPRINT
	if (!buf)
		INFO_LOG("%s", sbuf);
	return tsize;
}
EXPORT_SYMBOL(codec_mm_scatter_info_dump);

int codec_mm_dump_slot(struct codec_mm_slot *slot, void *buf, int size)
{
	char *pbuf = buf;
	char sbuf[512];
	int tsize = 0;
	int s;
	int i;
	int sum;
	char bits_incharNhalf[] = { 0, 1, 1, 2, 1, 2, 2, 3,
		1, 2, 2, 3, 2, 2, 3, 4, 1, 2};

	if (!pbuf)
		pbuf = sbuf;

#define BUFPRINT(args...) \
		do {\
			s = sprintf(pbuf, args);\
			tsize += s;\
			pbuf += s; \
		} while (0)

	BUFPRINT("slot info:%p\n", slot);
	BUFPRINT("\tfrom:%s\n",
		(slot->from_type == SLOT_FROM_CODEC_MM ? "codec_mm" :
			"sys"));
	BUFPRINT("\tbase addr range:%p<-->%p\n",
			 (void *)slot->phy_addr,
			 (void *)(slot->phy_addr +
			 (slot->page_num << PAGE_SHIFT)
					 - 1));
	BUFPRINT("\tpage_num:%d\n", slot->page_num);
	BUFPRINT("\talloced:%d,free:%d\n", slot->alloced_page_num,
		slot->page_num - slot->alloced_page_num);
	BUFPRINT("\tnext bit:%d\n", slot->next_bit);
	BUFPRINT("\tsid:%x\n", slot->sid);
	BUFPRINT("\troot:%d\n", slot->isroot);
	sum = 0;
	for (i = 0; i < slot->pagemap_size; i++) {
		int c = ((unsigned char *)slot->pagemap)[i];

		sum += bits_incharNhalf[c & 0xf] + bits_incharNhalf[c >> 4];
	}
	BUFPRINT("\tbitmap.setbits.sum:%d\n", sum);
	BUFPRINT("\tbitmap");
	i = 0;
	while (i < slot->pagemap_size && i < 16)
		BUFPRINT(":%02x", ((char *)slot->pagemap)[i++]);
	BUFPRINT("\n");
#undef BUFPRINT
	if (!buf)
		INFO_LOG("%s", sbuf);
	return 0;

}
EXPORT_SYMBOL(codec_mm_dump_slot);

int codec_mm_dump_all_slots(void)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	struct codec_mm_slot *slot, *fslot;
	int total_pages = 0;
	int alloced_pages = 0;
	int slot_cnt = 0;
	int i;

	codec_mm_list_lock(smgt);
	INFO_LOG("start dump all slots!\n");
	for (i = 0; i < MAX_SID; i++) {
		fslot = smgt->slot_list_map[i];
		if (fslot) {
			codec_mm_dump_slot(fslot, NULL, 0);
			slot_cnt++;
			total_pages += fslot->page_num;
			alloced_pages += fslot->alloced_page_num;
			if (!list_empty(&fslot->sid_list)) {
				slot = list_entry(fslot->sid_list.next,
					struct codec_mm_slot, sid_list);
				while (slot != fslot) {
					codec_mm_dump_slot(slot, NULL, 0);
					slot_cnt++;
					total_pages += slot->page_num;
					alloced_pages += slot->alloced_page_num;
					slot = list_entry(slot->sid_list.next,
						struct codec_mm_slot, sid_list);
				}
			}
		}

	}
	codec_mm_list_unlock(smgt);
	INFO_LOG("end dump, slot cnt:%d total pages:%d, free:%d\n",
		slot_cnt,
		total_pages,
		total_pages - alloced_pages);
	return 0;
}
EXPORT_SYMBOL(codec_mm_dump_all_slots);

int codec_mm_dump_all_hash_table(void)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	struct codec_mm_slot *slot, *fslot;
	int i;
	int total_pages = 0;
	int alloced_pages = 0;

	INFO_LOG("start dump sid hash table!\n");
	codec_mm_list_lock(smgt);
	for (i = 0; i < MAX_SID; i++) {
		int cnt = 0;
		int pages = 0;
		int alloced = 0;

		fslot = smgt->slot_list_map[i];
		if (fslot) {
			cnt++;
			pages += fslot->page_num;
			alloced += fslot->alloced_page_num;
			if (!list_empty(&fslot->sid_list)) {
				slot = list_entry(fslot->sid_list.next,
					struct codec_mm_slot, sid_list);
				while (slot != fslot) {
					cnt++;
					pages += slot->page_num;
					alloced += slot->alloced_page_num;
					slot = list_entry(slot->sid_list.next,
						struct codec_mm_slot, sid_list);
				}
			}
		}
		if (cnt > 0) {
			total_pages += pages;
			alloced_pages += alloced;
			INFO_LOG(
			"\tSID(%d):\tslots:%d,\tpages:%d:\tfree pages:%d\n",
			i, cnt, pages, pages - alloced);
		}
	}
	codec_mm_list_unlock(smgt);
	INFO_LOG("end dump sid hash table, total pages:%d, free:%d\n",
		total_pages, total_pages - alloced_pages);
	return 0;
}
EXPORT_SYMBOL(codec_mm_dump_all_hash_table);

int codec_mm_dump_free_slots(void)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	struct codec_mm_slot *slot;
	int total_pages = 0;
	int alloced_pages = 0;

	INFO_LOG("dump all free slots:\n");
	codec_mm_list_lock(smgt);
	if (!list_empty(&smgt->free_list)) {
		struct list_head *header, *list;

		header = &smgt->free_list;
		list = header->prev;
		while (list != header) {
			slot = list_entry(list, struct codec_mm_slot,
				free_list);
			codec_mm_dump_slot(slot, NULL, 0);
			total_pages += slot->page_num;
			alloced_pages += slot->alloced_page_num;
			list = list->prev;
		};
	}
	codec_mm_list_unlock(smgt);
	INFO_LOG("end all free slots: total pages:%d, freed:%d\n",
		total_pages,
		total_pages - alloced_pages);
	return 0;
}
EXPORT_SYMBOL(codec_mm_dump_free_slots);

int codec_mm_dump_scatter(struct codec_mm_scatter *mms, void *buf, int size)
{
	char *pbuf = buf;
	char sbuf[512];
	int tsize = 0;
	int s;
	int i;

	if (!pbuf)
		pbuf = sbuf;

#define BUFPRINT(args...) \
		do {\
			s = sprintf(pbuf, args);\
			tsize += s;\
			pbuf += s; \
		} while (0)

	BUFPRINT("scatter info:%p\n", mms);
	BUFPRINT("\tsize:%d\n", (int)(mms->page_cnt * PAGE_SIZE));
	BUFPRINT("\tmax:%d\n", mms->page_max_cnt);
	BUFPRINT("\tpage_cnt:%d\n", mms->page_cnt);
	BUFPRINT("\tpage_used:%d\n", mms->page_used);
	BUFPRINT("\tpage_tail:%d\n", mms->page_tail);
	BUFPRINT("\tuser_cnt:%d\n", atomic_read(&mms->user_cnt));
	BUFPRINT("\ttofree_jiffies:%ld\n", mms->tofree_jiffies);

	i = 0;
	while (i < mms->page_cnt && i < 16)
		BUFPRINT(":%x", (u32) mms->pages_list[i++]);
	BUFPRINT("\n");
#undef BUFPRINT
	if (!buf)
		INFO_LOG("%s", sbuf);

	return 0;

}
EXPORT_SYMBOL(codec_mm_dump_scatter);

int codec_mm_dump_all_scatters(void)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	struct codec_mm_scatter *mms;
	struct list_head *pos, *tmp;

	INFO_LOG("start dump all scatters!\n");
	codec_mm_list_lock(smgt);
	do {
		if (list_empty(&smgt->scatter_list))
			break;
		list_for_each_safe(pos, tmp, &smgt->scatter_list) {
			mms = list_entry(pos, struct codec_mm_scatter, list);
			codec_mm_dump_scatter(mms, 0, 0);
		}
	} while (0);
	INFO_LOG("start dump free scatters!\n");
	if (smgt->cache_sc)
		codec_mm_dump_scatter(smgt->cache_sc, 0, 0);
	codec_mm_list_unlock(smgt);
	INFO_LOG("finished dump all scatters!\n");
	return 0;
}
EXPORT_SYMBOL(codec_mm_dump_all_scatters);

struct sc_configs {
	int id;
	const char *name;
};

enum config_id {
	ID0_KEEP_SIZE,
	ID1_RES_BLK_SIZEM,
	ID2_ONCE_CMA_SIZEM,
	ID3_MAX_SYS_PAGES,
	ID4_MIN_SYS_PAGES,
	ID5_SLOT_FROM_SYS,
	ID6_NO_CACHE_SIZE,
	IDX_MAX,
};
struct sc_configs sc_global_config[] = {
	{ID0_KEEP_SIZE, "keep_size"},
	{ID1_RES_BLK_SIZEM, "res_blk_size"},
	{ID2_ONCE_CMA_SIZEM, "once_cma_size"},
	{ID3_MAX_SYS_PAGES, "max_sys_pages"},
	{ID4_MIN_SYS_PAGES, "min_sys_pages"},
	{ID5_SLOT_FROM_SYS, "enable_slot_from_sys"},
	{ID6_NO_CACHE_SIZE, "no_cache_size_M"},
};

static int codec_mm_scatter_mgt_get_config_in(int id)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();

	switch (id) {
	case ID0_KEEP_SIZE:
		return (smgt->keep_size_PAGE << PAGE_SHIFT) / SZ_1M;
	case ID1_RES_BLK_SIZEM:
		return smgt->reserved_block_mm_M;
	case ID2_ONCE_CMA_SIZEM:
		return smgt->try_alloc_in_cma_page_cnt;
	case ID3_MAX_SYS_PAGES:
		return smgt->try_alloc_in_sys_page_cnt_max;
	case ID4_MIN_SYS_PAGES:
		return smgt->try_alloc_in_sys_page_cnt_min;
	case ID5_SLOT_FROM_SYS:
		return smgt->enable_slot_from_sys;
	case ID6_NO_CACHE_SIZE:
		return smgt->no_cache_size_M;
	default:
		return 0;
	}
	return -1;
}

static int codec_mm_scatter_mgt_set_config_in(int id, int newset)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	int val;

	switch (id) {
	case ID0_KEEP_SIZE:
		val = newset * (SZ_1M >> PAGE_SHIFT);
		smgt->keep_size_PAGE = val;
		break;
	case ID1_RES_BLK_SIZEM:
		smgt->reserved_block_mm_M = newset;
		break;
	case ID2_ONCE_CMA_SIZEM:
		smgt->try_alloc_in_cma_page_cnt = newset;
		break;
	case ID3_MAX_SYS_PAGES:
		smgt->try_alloc_in_sys_page_cnt_max = newset;
		break;
	case ID4_MIN_SYS_PAGES:
		smgt->try_alloc_in_sys_page_cnt_min = newset;
		break;
	case ID5_SLOT_FROM_SYS:
		smgt->enable_slot_from_sys = newset;
		smgt->support_from_slot_sys = newset;
		break;
	case ID6_NO_CACHE_SIZE:
		smgt->no_cache_size_M = newset;
		break;
	default:
		return -1;
	}
	return 0;
}

int codec_mm_scatter_mgt_get_config(char *buf)
{
	size_t ret = 0;
	int i;

	for (i = 0; i < IDX_MAX; i++) {
		ret += sprintf(buf + ret,
			"scatter:%s:%d\n",
			sc_global_config[i].name,
			codec_mm_scatter_mgt_get_config_in(i));
	}
	return ret;
}
EXPORT_SYMBOL(codec_mm_scatter_mgt_get_config);

int codec_mm_scatter_mgt_set_config(const char *buf, size_t size)
{
	size_t ret = 0;
	char *str;
	int i;
	int val;

	for (i = 0; i < IDX_MAX; i++) {
		str = strstr(buf, sc_global_config[i].name);
		if (!str)
			continue;	/*to next. */
		str += strlen(sc_global_config[i].name);
		if (str[0] != ':' || str[1] == '\0')
			continue;	/*to next. */
		ret = sscanf(str, ":%d", &val);
		if (ret == 1) {
			codec_mm_scatter_mgt_set_config_in(i, val);
			return size;
		}
	}
	pr_info("unknown cmd %s\n", buf);
	return -1;
}
EXPORT_SYMBOL(codec_mm_scatter_mgt_set_config);

int codec_mm_scatter_mgt_delay_free_swith(int on,
	int delay_ms,
	int wait_size_M)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();

	codec_mm_list_lock(smgt);
	if (on) {
		smgt->delay_free_on++;
		smgt->delay_free_timeout_jiffies64 =
			get_jiffies_64() + delay_ms * HZ/1000;
	} else {
		smgt->delay_free_on--;
		if (smgt->delay_free_on <= 0) {
			smgt->delay_free_on = 0;
			smgt->delay_free_timeout_jiffies64 =
				get_jiffies_64() + delay_ms * HZ/1000;
		}
	}
	codec_mm_list_unlock(smgt);
	if (on && wait_size_M > 0) {
		u64 start_time = get_jiffies_64();
		int try_max = 1000;

		smgt->force_cache_on = 1;
		smgt->force_cache_page_cnt = wait_size_M >> PAGE_SHIFT;
		smgt->delay_free_timeout_jiffies64 =
			get_jiffies_64() + 10000 * HZ/1000;
		codec_mm_schedule_delay_work(0, 1);/*start cache*/
		while (smgt->cached_pages < smgt->force_cache_page_cnt) {
			if (smgt->cache_sc &&
				(smgt->cached_pages >=
					smgt->cache_sc->page_max_cnt - 100)) {
				/*cache sc fulled.*/
				break;
			}
			if (try_max-- <= 0 || time_after64(get_jiffies_64(),
					start_time + HZ)) {
				break;
			}
			msleep(20);
		}
		pr_info("end: cached pages: %d, speed %d ms\n",
			smgt->cached_pages,
			(int)(get_jiffies_64() - start_time) * 1000/HZ);
		smgt->force_cache_on = 0;
		smgt->delay_free_timeout_jiffies64 =
			get_jiffies_64() + delay_ms * HZ/1000;
	} else if (on) {
		codec_mm_schedule_delay_work(0, 1);
	} else {
		codec_mm_schedule_delay_work(delay_ms, 0);
	}
	return 0;
}
EXPORT_SYMBOL(codec_mm_scatter_mgt_delay_free_swith);

static void codec_mm_scatter_cache_manage(struct codec_mm_scatter_mgt *smgt)
{
	struct codec_mm_scatter *mms;
	int alloced = 0;
	int total_free_page = smgt->total_page_num -
		smgt->alloced_page_num + smgt->cached_pages;

	if (smgt->delay_free_on > 0 && smgt->keep_size_PAGE > 0) {
		/*if alloc too much ,don't cache any more.*/
		if (smgt->no_cache_size_M > 0 &&
			(smgt->cached_pages <= smgt->keep_size_PAGE) &&
			(smgt->total_page_num >=
			 (smgt->no_cache_size_M * (SZ_1M >> PAGE_SHIFT)))) {
			/*have enough pages for most movies.*/
			  /*don't cache more.*/
		} else if ((smgt->cached_pages < smgt->keep_size_PAGE) ||
			(smgt->force_cache_on &&/*on star cache*/
			(smgt->cached_pages < smgt->force_cache_page_cnt))
		) {/*first 500ms ,alloc double.*/
			mms = smgt->cache_sc;
			if (mms) {
				int need;
				int once_alloc = 1000;/*once 4M*/

				if (smgt->force_cache_on) {
					once_alloc = 4000;
					if ((smgt->total_page_num -
						smgt->alloced_page_num) >
							once_alloc) {
						once_alloc =
							smgt->total_page_num -
							smgt->alloced_page_num;
					}
				}
				need = mms->page_cnt + once_alloc;
				if ((need - mms->page_cnt) > once_alloc)
					need = mms->page_cnt + once_alloc;
				if (need > smgt->force_cache_page_cnt)
					need = smgt->force_cache_page_cnt;
				if (need > mms->page_max_cnt)
					need = mms->page_max_cnt - 4;
				if (need > mms->page_cnt) {
					alloced =
					!codec_mm_scatter_alloc_want_pages_in(
						mms,
						need);
				} else {
					alloced = 0;
				}
			} else {
				alloced = 0;
			}
		/*int from_sys = codec_mm_get_free_size() < 16 * SZ_1M;*/
		/*alloced = !!codec_mm_slot_alloc(0, (from_sys ? 1 : 0));*/
		} else if ((smgt->cached_pages >
			(smgt->keep_size_PAGE + 1000)) &&
			time_after64(get_jiffies_64(),
			smgt->delay_free_timeout_jiffies64)) {
			/*wait time out can free.*/
			mms = smgt->cache_sc;
			if (mms) {	/*only free some 1M cache */
				int free_start = smgt->cached_pages - 256;

				if (free_start < smgt->keep_size_PAGE)
					free_start = smgt->keep_size_PAGE;
				codec_mm_scatter_free_tail_pages(mms,
					free_start);
			}
			codec_mm_free_all_free_slots();
			/*free some slots. */
		}
	} else if (smgt->delay_free_on <= 0 &&
			time_after64(get_jiffies_64(),
			smgt->delay_free_timeout_jiffies64)) {
		/*free all free pages, no delay needed.*/
		codec_mm_free_all_free_slots();
	}
	if (smgt->keep_size_PAGE > 0 && smgt->delay_free_on) {
		if (((smgt->force_cache_on ||
			  (total_free_page < smgt->keep_size_PAGE)) &&
			!codec_mm_video_tvp_enabled()) &&
			alloced) {/*if failed may deadlock...*/
			/*ignore keep on tvp mode.*/
			if (smgt->force_cache_on)
				codec_mm_schedule_delay_work(0, 1);
			else
				codec_mm_schedule_delay_work(10, 0);
		} else
			codec_mm_schedule_delay_work(100, 0);
	} else if (!smgt->delay_free_on && smgt->total_page_num > 0) {
		codec_mm_schedule_delay_work(100, 0);
	}
}

static int codec_mm_scatter_scatter_arrange(struct codec_mm_scatter_mgt *smgt)
{
	struct codec_mm_scatter *mms;
	struct codec_mm_scatter *first_free_mms = NULL;
	struct list_head *pos, *tmp;
	int n = 0;

	if (smgt->delay_free_on > 0 && !smgt->cache_sc) {
		/*no free scatter. */
		mms = codec_mm_scatter_alloc_new(16384, 0);
		if (mms) {
			codec_mm_list_lock(smgt);
			list_del_init(&mms->list);
			smgt->cache_sc = mms;
			codec_mm_list_unlock(smgt);
		}
	}
	if (smgt->delay_free_on <= 0 && smgt->cache_sc &&
		time_after64(get_jiffies_64(),
			smgt->delay_free_timeout_jiffies64)) {
		codec_mm_list_lock(smgt);
		mms = smgt->cache_sc;
		smgt->cache_sc = NULL;
		smgt->cached_pages = 0;
		codec_mm_list_unlock(smgt);
		codec_mm_scatter_dec_owner_user(mms, 0);
		codec_mm_scatter_free_on_nouser(mms);
	}

	codec_mm_list_lock(smgt);
	if (list_empty(&smgt->scatter_list)) {
		codec_mm_list_unlock(smgt);
		return 0;
	}
	list_for_each_safe(pos, tmp, &smgt->scatter_list) {
		mms = list_entry(pos, struct codec_mm_scatter, list);
		if (atomic_read(&mms->user_cnt) == 0 &&
			time_after(jiffies, mms->tofree_jiffies)) {
			if (!first_free_mms ||
				mms->tofree_jiffies <
				first_free_mms->tofree_jiffies)
				first_free_mms = mms;
		} else
			n++;
	}
	if (first_free_mms)
		list_move_tail(&mms->list, &smgt->scatter_list);
	codec_mm_list_unlock(smgt);

	return 0;
}

static int codec_mm_scatter_scatter_clear(
	struct codec_mm_scatter_mgt *smgt,
	int force)/*not check jiffies & cache*/
{
	struct codec_mm_scatter *mms, *to_free_mms, *less_page_mms;
	struct list_head *pos, *tmp;
	int to_free_mms_cnt = 0;

	codec_mm_list_lock(smgt);
	to_free_mms = NULL;
	less_page_mms = NULL;
	list_for_each_safe(pos, tmp, &smgt->scatter_list) {
		mms = list_entry(pos, struct codec_mm_scatter, list);
		if (atomic_read(&mms->user_cnt) == 0) {
			if (!to_free_mms ||
				(mms->tofree_jiffies <
					to_free_mms->tofree_jiffies))
				to_free_mms = mms;
			to_free_mms_cnt++;
		}
		if (!less_page_mms && mms->page_used < mms->page_cnt) {
			less_page_mms = mms;
			break;
		}
	}
	if ((to_free_mms != NULL) &&
		(((to_free_mms_cnt > 1 || !smgt->delay_free_on) &&
		time_after(jiffies, to_free_mms->tofree_jiffies)) ||
		force)) {	/*force== no checktimeer. */
		/*set to nagative for free now. */
		int cnt = atomic_sub_return(100000, &to_free_mms->user_cnt);

		if (cnt != -100000) {
			atomic_add(100000, &to_free_mms->user_cnt);
			to_free_mms = NULL;
		} else {
			list_del_init(&to_free_mms->list);
		}
	} else {
		to_free_mms = NULL;
	}
	codec_mm_list_unlock(smgt);
	if (to_free_mms != NULL)
		codec_mm_scatter_free_on_nouser(to_free_mms);
	if (less_page_mms && (less_page_mms != to_free_mms))
		codec_mm_scatter_free_unused_pages(less_page_mms);
	return (to_free_mms != NULL) || (less_page_mms != NULL);
}

/*
*clear all ignore any cache.
*
*return the total num alloced.
*0 is all freeed.
*N is have some pages not alloced.
*/
int codec_mm_scatter_free_all_ignorecache(void)
{
	struct codec_mm_scatter_mgt *smgt = codec_mm_get_scatter_mgt();
	int need_retry = 1;
	int retry_num = 0;
	mutex_lock(&smgt->monitor_lock);
	pr_info("force free all scatter ignorecache!\n");
	do {
		struct codec_mm_scatter *mms;
		/*clear cache first. */
		/*disabled free on first. */
		smgt->delay_free_on = 0;
		codec_mm_list_lock(smgt);
		mms = smgt->cache_sc;
		smgt->cache_sc = NULL;
		smgt->cached_pages = 0;
		codec_mm_list_unlock(smgt);
		if (mms) {
			codec_mm_scatter_dec_owner_user(mms, 0);
			codec_mm_scatter_free_on_nouser(mms);
		}
		/*alloced again on timer?*/
		  /* check again. */
	} while (smgt->cache_sc != NULL);
	do {
		need_retry = codec_mm_scatter_scatter_clear(smgt, 1);
	} while ((smgt->scatters_cnt > 0) && (retry_num++ < 1000));
	if (need_retry || smgt->scatters_cnt > 0) {
		pr_info("can't free all scatter, because some have used!!\n");
		codec_mm_dump_all_scatters();
	}
	codec_mm_free_all_free_slots();
	if (smgt->total_page_num > 0) {
		/*have some not free,dump tables for debug */
		pr_info("Some slots have not free!!\n\n");
		codec_mm_dump_all_hash_table();
	}
	mutex_unlock(&smgt->monitor_lock);
	return smgt->total_page_num;
}
EXPORT_SYMBOL(codec_mm_scatter_free_all_ignorecache);

static void codec_mm_scatter_monitor(struct work_struct *work)
{
	struct codec_mm_scatter_mgt *smgt = container_of(work,
					struct codec_mm_scatter_mgt,
					dealy_work.work);
	int needretry = 0;
	mutex_lock(&smgt->monitor_lock);
	smgt->scatter_task_run_num++;

	codec_mm_scatter_scatter_arrange(smgt);
	needretry = codec_mm_scatter_scatter_clear(smgt, 0);

	if (needretry)
		codec_mm_schedule_delay_work(10, 0);
	else if (smgt->scatters_cnt > 0)
		codec_mm_schedule_delay_work(100, 0);
	codec_mm_scatter_cache_manage(smgt);
	mutex_unlock(&smgt->monitor_lock);
}

int codec_mm_scatter_mgt_init(void)
{
	scatter_mgt = kmalloc(sizeof(struct codec_mm_scatter_mgt), GFP_KERNEL);
	if (!scatter_mgt) {
		ERR_LOG("ERR:codec mm mpt init ERROR\n");
		return -1;
	}
	memset(scatter_mgt, 0, sizeof(struct codec_mm_scatter_mgt));
	spin_lock_init(&scatter_mgt->list_lock);
	scatter_mgt->alloced_page_num = 0;
	scatter_mgt->try_alloc_in_cma_page_cnt = (4 * 1024 * 1024) / PAGE_SIZE;
	scatter_mgt->try_alloc_in_sys_page_cnt_max = MAX_SYS_BLOCK_PAGE;
	scatter_mgt->try_alloc_in_sys_page_cnt = MAX_SYS_BLOCK_PAGE;
	scatter_mgt->try_alloc_in_sys_page_cnt_min = MIN_SYS_BLOCK_PAGE;
	scatter_mgt->reserved_block_mm_M = 64;
	scatter_mgt->keep_size_PAGE = 20 * SZ_1M >> PAGE_SHIFT;
	scatter_mgt->alloc_from_cma_first = 1;
	scatter_mgt->enable_slot_from_sys = 1;
	scatter_mgt->support_from_slot_sys =
		scatter_mgt->enable_slot_from_sys;
	if ((totalram_pages << PAGE_SHIFT) < 800 * SZ_1M) {
		/*less memory boards don't cache more,*/
		/*after alloced many pages.*/
		scatter_mgt->no_cache_size_M = 100;
	} else
		scatter_mgt->no_cache_size_M = 0;
	INIT_LIST_HEAD(&scatter_mgt->free_list);
	INIT_LIST_HEAD(&scatter_mgt->scatter_list);
	mutex_init(&scatter_mgt->monitor_lock);

	INIT_DELAYED_WORK(&scatter_mgt->dealy_work,
				codec_mm_scatter_monitor);

	return 0;
}
EXPORT_SYMBOL(codec_mm_scatter_mgt_init);

int codec_mm_scatter_mgt_test(void)
{
#if 0
	struct codec_mm_scatter *sc[64];

	INFO_LOG("codec_mm_scatter_mgt_test end dump info.11..\n");
	codec_mm_scatter_info_dump(NULL, 0);
	sc[0] = codec_mm_scatter_alloc(10240, 10000);
	sc[1] = codec_mm_scatter_alloc(10240, 10000);
	sc[2] = codec_mm_scatter_alloc(10240, 10000);
	codec_mm_dump_all_scatters();
	/*codec_mm_dump_all_slots(); */
	codec_mm_scatter_free(sc[0]);
	codec_mm_scatter_free(sc[1]);
	codec_mm_scatter_free(sc[2]);
	codec_mm_dump_all_scatters();
	/*codec_mm_dump_all_slots(); */
#endif
#if 0
	struct codec_mm_scatter *sc[64];

	INFO_LOG("codec_mm_scatter_mgt_test end dump info.11..\n");
	codec_mm_scatter_info_dump(NULL, 0);
	sc[0] = codec_mm_scatter_alloc(1024, 512);

	INFO_LOG("codec_mm_scatter_mgt_test end dump info.22..\n");
	codec_mm_scatter_info_dump(NULL, 0);
	codec_mm_dump_all_scatters();
	codec_mm_dump_all_slots();
	sc[1] = codec_mm_scatter_alloc(128, 32);
	sc[2] = codec_mm_scatter_alloc(128, 64);
	sc[3] = codec_mm_scatter_alloc(128, 128);
	INFO_LOG("codec_mm_scatter_mgt_test end dump info.33..\n");
	codec_mm_scatter_info_dump(NULL, 0);
	codec_mm_dump_all_scatters();
	codec_mm_dump_all_slots();
	codec_mm_scatter_free_tail_pages(sc[0], 128);	/* 128 */
	codec_mm_scatter_free_tail_pages(sc[1], 16);	/* 16 */
	codec_mm_scatter_free_tail_pages(sc[2], 4);	/* 4/4 */
	INFO_LOG("codec_mm_scatter_mgt_test end dump info.44..\n");
	codec_mm_scatter_info_dump(NULL, 0);
	codec_mm_dump_all_scatters();
	codec_mm_dump_all_slots();
	codec_mm_scatter_info_dump(NULL, 0);
	codec_mm_scatter_free(sc[1]);	/* 0 */
	sc[4] = codec_mm_scatter_alloc(128, 32);	/* 32 */
	sc[5] = codec_mm_scatter_alloc(512, 256);	/* 256 */
	codec_mm_scatter_alloc_want_pages(sc[2], 44);	/* 44-->//48 */
	INFO_LOG("codec_mm_scatter_mgt_test end dump info.55.\n");
	codec_mm_scatter_info_dump(NULL, 0);
	codec_mm_dump_all_scatters();
	codec_mm_dump_all_slots();
	codec_mm_scatter_free(sc[0]);
	codec_mm_scatter_free(sc[2]);
	codec_mm_scatter_free(sc[3]);

	INFO_LOG("codec_mm_scatter_mgt_test end dump info.66..\n");
	codec_mm_scatter_info_dump(NULL, 0);
	codec_mm_dump_all_scatters();
	codec_mm_scatter_free(sc[4]);
	codec_mm_scatter_free(sc[5]);
	INFO_LOG("codec_mm_scatter_mgt_test end dump info.77..\n");
	codec_mm_scatter_info_dump(NULL, 0);
	codec_mm_dump_all_scatters();
	codec_mm_dump_all_slots();
#endif

	return 0;
}
EXPORT_SYMBOL(codec_mm_scatter_mgt_test);

/*
*mode:0,dump, 1,alloc 2,more,3,free some,4,free all
*0:dump ALL
*1: alloc id,num
*2: alloc more  id,num
*3: free tail  id,start
*4: free all  id
*5:dump id
*6:dump all free slots
*/
int codec_mm_scatter_test(int mode, int p1, int p2)
{
	static int init;
	static struct codec_mm_scatter *sc[64];

	if (!init) {
		init++;
		memset(sc, 0, sizeof(sc));
	}
	switch (mode) {
	case 1:		/*alloc */
		INFO_LOG(" alloc sc[%d] num %d:\n", p1, p2);
		if (p1 > 0 && p1 < 64) {
			if (sc[p1])
				codec_mm_scatter_free_on_nouser(sc[p1]);
			sc[p1] = codec_mm_scatter_alloc(p2 * 2, p2);
		}
		break;
	case 2:		/*alloc more */
		INFO_LOG(" alloc more sc[%d] num %d\n", p1, p2);
		if (p1 > 0 && p1 < 64 && sc[p1])
			codec_mm_scatter_alloc_want_pages(sc[p1], p2);
		break;
	case 3:		/*alloc tails */
		INFO_LOG(" free some sc[%d] start free id %d\n", p1, p2);
		if (p1 > 0 && p1 < 64 && sc[p1])
			codec_mm_scatter_free_tail_pages(sc[p1], p2);
		break;
	case 4:
		INFO_LOG(" free sc[%d] all\n", p1);
		if (p1 > 0 && p1 < 64 && sc[p1]) {
			codec_mm_scatter_free_on_nouser(sc[p1]);
			sc[p1] = NULL;
		}
		break;
	case 5:
		INFO_LOG(" sc %d info:\n", p1);
		if (p1 > 0 && p1 < 64 && sc[p1])
			codec_mm_dump_scatter(sc[p1], NULL, 0);
		break;
	case 0:
	default:{
			int i;

			INFO_LOG(" dump all test sc info:\n");
			for (i = 0; i < 64; i++) {
				if (sc[i] != NULL) {
					INFO_LOG(" alloc sc[%d] has data\n", i);
					codec_mm_dump_scatter(sc[i], NULL, 0);
				}
			}
		}
		break;
	}
	return 0;
}
EXPORT_SYMBOL(codec_mm_scatter_test);

