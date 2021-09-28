/*
 * drivers/gpu/ion/ion_cma_heap.c
 *
 * Copyright (C) Linaro 2012
 * Author: <benjamin.gaignard@linaro.org> for ST-Ericsson.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>

#include <linux/swap.h>
#include <asm/dma-contiguous.h>
#include <asm/outercache.h>
#include <asm/memory.h>
#include <linux/dma-contiguous.h>
#include <linux/cma.h>
#include <linux/migrate.h>
#include <linux/freezer.h>
#include <linux/bug.h>
#include <linux/kthread.h>
#include <linux/page-isolation.h>
#include <internal.h>
#include <cma.h>	// for struct cma
#include <linux/bitmap.h>
#include <linux/bitops.h>
#include "../../../mm/internal.h"

#include "ion.h"
#include "ion_priv.h"
#include "ion_mstar_cma_heap.h"

static int ion_mstar_cma_shrink_lock(struct ion_heap *heap);
static int get_cma_freebits(struct cma *cma);


static struct gralloc_dma_pool gralloc_pool;
struct device *gralloc_cma_device;
static size_t gralloc_pool_size;

struct gralloc_dma_pool {
	size_t size;
	spinlock_t lock;
	unsigned long *bitmap;
	unsigned long nr_pages;
	void *vaddr;
	struct page **pages;
};

static int gralloc_pool_init(void)
{
	struct gralloc_dma_pool *pool = &gralloc_pool;
	unsigned long nr_pages;
	unsigned long *bitmap;
	struct page *page;
	struct page **pages;
	void *ptr;
	int bitmap_size;
	unsigned int pool_size_order;
	int i, retry_time = 0;

	if (!gralloc_cma_device) {
		printk(KERN_ERR"\33[0;36m   %s:%d   gralloc_pool_init fail no  MALI_GOP CMA \33[m \n", __func__, __LINE__);
		goto no_init;
	}
	pool->size = ((gralloc_cma_device->cma_area->count) << PAGE_SHIFT);
	nr_pages = pool->size >> PAGE_SHIFT;
	bitmap_size = BITS_TO_LONGS(nr_pages) * sizeof(long);

	bitmap = kzalloc(bitmap_size, GFP_KERNEL);
	if (!bitmap)
		goto no_bitmap;

	pages = kzalloc(nr_pages * sizeof(struct page *), GFP_KERNEL);
	if (!pages)
		goto no_pages;

	pool_size_order = get_order(pool->size);
retry:
	page = dma_alloc_from_contiguous(gralloc_cma_device, nr_pages, pool_size_order);
	if (!page) {
		if (retry_time < 3) {
			retry_time++;
			goto retry;
		} else
			goto no_cma;
	}
	ptr =  page_address(page);

	if (ptr) {
		memset(ptr, 0, pool->size);
		dmac_flush_range(ptr, ptr + pool->size);
		outer_flush_range(__pa(ptr), __pa(ptr) + pool->size);

		for (i = 0; i < nr_pages; i++)
			pages[i] = page + i;

		spin_lock_init(&pool->lock);
		pool->vaddr = ptr;
		pool->pages = pages;
		pool->bitmap = bitmap;
		pool->nr_pages = nr_pages;
		pr_info("%s DMA: preallocated %u KiB pool for atomic coherent allocations\n", gralloc_cma_device->init_name,
		       (unsigned)pool->size / 1024);
		return 0;
	} else
		goto no_cma;

no_cma:
	kfree(pages);
no_pages:
	kfree(bitmap);
no_bitmap:

no_init:
	pr_err("%s DMA: failed to allocate %u KiB pool for atomic coherent allocation\n", gralloc_cma_device->init_name,
	       (unsigned)pool->size / 1024);
	return -ENOMEM;
}

static void *gralloc_alloc_from_pool(size_t size, struct page **ret_page)
{
	struct gralloc_dma_pool *pool = &gralloc_pool;
	unsigned int count = PAGE_ALIGN(size) >> PAGE_SHIFT;
	unsigned int pageno;
	void *ptr = NULL;
	unsigned long align_mask;

	if (!pool->vaddr) {
		WARN(1, "coherent pool not initialised!\n");
		return NULL;
	}

	if (!pool)
		return NULL;

	/*
	 * Align the region allocation - allocations from pool are rather
	 * small, so align them to their order in pages, minimum is a page
	 * size. This helps reduce fragmentation of the DMA space.
	 */
	align_mask = (1 << get_order(size)) - 1;

	spin_lock(&pool->lock);
	pageno = bitmap_find_next_zero_area(pool->bitmap, pool->nr_pages,
					    0, count, align_mask);
	if (pageno < pool->nr_pages) {
		bitmap_set(pool->bitmap, pageno, count);
		ptr = pool->vaddr + PAGE_SIZE * pageno;
		*ret_page = pool->pages[pageno];
	} else {
		pr_err_once("ERROR:%s %u KiB atomic DMA coherent pool is too small!\n"
			    "Please increase it with coherent_pool= kernel parameter!\n", gralloc_cma_device->init_name,
			    (unsigned)pool->size / 1024);
	}
	spin_unlock(&pool->lock);

	return ptr;
}

static bool gralloc_in_atomic_pool(void *start, size_t size)
{
	struct gralloc_dma_pool *pool = &gralloc_pool;
	void *end = start + size;
	void *pool_start;
	void *pool_end;

	if (!pool)
		return false;

	pool_start = pool->vaddr;
	pool_end = pool->vaddr + pool->size;

	if (start < pool_start || start >= pool_end)
		return false;

	if (end <= pool_end)
		return true;

	WARN(1, "%s wrong coherent size(%p-%p) from atomic pool(%p-%p)\n", gralloc_cma_device->init_name,
	     start, end - 1, pool_start, pool_end - 1);

	return false;
}

static int gralloc_free_from_pool(void *start, size_t size)
{
	struct gralloc_dma_pool *pool = &gralloc_pool;
	unsigned long pageno, count;

	if (!gralloc_in_atomic_pool(start, size))
		return 0;
	pageno = (start - pool->vaddr) >> PAGE_SHIFT;
	count = size >> PAGE_SHIFT;

	spin_lock(&pool->lock);
	bitmap_clear(pool->bitmap, pageno, count);
	spin_unlock(&pool->lock);

	return 1;
}

static void ion_mstar_cma_clear_pages(struct page *page, unsigned long len)
{
	int pfn = page_to_pfn(page);

	CMA_BUG_ON(len % PAGE_SIZE, "size not aligned\n");
	while(len > 0){
		page = pfn_to_page(pfn);
		clear_highpage(page);
		pfn++;
		len -= PAGE_SIZE;
	}
}
unsigned long  get_free_bit_count(struct cma *cma, unsigned long start, unsigned long len)
{
	unsigned long count = 0;
	unsigned long offset_zero;
	unsigned long end = start+len;

	if(len == 0 || start >= cma->count)
		return 0;

	end = start+len;
	end = min(end, cma->count);

	offset_zero = start;

	while(offset_zero < cma->count){
		unsigned offset_bit;
		offset_zero = find_next_zero_bit(cma->bitmap, end, offset_zero);
		if(offset_zero>= end)
			break;
		offset_bit = find_next_bit(cma->bitmap, end, offset_zero+1);
		count += offset_bit-offset_zero;
		offset_zero = offset_bit+1;
	}

	return count;
}

static int ion_mstar_cma_get_sgtable(struct device *dev, struct sg_table *sgt,
			       struct page *page, dma_addr_t handle, size_t size)
{
	int ret;

	ret = sg_alloc_table(sgt, 1, GFP_KERNEL);
	if (unlikely(ret))
		return ret;

	sg_set_page(sgt->sgl, page, PAGE_ALIGN(size), 0);
	return 0;
}

static int cma_increase_pool_freelist_threshold(struct ion_mstar_cma_heap *mstar_cma_heap, unsigned long len)
{
	struct mstar_cma *mstar_cma= mstar_cma_heap->mstar_cma;
	unsigned long retlen = 0, nlen = CACHE_POOL_LEN;
	struct device *dev = mstar_cma_heap->dev;
	struct cma_cache_node *node;
	struct page *page;
	int pfn, totalfreebits;
	int ret = -ENOMEM;

	//No free bits in cma, return directly
	totalfreebits = get_cma_freebits(mstar_cma->cma);
	if(totalfreebits == 0)
		return -ENOMEM;

	/*
	 *Auto adjust cache size & cache node size
	 */
	if(nlen < len)
		nlen = PAGE_ALIGN(len);

	spin_lock(&mstar_cma->cache_lock);
	len += mstar_cma->cache_size;
	spin_unlock(&mstar_cma->cache_lock);
	if(len < CACHE_POOL_MAX)
		len = CACHE_POOL_MAX;
	else
		len = PAGE_ALIGN(len);

	spin_lock(&mstar_cma->cache_lock);
	while(mstar_cma->cache_size < len){
		spin_unlock(&mstar_cma->cache_lock);
		page = dma_alloc_from_contiguous_direct(dev, nlen>>PAGE_SHIFT, 0, &retlen);
		if(!page){
			if(ret == -ENOMEM)
				return ret;
			else
				return 0;
		}
		ret = 0;

		pfn = page_to_pfn(page);

		CMA_BUG_ON((page && !retlen) || (!page && retlen), "alloc out of cma area\n");
		CMA_BUG_ON(pfn < mstar_cma->cma->base_pfn, "alloc out of cma arean");
		CMA_BUG_ON(pfn >= mstar_cma->cma->base_pfn + mstar_cma->cma->count, "alloc out of cma area\n");

		node = kmalloc(sizeof(struct cma_cache_node), GFP_KERNEL);
		if(!node){
			printk("[%s] Alloc cache node fail\n", __func__);
			dma_release_from_contiguous(dev, page, retlen);
			return -ENOMEM;
		}
		INIT_LIST_HEAD(&node->list);
		node->len = retlen * PAGE_SIZE;
		node->page = page;

		spin_lock(&mstar_cma->cache_lock);
		list_add_tail(&node->list, &mstar_cma->cache_head);
		mstar_cma->cache_size += retlen * PAGE_SIZE;
		mstar_cma->cache_page_count += retlen;
	}
	spin_unlock(&mstar_cma->cache_lock);
	return 0;
}

//get one page
struct page* __mstar_get_discrete(struct ion_heap *heap)
{
	struct ion_mstar_cma_heap *mstar_heap = to_mstar_cma_heap(heap);
	struct mstar_cma *mstar_cma = mstar_heap->mstar_cma;
	struct page *page;
	struct cma_cache_node *node;
	int pfn;
	struct list_head *head;

retry:
	head = &mstar_cma->cache_head;
	spin_lock(&mstar_cma->cache_lock);

	CMA_BUG_ON(list_empty(&mstar_cma->cache_head) && mstar_cma->cache_size, "list empty but size is not 0\n");
	CMA_BUG_ON(!list_empty(&mstar_cma->cache_head) && !mstar_cma->cache_size, "list not empty but size is 0\n");
	CMA_BUG_ON(mstar_cma->cache_size != mstar_cma->cache_page_count * PAGE_SIZE, "cache size not match cache pages, %ld, %ld\n", mstar_cma->cache_page_count, mstar_cma->cache_size);

	if(unlikely(list_empty(head))){
		spin_unlock(&mstar_cma->cache_lock);
		cma_increase_pool_freelist_threshold(mstar_heap, 0);

		spin_lock(&mstar_cma->cache_lock);
		if(unlikely(list_empty(head))){
			spin_unlock(&mstar_cma->cache_lock);
			printk("\033[31mFunction = %s, Line = %d, mali cma no page\033[m\n", __PRETTY_FUNCTION__, __LINE__);
			page = alloc_page(GFP_HIGHUSER| __GFP_ZERO | __GFP_REPEAT | __GFP_NOWARN);
			return page;
		}
		else{
			spin_unlock(&mstar_cma->cache_lock);
			goto retry;
		}
	}
	else
	{
		BUG_ON(list_empty(&mstar_cma->cache_head) && mstar_cma->cache_size);

		node = list_first_entry(head, struct cma_cache_node, list);
		pfn = page_to_pfn(node->page);
		page = node->page;
		node->len -= PAGE_SIZE;
		if(node->len == 0){
			list_del(&node->list);
			kfree(node);
		}
		else
			node->page = pfn_to_page(pfn + 1);

		mstar_cma->cache_size -= PAGE_SIZE;
		mstar_cma->cache_page_count--;

		CMA_BUG_ON(mstar_cma->cache_size < 0 || mstar_cma->cache_page_count < 0, "cache < 0\n");
		CMA_BUG_ON(mstar_cma->cache_size % PAGE_SIZE, "cache size not aligned to page size\n");

		spin_unlock(&mstar_cma->cache_lock);
		return page;
	}
}

static int ion_mstar_cma_alloc_discrete(struct ion_heap *heap,
														struct ion_buffer *buffer,
														unsigned long len)
{
	struct cma_cache_node *node, *tmp;
	struct ion_mstar_cma_buffer_info* buffer_info;
	struct ion_mstar_cma_heap * mstar_heap = to_mstar_cma_heap(heap);
	struct mstar_cma *mstar_cma = mstar_heap->mstar_cma;
	struct scatterlist *sg;
	struct list_head head, *cache_head = &mstar_cma->cache_head;
	int index = 0, ret;
	len = PAGE_ALIGN(len);

	INIT_LIST_HEAD(&head);

	buffer_info = kzalloc(sizeof(struct ion_mstar_cma_buffer_info), GFP_KERNEL);
	if (!buffer_info) {
		printk("[%s] Can't allocate buffer info\n", __func__);
		return ION_MSTAR_CMA_ALLOCATE_FAILED;
	}

retry:
	spin_lock(&mstar_cma->cache_lock);

	CMA_BUG_ON(list_empty(&mstar_cma->cache_head) && mstar_cma->cache_size, "list empty but size is not 0\n");
	CMA_BUG_ON(!list_empty(&mstar_cma->cache_head) && !mstar_cma->cache_size, "list not empty but size is 0\n");
	CMA_BUG_ON(mstar_cma->cache_size != mstar_cma->cache_page_count * PAGE_SIZE, "cache size not match cache pages\n");

	list_for_each_entry_safe(node, tmp, cache_head, list){

		CMA_BUG_ON(!node->len, "cache node len is 0\n");

		if(len >= node->len){
			list_del(&node->list);
			list_add_tail(&node->list, &head);
			len -= node->len;
			mstar_cma->cache_size -= node->len;
			mstar_cma->cache_page_count -= node->len / PAGE_SIZE;
			CMA_BUG_ON(mstar_cma->cache_size < 0 || mstar_cma->cache_page_count < 0, "cache < 0\n");
		}
		else{
			struct cma_cache_node *nnode = kmalloc(sizeof(struct cma_cache_node), GFP_KERNEL);
			int pfn = page_to_pfn(node->page);
			if(!nnode){
				printk("%s: malloc cma cache node failed\n", __func__);
				spin_unlock(&mstar_cma->cache_lock);
				goto err3;
			}
			nnode->page = node->page;
			nnode->len = len;
			node->len -= len;
			node->page = pfn_to_page(pfn + len / PAGE_SIZE);
			mstar_cma->cache_size -= len;
			mstar_cma->cache_page_count -= len / PAGE_SIZE;
			CMA_BUG_ON(mstar_cma->cache_size < 0 || mstar_cma->cache_page_count < 0, "cache < 0\n");
			INIT_LIST_HEAD(&nnode->list);
			list_add_tail(&nnode->list, &head);
			len = 0;
		}
		index++;

		CMA_BUG_ON(mstar_cma->cache_size < 0 || mstar_cma->cache_page_count < 0, "cache < 0\n");
		if(len == 0)
			break;
	}
	spin_unlock(&mstar_cma->cache_lock);

	if(len){
		cma_increase_pool_freelist_threshold(mstar_heap, len);

		spin_lock(&mstar_cma->cache_lock);
		if(list_empty(&mstar_cma->cache_head)){
			spin_unlock(&mstar_cma->cache_lock);
			while(len){
				struct page *page;
				struct cma_cache_node *nnode;
				printk("\033[31mFunction = %s, Line = %d, mali cma no page\033[m\n", __PRETTY_FUNCTION__, __LINE__);
				page = alloc_page(GFP_HIGHUSER| __GFP_ZERO | __GFP_REPEAT | __GFP_NOWARN);
				if(!page){
					printk("[%s] alloc page fail, %d\n", __func__, __LINE__);
					goto err3;
				}
				nnode = kmalloc(sizeof(struct cma_cache_node), GFP_KERNEL);
				if(!nnode){
					printk("[%s] malloc cma cache node faill, %d\n", __func__, __LINE__);
					__free_page(page);
					goto err3;
				}
				nnode->len = PAGE_SIZE;
				nnode->page = page;
				INIT_LIST_HEAD(&nnode->list);
				list_add_tail(&nnode->list, &head);

				index++;
				len -= PAGE_SIZE;
			}
		}
		else{
			spin_unlock(&mstar_cma->cache_lock);
			goto retry;
		}
	}

	buffer_info->table = kzalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!buffer_info->table){
		printk("[%s] malloc buffer table failed, %d\n", __func__, __LINE__);
		goto err3;
	}
	ret = sg_alloc_table(buffer_info->table, index, GFP_KERNEL);
	if (ret){
		printk("[%s] malloc sg table failed, %d\n", __func__, __LINE__);
		goto err2;
	}

	sg = buffer_info->table->sgl;
	list_for_each_entry_safe(node, tmp, &head, list){
		struct page *page = node->page;
		sg_set_page(sg, page, node->len, 0);
		ion_mstar_cma_clear_pages(node->page, node->len);
		ion_pages_sync_for_device(NULL, page, node->len, DMA_BIDIRECTIONAL);
		sg = sg_next(sg);
		list_del(&node->list);
		kfree(node);
	}
	buffer_info->flag |= ION_FLAG_DISCRETE;
	buffer->priv_virt = buffer_info;
	buffer->sg_table = buffer_info->table;
	return 0;

	sg_free_table(buffer_info->table);
err2:
	kfree(buffer_info->table);
err3:
	spin_lock(&mstar_cma->cache_lock);
	list_for_each_entry_safe(node, tmp, cache_head, list) {
		int pfn = page_to_pfn(node->page);
		struct page *page;
		for(index = 0; index < node->len / PAGE_SIZE; index++){
			page = pfn_to_page(pfn + index);
			__mstar_free_one_page(heap, page);
		}
		list_del(&node->list);
		kfree(node);
	}
	spin_unlock(&mstar_cma->cache_lock);
	kfree(buffer_info);
	return -ENOMEM;
}

#if 0
static int cma_swap_worker_thread(void *p)
{
	struct ion_mstar_cma_heap *mstar_cma_heap = (struct ion_mstar_cma_heap *)p;
	struct mstar_cma *mstar_cma = mstar_cma_heap->mstar_cma;
	set_freezable();
	while(1){
		DEFINE_WAIT(wait);
		try_to_freeze();

		spin_lock(&mstar_cma->cache_lock);

		CMA_BUG_ON(list_empty(&mstar_cma->cache_head) && mstar_cma->cache_size, "list empty but size is not 0\n");
		CMA_BUG_ON(!list_empty(&mstar_cma->cache_head) && !mstar_cma->cache_size, "list not empty but size is 0\n");
		CMA_BUG_ON(mstar_cma->cache_size != mstar_cma->cache_page_count * PAGE_SIZE, "cache size not match cache pages\n");

		if(mstar_cma->cache_size < CACHE_POOL_MIN){
			spin_unlock(&mstar_cma->cache_lock);
			cma_increase_pool_freelist_threshold(mstar_cma_heap, 0);
		}
		else
			spin_unlock(&mstar_cma->cache_lock);

		prepare_to_wait(&mstar_cma->cma_swap_wait, &wait, TASK_INTERRUPTIBLE);
		schedule_timeout(HZ * 20);
		finish_wait(&mstar_cma->cma_swap_wait, &wait);
	}
	return 0;
}
#endif

int __mstar_free_one_page(struct ion_heap *heap, struct page *page)
{
	struct ion_mstar_cma_heap *mstar_cma_heap = to_mstar_cma_heap(heap);
	struct mstar_cma *mstar_cma = mstar_cma_heap->mstar_cma;
	unsigned long pfn = page_to_pfn(page);

	CMA_BUG_ON(!heap || !page, "NULL heap or page pointer\n");
	CMA_BUG_ON(atomic_read(&page->_refcount)!= 1,"page_count=%d \n",page_count(page));

	if(!in_cma_range(heap, page)){
		heap = find_heap_by_page(page);
		if(heap){
			mstar_cma_heap = to_mstar_cma_heap(heap);
			mstar_cma = mstar_cma_heap->mstar_cma;
		}
	}

	if(heap){
		mutex_lock(&mstar_cma->cma->lock);
		if(!test_and_clear_bit(pfn - mstar_cma->cma->base_pfn, mstar_cma->cma->bitmap))
			BUG();
		mutex_unlock(&mstar_cma->cma->lock);
#ifdef CONFIG_MP_CMA_PATCH_CMA_AGGRESSIVE_ALLOC
		adjust_managed_cma_page_count(mstar_cma->zone, 1);
#endif
	}

	CMA_BUG_ON(page_count(page) != 1,"page_count=%d\n",page_count(page));

	__free_page(page);
	return 0;
}

void __dma_clear_buffer2(struct page *page, size_t size)
{
	void *ptr;
	if (!page)
		return;
	ptr = page_address(page);
	if (ptr) {
		memset(ptr, 0, size);
		dmac_flush_range(ptr, ptr + size);
		outer_flush_range(__pa(ptr), __pa(ptr) + size);
	}
}

static int ion_mstar_cma_alloc_contiguous(struct ion_heap *heap,
														struct ion_buffer *buffer,
														unsigned long len,
														unsigned long flags)
{
	struct ion_mstar_cma_buffer_info* buffer_info;
	struct ion_mstar_cma_heap *mstar_cma_heap = to_mstar_cma_heap(heap);
	struct device *dev = mstar_cma_heap->dev;
	struct mstar_cma *mstar_cma = mstar_cma_heap->mstar_cma;
	int freepg, retry = 0;

	buffer_info = kzalloc(sizeof(struct ion_mstar_cma_buffer_info), GFP_KERNEL);
	if (!buffer_info) {
		dev_err(dev, "Can't allocate buffer info\n");
		return ION_MSTAR_CMA_ALLOCATE_FAILED;
	}

ALLOC:
    buffer_info->page = dma_alloc_from_contiguous(dev, len>>PAGE_SHIFT, get_order(len));
    buffer_info->size = len;
    //__dma_clear_buffer2(buffer_info->page, (len>>PAGE_SHIFT) << PAGE_SHIFT);
	if (!buffer_info->page) {
		if (retry < CMA_CONTIG_RTYCNT)
		{
			// get lock in the first retry round
			if (!retry)
			{
				mutex_lock(&mstar_cma->contig_rtylock);
			}

			freepg = get_free_bit_count(mstar_cma->cma, 0, mstar_cma->cma->count)
					+ mstar_cma->cache_page_count;
			// number of free pages is qualified
			if (freepg >= len>>PAGE_SHIFT)
			{
				// we can shrink cma cache and try again
				retry++;
				freepg = ion_mstar_cma_shrink_lock(heap);
				printk("CMA contig alloc retry by shrinking %d pages!\n", freepg);

				goto ALLOC;
			}
			else
			{
				mutex_unlock(&mstar_cma->contig_rtylock);
				//printk("CMA contig alloc retry fail, lack of free pages! free:0x%x need:0x%lx retry:%d\n",
					//freepg, len>>PAGE_SHIFT, retry);
			}
		}
		else
		{
			mutex_unlock(&mstar_cma->contig_rtylock);
			printk("CMA contig alloc retry fail!\n");
		}

		dev_err(dev, "Fail to allocate buffer1\n");
		goto err;
	}

	if (retry && (retry <= CMA_CONTIG_RTYCNT))
	{
		mutex_unlock(&mstar_cma->contig_rtylock);
		printk("CMA contig alloc retry pass! retry %d\n", retry);
	}

#if defined(CONFIG_ARM)
	buffer_info->handle = (dma_addr_t)__pfn_to_phys(page_to_pfn(buffer_info->page));		// page to pfn to bus_addr
#else
	buffer_info->handle = (dma_addr_t)((page_to_pfn(buffer_info->page)) << PAGE_SHIFT); 	// page to pfn to bus_addr
#endif

	buffer_info->table = kmalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!buffer_info->table) {
		dev_err(dev, "Fail to allocate sg table\n");
		goto free_mem;
	}

	if (ion_mstar_cma_get_sgtable
	    (dev, buffer_info->table, buffer_info->page, buffer_info->handle, len)){
			dma_release_from_contiguous(dev, buffer_info->page, len >> PAGE_SHIFT);
			goto err1;
	}

	if (flags & ION_FLAG_ZERO_MEMORY)
	{
		ion_mstar_cma_clear_pages(buffer_info->page, len);
	}

	if (flags & (ION_FLAG_CACHED | ION_FLAG_ZERO_MEMORY))
	{
		ion_pages_sync_for_device(NULL, buffer_info->page, len, DMA_BIDIRECTIONAL);
	}

	buffer_info->flag |= ION_FLAG_CONTIGUOUS;
	buffer->priv_virt = buffer_info;
	buffer->sg_table = buffer_info->table;
	dev_dbg(dev, "Allocate buffer %p\n", buffer);

	return 0;

err1:
	kfree(buffer_info->table);
free_mem:
	dma_release_from_contiguous(dev, buffer_info->page, len >> PAGE_SHIFT);
err:
	kfree(buffer_info);
	return ION_MSTAR_CMA_ALLOCATE_FAILED;
}

static int ion_mstar_cma_alloc_contiguous_2(struct ion_heap *heap,
		struct ion_buffer *buffer,
		unsigned long len,
		unsigned long flags)
{
	struct ion_mstar_cma_buffer_info *buffer_info;
	struct ion_mstar_cma_heap *mstar_cma_heap = to_mstar_cma_heap(heap);
	struct device *dev = mstar_cma_heap->dev;
	struct mstar_cma *mstar_cma = mstar_cma_heap->mstar_cma;
	int freepg, retry = 0;

	buffer_info = kzalloc(sizeof(struct ion_mstar_cma_buffer_info), GFP_KERNEL);
	if (!buffer_info) {
		dev_err(dev, "Can't allocate buffer info\n");
		return ION_MSTAR_CMA_ALLOCATE_FAILED;
	}

	gralloc_alloc_from_pool(PAGE_ALIGN(len), &buffer_info->page);
	buffer_info->size = len;

	if (!buffer_info->page) {
		dev_err(dev, "Fail to allocate buffer\n");
		goto err;
	}

#if defined(CONFIG_ARM)
	/* page to pfn to bus_addr */
	buffer_info->handle = pfn_to_dma(dev, page_to_pfn(buffer_info->page));
#else
	/* page to pfn to bus_addr */
	buffer_info->handle = (dma_addr_t)((page_to_pfn(buffer_info->page)) << PAGE_SHIFT);
#endif

	buffer_info->table = kmalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!buffer_info->table) {
		dev_err(dev, "Fail to allocate sg table\n");
		goto err;
	}

	if (ion_mstar_cma_get_sgtable
	    (dev, buffer_info->table, buffer_info->page, buffer_info->handle, len)) {
		goto err1;
	}

	buffer_info->flag |= flags | ION_FLAG_CONTIGUOUS;
	buffer->priv_virt = buffer_info;
	buffer->sg_table = buffer_info->table;

	dev_dbg("ion_cma_alloc: allocate %d kB, buffer flag is [0x%x]\n",
				buffer_info->size/1024, (int)buffer_info->flag);

	return 0;

err1:
	kfree(buffer_info->table);
err:
	kfree(buffer_info);

	return ION_MSTAR_CMA_ALLOCATE_FAILED;
}


static int ion_mstar_cma_alloc_from_addr(struct ion_heap *heap,
														struct ion_buffer *buffer,
														unsigned long start,
														unsigned long len,
														unsigned long flags)
{
	struct ion_mstar_cma_buffer_info* buffer_info;
	struct ion_mstar_cma_heap *mstar_cma_heap = to_mstar_cma_heap(heap);
	struct device *dev = mstar_cma_heap->dev;
	struct mstar_cma *mstar_cma = mstar_cma_heap->mstar_cma;
	int freepg, retry = 0;

	buffer_info = kzalloc(sizeof(struct ion_mstar_cma_buffer_info), GFP_KERNEL);
	if (!buffer_info) {
		dev_err(dev, "Can't allocate buffer info\n");
		return ION_MSTAR_CMA_ALLOCATE_FAILED;
	}

ALLOC:
	//buffer_info->page = dma_alloc_from_contiguous_addr(dev,start,len >> PAGE_SHIFT,1);
	buffer_info->page = dma_alloc_at_from_contiguous(dev, len >> PAGE_SHIFT, 1, start);
	buffer_info->size = len;

	if (!buffer_info->page) {
		if (retry < CMA_CONTIG_RTYCNT)
		{
			// get lock in the first retry round
			if (!retry)
			{
				mutex_lock(&mstar_cma->contig_rtylock);
			}

			freepg = get_free_bit_count(mstar_cma->cma, 0, mstar_cma->cma->count)
					+ mstar_cma->cache_page_count;
			// number of free pages is qualified
			if (freepg >= len>>PAGE_SHIFT)
			{
				// we can shrink cma cache and try again
				retry++;
				freepg = ion_mstar_cma_shrink_lock(heap);
				printk("CMA contig alloc retry with addr by shrinking %d pages!\n", freepg);

				goto ALLOC;
			}
			else
			{
				mutex_unlock(&mstar_cma->contig_rtylock);
				//printk("CMA contig alloc retry with addr fail, lack of free pages! free:0x%x need:0x%lx retry:%d\n",
					//freepg, len>>PAGE_SHIFT, retry);
			}
		}
		else
		{
			mutex_unlock(&mstar_cma->contig_rtylock);
			printk("CMA contig alloc retry with addr fail!\n");
		}

		dev_err(dev, "Fail to allocate buffer1\n");
		goto err;
	}

	if (retry && (retry <= CMA_CONTIG_RTYCNT))
	{
		mutex_unlock(&mstar_cma->contig_rtylock);
		printk("CMA contig alloc retry with addr pass! retry %d\n", retry);
	}

#if defined(CONFIG_ARM)
	buffer_info->handle = (dma_addr_t)__pfn_to_phys(page_to_pfn(buffer_info->page));		// page to pfn to bus_addr
#else
	buffer_info->handle = (dma_addr_t)((page_to_pfn(buffer_info->page)) << PAGE_SHIFT); 	// page to pfn to bus_addr
#endif

	buffer_info->table = kmalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!buffer_info->table) {
		dev_err(dev, "Fail to allocate sg table\n");
		goto free_mem;
	}

	if (ion_mstar_cma_get_sgtable
	    (dev, buffer_info->table, buffer_info->page, buffer_info->handle, len)){
			dma_release_from_contiguous(dev, buffer_info->page, len >> PAGE_SHIFT);
			goto err1;
	}

	if (flags & ION_FLAG_ZERO_MEMORY)
	{
		ion_mstar_cma_clear_pages(buffer_info->page, len);
	}

	if (flags & (ION_FLAG_CACHED | ION_FLAG_ZERO_MEMORY))
	{
		ion_pages_sync_for_device(NULL, buffer_info->page, len, DMA_BIDIRECTIONAL);
        }

	buffer_info->flag |= ION_FLAG_CONTIGUOUS;
	buffer->priv_virt = buffer_info;
	buffer->sg_table = buffer_info->table;
	dev_dbg(dev, "Allocate buffer %p\n", buffer);
	return 0;

err1:
	kfree(buffer_info->table);
free_mem:
	dma_release_from_contiguous(dev, buffer_info->page, len >> PAGE_SHIFT);
err:
	kfree(buffer_info);
	return ION_MSTAR_CMA_ALLOCATE_FAILED;
}

static int ion_mstar_cma_allocate(struct ion_heap *heap, struct ion_buffer *buffer,
			    unsigned long len, unsigned long align,
			    unsigned long flags)
{
	int ret;

	if(flags & ION_FLAG_CONTIGUOUS){
		ret = ion_mstar_cma_alloc_contiguous(heap, buffer, len, flags);
		buffer->size = len;
	}
	else if(flags & ION_FLAG_STARTADDR){
		ret = ion_mstar_cma_alloc_from_addr(heap, buffer, align, len, flags);
		buffer->size = len;
	}
	else{
		ret = ion_mstar_cma_alloc_discrete(heap, buffer, len);
	}

	return ret;
}

static int mali_gop_init;
static int ion_mstar_cma_allocate_2(struct ion_heap *heap, struct ion_buffer *buffer,
			    unsigned long len, unsigned long align,
			    unsigned long flags)
{
	int ret;

	if (flags & ION_FLAG_CONTIGUOUS) {
		if (mali_gop_init == 0) {
			gralloc_pool_init();
			mali_gop_init = 1;
		}
		ret = ion_mstar_cma_alloc_contiguous_2(heap, buffer, len, flags);
		buffer->size = len;
	} else if (flags & ION_FLAG_STARTADDR) {
		printk("\033[31m grAlloc use ION_FLAG_STARTADDR!!!!!!!!!!!!!\033[m\n");
		ret = ION_MSTAR_CMA_ALLOCATE_FAILED;
	} else {
		ret = ION_MSTAR_CMA_ALLOCATE_FAILED;
	}

	return ret;
}

static void ion_mstar_cma_free(struct ion_buffer *buffer)
{
	struct ion_mstar_cma_heap *mstar_cma_heap = to_mstar_cma_heap(buffer->heap);
	struct device *dev = mstar_cma_heap->dev;
	struct ion_mstar_cma_buffer_info *buffer_info = buffer->priv_virt;
	int i;

	dev_dbg(dev, "Release buffer %p\n", buffer);
	/* release memory */

	if(buffer_info->flag & ION_FLAG_DISCRETE){
		struct sg_table *table = buffer_info->table;
		struct scatterlist *sg;

		for_each_sg(table->sgl, sg, table->nents, i){
			//CMA_BUG_ON(sg->length != PAGE_SIZE,"invalide sg length %d \n",sg->length);
			struct page *page = sg_page(sg);
			unsigned long i, pfn = page_to_pfn(page);
			for(i = 0; i < sg->length; i+= PAGE_SIZE){
				page = pfn_to_page(pfn);
				__mstar_free_one_page(buffer->heap,page);
				pfn++;
			}
		}
	}
	else if(buffer_info->flag & (ION_FLAG_CONTIGUOUS | ION_FLAG_STARTADDR)){
		dma_release_from_contiguous(dev, buffer_info->page, buffer_info->size >> PAGE_SHIFT);
	}
	else
		CMA_BUG_ON(1,"BUG!!! \n");

	/* release sg table */
	sg_free_table(buffer_info->table);
	kfree(buffer_info->table);
	kfree(buffer_info);
}

static void ion_mstar_cma_free_2(struct ion_buffer *buffer)
{
	struct ion_mstar_cma_heap *mstar_cma_heap = to_mstar_cma_heap(buffer->heap);
	struct device *dev = mstar_cma_heap->dev;
	struct ion_mstar_cma_buffer_info *buffer_info = buffer->priv_virt;
	int i;

	dev_dbg(dev, "Release buffer %p\n", buffer);
	/* release memory */

	if (buffer_info->flag & ION_FLAG_DISCRETE) {
		struct sg_table *table = buffer_info->table;
		struct scatterlist *sg;

		for_each_sg(table->sgl, sg, table->nents, i) {
			/* CMA_BUG_ON(sg->length != PAGE_SIZE,"invalide sg length %d\n",sg->length); */
			struct page *page = sg_page(sg);
			unsigned long i, pfn = page_to_pfn(page);
			for (i = 0; i < sg->length; i += PAGE_SIZE) {
				page = pfn_to_page(pfn);
				__mstar_free_one_page(buffer->heap, page);
				pfn++;
			}
		}
	} else if (buffer_info->flag & (ION_FLAG_CONTIGUOUS | ION_FLAG_STARTADDR)) {
		struct sg_table *table = buffer_info->table;

		pr_debug("ion_cma_free: free %d kB, buffer flag is [0x%x]\n",
					buffer_info->size/1024, (int)buffer_info->flag);

		if (ion_buffer_cached(buffer))
			dma_sync_sg_for_device(NULL, table->sgl, table->nents,
									DMA_BIDIRECTIONAL);

		memset(__va(page_to_pfn(buffer_info->page) << PAGE_SHIFT), 0, PAGE_ALIGN(buffer_info->size));
		gralloc_free_from_pool(__va(page_to_pfn(buffer_info->page) << PAGE_SHIFT), PAGE_ALIGN(buffer_info->size));
	} else
		CMA_BUG_ON(1, "BUG!!!\n");

	/* release sg table */
	sg_free_table(buffer_info->table);
	kfree(buffer_info->table);
	kfree(buffer_info);
}

/* return physical address in addr */
static int ion_mstar_cma_phys(struct ion_heap *heap, struct ion_buffer *buffer,
			ion_phys_addr_t *addr, size_t *len)
{
	struct ion_mstar_cma_heap *mstar_cma_heap = to_mstar_cma_heap(buffer->heap);
	struct device *dev = mstar_cma_heap->dev;
	struct ion_mstar_cma_buffer_info *info = buffer->priv_virt;


	if(info->flag & ION_FLAG_DISCRETE){
		return -ENODEV;
	}
	else if(info->flag & (ION_FLAG_CONTIGUOUS | ION_FLAG_STARTADDR)){
		dev_dbg(dev, "Return buffer %p physical address 0x%pa\n", buffer,
			&info->handle);

		*addr = info->handle;
		*len = buffer->size;
	}
	else
		CMA_BUG_ON(1,"BUG!!! \n");
	return 0;
}
//#endif

#if 0
static struct sg_table *ion_mstar_cma_heap_map_dma(struct ion_heap *heap,
					     struct ion_buffer *buffer)
{
	struct ion_mstar_cma_buffer_info *info = buffer->priv_virt;

	return info->table;
}
#endif

#if 0
static void ion_mstar_cma_heap_unmap_dma(struct ion_heap *heap,
				   struct ion_buffer *buffer)
{
	return;
}
#endif


static int ion_cma_heap_map_user(struct ion_heap *heap, struct ion_buffer *buffer,
		      struct vm_area_struct *vma)
{
	struct ion_mstar_cma_buffer_info *info = buffer->priv_virt;

	struct sg_table *table = info->table;
	unsigned long addr = vma->vm_start;
	unsigned long offset = vma->vm_pgoff * PAGE_SIZE;
	struct scatterlist *sg;
	int i;
	int ret;

	for_each_sg(table->sgl, sg, table->nents, i) {
		struct page *page = sg_page(sg);
		unsigned long remainder = vma->vm_end - addr;
		unsigned long len = sg->length;

		if (offset >= sg->length) {
			offset -= sg->length;
			continue;
		} else if (offset) {
			page += offset / PAGE_SIZE;
			len = sg->length - offset;
			offset = 0;
		}
		len = min(len, remainder);
		ret = remap_pfn_range(vma, addr, page_to_pfn(page), len,
				vma->vm_page_prot);
		if (ret)
			return ret;
		addr += len;
		if (addr >= vma->vm_end)
			return 0;
	}
	return 0;
}

static void *ion_mstar_cma_map_kernel(struct ion_heap *heap,
				struct ion_buffer *buffer)
{
	struct ion_mstar_cma_buffer_info *info = buffer->priv_virt;

	if(info->flag & ION_FLAG_DISCRETE)
		return ion_heap_map_kernel(heap,buffer);
	else if(info->flag & (ION_FLAG_CONTIGUOUS | ION_FLAG_STARTADDR)){
		return ion_heap_map_kernel(heap,buffer);
	}
	else
		CMA_BUG_ON(1,"BUG!!! \n");

	return 0;
}

static void ion_mstar_cma_unmap_kernel(struct ion_heap *heap,
					struct ion_buffer *buffer)
{
	struct ion_mstar_cma_buffer_info *info = buffer->priv_virt;

	if(info->flag & ION_FLAG_DISCRETE)
		return ion_heap_unmap_kernel(heap,buffer);
	else if(info->flag & (ION_FLAG_CONTIGUOUS | ION_FLAG_STARTADDR)){
		return ion_heap_unmap_kernel(heap,buffer);
	}
	else
		CMA_BUG_ON(1,"BUG!!! \n");
}

static int ion_mstar_cma_shrink(struct ion_heap *heap, gfp_t gfp_mask,
					int nr_to_scan)
{
	struct ion_mstar_cma_heap *mstar_cma_heap = to_mstar_cma_heap(heap);
	struct mstar_cma *mstar_cma = mstar_cma_heap->mstar_cma;
	unsigned long pfn, freed = 0, len = 0;
	struct page *page;
	struct cma_cache_node *node, *tmp;

	if (gfpflags_to_migratetype(gfp_mask) != MIGRATE_MOVABLE)
		return 0;

	if(!mutex_trylock(&mstar_cma->cma->lock))
		goto OUT_NO_LOCK;

	spin_lock(&mstar_cma->cache_lock);
	list_for_each_entry_safe(node, tmp, &mstar_cma->cache_head, list){
		pfn = page_to_pfn(node->page);
		len = 0;

		CMA_BUG_ON(pfn<mstar_cma->cma->base_pfn || pfn>=mstar_cma->cma->base_pfn+mstar_cma->cma->count, "pfn=%lx,cma[%lx,%lx] \n",pfn,mstar_cma->cma->base_pfn,mstar_cma->cma->count);

		/*
		 *Free entire node, so freed may be a little larger than @nr_to_scan
		 */
		while(len < node->len){
			if(!test_and_clear_bit(pfn - mstar_cma->cma->base_pfn, mstar_cma->cma->bitmap)){
				BUG();
			}
			page = pfn_to_page(pfn);
			__free_page(page);
			pfn++;
			len += PAGE_SIZE;
		}
		list_del(&node->list);
		mstar_cma->cache_size -= node->len;
		mstar_cma->cache_page_count -= node->len / PAGE_SIZE;
		freed += node->len / PAGE_SIZE;
		kfree(node);
		if(freed >= nr_to_scan){
			break;
		}
	}
	spin_unlock(&mstar_cma->cache_lock);
	mutex_unlock(&mstar_cma->cma->lock);

OUT_NO_LOCK:
	return mstar_cma->cache_page_count;
}

static int ion_mstar_cma_shrink_lock(struct ion_heap *heap)
{
	struct ion_mstar_cma_heap *mstar_cma_heap = to_mstar_cma_heap(heap);
	struct mstar_cma *mstar_cma = mstar_cma_heap->mstar_cma;
	unsigned long pfn, freed = 0, len = 0;
	struct page *page;
	struct cma_cache_node *node, *tmp;

	mutex_lock(&mstar_cma->cma->lock);
	spin_lock(&mstar_cma->cache_lock);

	list_for_each_entry_safe(node, tmp, &mstar_cma->cache_head, list){
		pfn = page_to_pfn(node->page);
		len = 0;

		CMA_BUG_ON(pfn<mstar_cma->cma->base_pfn || pfn>=mstar_cma->cma->base_pfn+mstar_cma->cma->count, "pfn=%lx,cma[%lx,%lx] \n",pfn,mstar_cma->cma->base_pfn,mstar_cma->cma->count);

		while(len < node->len){
			if(!test_and_clear_bit(pfn - mstar_cma->cma->base_pfn, mstar_cma->cma->bitmap)){
				BUG();
			}
			page = pfn_to_page(pfn);
			__free_page(page);
			pfn++;
			len += PAGE_SIZE;
		}
		list_del(&node->list);
		mstar_cma->cache_size -= node->len;
		mstar_cma->cache_page_count -= node->len / PAGE_SIZE;
		freed += node->len / PAGE_SIZE;
		kfree(node);
	}

	spin_unlock(&mstar_cma->cache_lock);
	mutex_unlock(&mstar_cma->cma->lock);

	return freed;
}

#ifdef CONFIG_MSTAR_CMAPOOL
struct device *ion_mstar_cma_get_dev(struct ion_heap *heap)
{
	struct ion_mstar_cma_heap *mstar_cma_heap = to_mstar_cma_heap(heap);
	return mstar_cma_heap->dev;
}
#endif

static struct ion_heap_ops ion_mstar_cma_ops = {
	.allocate = ion_mstar_cma_allocate,
	.free = ion_mstar_cma_free,
	//.map_dma = ion_mstar_cma_heap_map_dma,
	//.unmap_dma = ion_mstar_cma_heap_unmap_dma,
	.phys = ion_mstar_cma_phys,
	.map_user = ion_cma_heap_map_user,
	.map_kernel = ion_mstar_cma_map_kernel,
	.unmap_kernel = ion_mstar_cma_unmap_kernel,
	.shrink = ion_mstar_cma_shrink,
#ifdef CONFIG_MSTAR_CMAPOOL
	.get_dev = ion_mstar_cma_get_dev,
#endif
};

static struct ion_heap_ops ion_mstar_cma_ops_2 = {
	.allocate = ion_mstar_cma_allocate_2,
	.free = ion_mstar_cma_free_2,
	//.map_dma = ion_mstar_cma_heap_map_dma,
	//.unmap_dma = ion_mstar_cma_heap_unmap_dma,
	.phys = ion_mstar_cma_phys,
	.map_user = ion_cma_heap_map_user,
	.map_kernel = ion_mstar_cma_map_kernel,
	.unmap_kernel = ion_mstar_cma_unmap_kernel,
/*.shrink = ion_mstar_cma_shrink,*/
#ifdef CONFIG_MSTAR_CMAPOOL
	.get_dev = ion_mstar_cma_get_dev,
#endif
};

struct ion_heap *ion_mstar_cma_heap_create(struct ion_platform_heap *data)
{
	struct ion_mstar_cma_heap *mstar_cma_heap;
	struct cma *cma;
	//char name[64];
	struct mstar_cma_heap_private *mcma_private;

	mstar_cma_heap = kzalloc(sizeof(struct ion_mstar_cma_heap), GFP_KERNEL);

	if (!mstar_cma_heap)
		return ERR_PTR(-ENOMEM);

	mstar_cma_heap->heap.ops = &ion_mstar_cma_ops;
	/* get device from private heaps data, later it will be
	 * used to make the link with reserved CMA memory */

	mcma_private = (struct mstar_cma_heap_private *)data->priv;
	mstar_cma_heap->dev = (struct device*)(mcma_private->cma_dev);

	if (strncmp(mstar_cma_heap->dev->init_name, "MALI_GOP", 8) == 0) {
		gralloc_cma_device = mstar_cma_heap->dev;
		mstar_cma_heap->heap.ops = &ion_mstar_cma_ops_2;
	}
	printk("\033[31mFunction = %s, dev = %s, cma = %p, cma from 0x%lX to 0x%lX\033[m\n",
		__PRETTY_FUNCTION__, mstar_cma_heap->dev->init_name, mstar_cma_heap->dev->cma_area,
		mstar_cma_heap->dev->cma_area->base_pfn, mstar_cma_heap->dev->cma_area->base_pfn+mstar_cma_heap->dev->cma_area->count);
	BUG_ON(!mstar_cma_heap->dev);
	mstar_cma_heap->heap.type = ION_HEAP_TYPE_MSTAR_CMA;

	mstar_cma_heap->mstar_cma = kzalloc(sizeof(struct mstar_cma),GFP_KERNEL);
	if(!mstar_cma_heap->mstar_cma)
		goto err;

	//initialize the mstar cma private data
	cma = dev_get_cma_area(mstar_cma_heap->dev);
	if (!cma)
		goto err;

	mstar_cma_heap->mstar_cma->cma = cma;
	mstar_cma_heap->mstar_cma->zone = page_zone(pfn_to_page(cma->base_pfn));
	init_waitqueue_head(&mstar_cma_heap->mstar_cma->cma_swap_wait);
	mutex_init(&mstar_cma_heap->mstar_cma->contig_rtylock);

	mstar_cma_heap->mstar_cma->cache_size = 0;
	mstar_cma_heap->mstar_cma->cache_page_count = 0;
	mstar_cma_heap->mstar_cma->fail_alloc_count = 0;
	INIT_LIST_HEAD(&mstar_cma_heap->mstar_cma->cache_head);
	spin_lock_init(&mstar_cma_heap->mstar_cma->cache_lock);

	//only descrete cma need worker thread
	if(mcma_private->flag == DESCRETE_CMA){
		//sprintf(name,"cma_%s",dev_name(mstar_cma_heap->dev));
		//mstar_cma_heap->mstar_cma->swap_worker = kthread_run(cma_swap_worker_thread, mstar_cma_heap, name);
		//CMA_BUG_ON(IS_ERR(mstar_cma_heap->mstar_cma->swap_worker),"create swap worker thread error! \n");
	}
	return &mstar_cma_heap->heap;

err:
	printk("\033[35mFunction = %s, Line = %d,error! \033[m\n", __PRETTY_FUNCTION__, __LINE__);

	kfree(mstar_cma_heap);
	return ERR_PTR(-ENOMEM);
}

void ion_mstar_cma_heap_destroy(struct ion_heap *heap)
{
	struct ion_mstar_cma_heap *mstar_cma_heap = to_mstar_cma_heap(heap);

	kfree(mstar_cma_heap->mstar_cma);
	kfree(mstar_cma_heap);
}

//helper function
static int get_cma_freebits(struct cma *cma)
{
	int total_free	= 0;
	int start	= 0;
	int pos		= 0;
	int next	= 0;

	while(1){
		pos = find_next_zero_bit(cma->bitmap, cma->count, start);

		if(pos >= cma->count)
			break;
		start = pos + 1;
		next = find_next_bit(cma->bitmap, cma->count, start);

		if(next >= cma->count){
			total_free += (cma->count - pos);
			break;
		}

		total_free += (next - pos);
		start = next + 1;

		if(start >= cma->count)
			break;
	}

	return total_free;
}

void get_system_heap_info(struct cma *system_cma, int *mali_heap_info)
{
	int freebits;

	mutex_lock(&system_cma->lock);
	freebits = get_cma_freebits(system_cma);
	mali_heap_info[0] = system_cma->count - freebits;	// alloc
	mali_heap_info[1] = 0;
	mali_heap_info[2] = 0;
	mali_heap_info[3] = freebits;						// free
	mutex_unlock(&system_cma->lock);
}

void get_cma_heap_info(struct ion_heap *heap, int *mali_heap_info, char *name)
{
	struct ion_mstar_cma_heap *mstar_cma_heap = to_mstar_cma_heap(heap);
	int freebits;

	spin_lock(&mstar_cma_heap->mstar_cma->cache_lock);
	freebits = get_cma_freebits(mstar_cma_heap->mstar_cma->cma);
	mali_heap_info[0] = mstar_cma_heap->mstar_cma->cma->count - freebits;
	mali_heap_info[1] = mstar_cma_heap->mstar_cma->cache_page_count;
	mali_heap_info[2] = mstar_cma_heap->mstar_cma->fail_alloc_count;
	mali_heap_info[3] = freebits;

	spin_unlock(&mstar_cma_heap->mstar_cma->cache_lock);
	strncpy(name, dev_name(mstar_cma_heap->dev), CMA_HEAP_NAME_LENG);
}

int in_cma_range(struct ion_heap* heap, struct page* page)
{
	struct ion_mstar_cma_heap *mstar_cma_heap = to_mstar_cma_heap(heap);
	struct mstar_cma *mstar_cma = mstar_cma_heap->mstar_cma;
	unsigned long pfn = page_to_pfn(page);

	if(pfn >= mstar_cma->cma->base_pfn && pfn < mstar_cma->cma->base_pfn + mstar_cma->cma->count){
		return 1;
	}

	return 0;
}
