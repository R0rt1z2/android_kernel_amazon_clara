#ifndef __CMA_H__
#define __CMA_H__

/*
 * There is always at least global CMA area and a few optional
 * areas configured in kernel .config.
 */
#ifdef CONFIG_CMA_AREAS
#define MAX_CMA_AREAS	(1 + CONFIG_CMA_AREAS)

#else
#define MAX_CMA_AREAS	(0)

#endif

#if defined(CONFIG_MP_CMA_PATCH_CMA_MSTAR_DRIVER_BUFFER) || defined(CONFIG_MSTAR_IPAPOOL)
#define CMA_HEAP_MIUOFFSET_NOCARE (-1UL)
#endif

#ifdef CONFIG_MP_CMA_PATCH_CMA_MSTAR_DRIVER_BUFFER
#ifdef CONFIG_MP_CMA_PATCH_MBOOT_STR_USE_CMA
#define CMA_HEAP_NAME_LENG  32
#else
#define CMA_HEAP_NAME_LENG  32
#endif

struct CMA_BootArgs_Config {
	int miu;
	int heap_type;
	int pool_id;
	unsigned long start;  //for boot args this is miu offset
	unsigned long size;
	char name[CMA_HEAP_NAME_LENG];
};
#endif

#ifdef CONFIG_MP_CMA_PATCH_CMA_MSTAR_DRIVER_BUFFER
#include <linux/device.h>
extern struct page *dma_alloc_at_from_contiguous(struct device *dev, int count,
					unsigned int align, phys_addr_t at_addr);

extern struct page *dma_alloc_from_contiguous_direct(struct device *dev, int count,
					unsigned int align, long *retlen);
#ifdef CONFIG_MP_MMA_CMA_ENABLE
extern struct page *dma_alloc_at_from_contiguous_from_high_to_low(struct device *dev, int count,
				       unsigned int order, phys_addr_t at_addr);
#endif
#endif

struct cma;

extern unsigned long totalcma_pages;
extern phys_addr_t cma_get_base(const struct cma *cma);
extern unsigned long cma_get_size(const struct cma *cma);

extern int __init cma_declare_contiguous(phys_addr_t base,
			phys_addr_t size, phys_addr_t limit,
			phys_addr_t alignment, unsigned int order_per_bit,
			bool fixed, struct cma **res_cma);
extern int cma_init_reserved_mem(phys_addr_t base, phys_addr_t size,
					unsigned int order_per_bit,
					struct cma **res_cma);
extern struct page *cma_alloc(struct cma *cma, size_t count, unsigned int align);
extern bool cma_release(struct cma *cma, const struct page *pages, unsigned int count);
#endif
