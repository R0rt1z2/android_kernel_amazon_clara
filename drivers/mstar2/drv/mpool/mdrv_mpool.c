/* SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause */
/******************************************************************************
 *
 * This file is provided under a dual license.  When you use or
 * distribute this software, you may choose to be licensed under
 * version 2 of the GNU General Public License ("GPLv2 License")
 * or BSD License.
 *
 * GPLv2 License
 *
 * Copyright(C) 2019 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 *
 * BSD LICENSE
 *
 * Copyright(C) 2019 MediaTek Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// file    mdrv_mpool.c
/// @brief  Memory Pool Control Interface
/// @author MStar Semiconductor Inc.
///////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------
//  Include Files
//-------------------------------------------------------------------------------------------------
//#include "MsCommon.h"
#include <generated/autoconf.h>
//#include <linux/undefconf.h> //unused header file now
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/cdev.h>
#include <linux/time.h>  //added
#include <linux/timer.h> //added
#include <linux/device.h>
#include <linux/version.h>
#include <asm/io.h>
#include <asm/types.h>
#include <asm/cacheflush.h>
#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif

#ifdef CONFIG_HAVE_HW_BREAKPOINT
#include <linux/hw_breakpoint.h>
#endif

#if defined(CONFIG_MIPS)
#include <asm/mips-boards/prom.h>
#include "mdrv_cache.h"
#elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
#endif

#include "mst_devid.h"
#include "mdrv_mpool.h"
#include "mhal_mpool.h"
#include "mdrv_types.h"
#include "mst_platform.h"
#include "mdrv_system.h"
#include "mdrv_mstypes.h"

#include "chip_setup.h"

//-------------------------------------------------------------------------------------------------
// Macros
//-------------------------------------------------------------------------------------------------
//#define MPOOL_DPRINTK(fmt, args...) printk(KERN_WARNING"%s:%d " fmt,__FUNCTION__,__LINE__,## args)
#define MPOOL_DPRINTK(fmt, args...)

//-------------------------------------------------------------------------------------------------
//  Global variables
//-------------------------------------------------------------------------------------------------
#if defined(CONFIG_MIPS)
unsigned int bMPoolInit = 0;
#endif
//-------------------------------------------------------------------------------------------------
//  Local Defines
//-------------------------------------------------------------------------------------------------
#define MPOOL_VERSION 1
#define MOD_MPOOL_DEVICE_COUNT     1
#define MOD_MPOOL_NAME             "malloc"

#define KER_CACHEMODE_UNCACHE_NONBUFFERABLE 0
#define KER_CACHEMODE_CACHE   1
#define KER_CACHEMODE_UNCACHE_BUFFERABLE 2

#if defined(CONFIG_MP_MMAP_MMAP_BOUNDARY_PROTECT)
#define KER_CACHEMODE_TRAP                          3

#define KER_CACHEMODE_UNCACHE_NONBUFFERABLE_PROTECT 4
#define KER_CACHEMODE_CACHE_PROTECT                 5
#define KER_CACHEMODE_UNCACHE_BUFFERABLE_PROTECT    6
#endif


// Define MPOOL Device
U32 linux_base;
U32 linux_size;
U32 linux2_base;
U32 linux2_size;
U32 emac_base;
U32 emac_size;

#if defined(CONFIG_ARM) || defined(CONFIG_MIPS)
typedef  struct
{
   U32 mpool_base;
   U32 mpool_size;
   U32 mmap_offset;
   U32 mmap_size;
   U32 mmap_interval;
   U8  mmap_miusel;
   unsigned int u8MapCached;
   bool setflag;
}MMAP_FileData;
#elif defined(CONFIG_ARM64)
typedef  struct
{
   u64 mpool_base;
   u64 mpool_size;
   u64 mmap_offset;
   u64 mmap_size;
   u64 mmap_interval;
   U8  mmap_miusel;
   unsigned int u8MapCached;
   bool setflag;
}MMAP_FileData;

#endif

//bool setflag;
struct mutex mpool_iomap_mutex = __MUTEX_INITIALIZER(mpool_iomap_mutex);

unsigned int mpool_version = MPOOL_VERSION;

//-------------------------------------------------------------------------------------------------
//  Local Structurs
//-------------------------------------------------------------------------------------------------;

typedef struct
{
    int                         s32MPoolMajor;
    int                         s32MPoolMinor;
    void*                       dmaBuf;
    struct cdev                 cDevice;
    struct file_operations      MPoolFop;
} MPoolModHandle;

//-------------------------------------------------------------------------------------------------
//  Global Variables
//-------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// Forward declaration
//--------------------------------------------------------------------------------------------------
static int                      _MDrv_MPOOL_Open (struct inode *inode, struct file *filp);
static int                      _MDrv_MPOOL_Release(struct inode *inode, struct file *filp);
static int                      _MDrv_MPOOL_MMap(struct file *filp, struct vm_area_struct *vma);

static long                     _MDrv_MPOOL_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg);


#if defined(CONFIG_COMPAT)
static long Compat_MDrv_MPOOL_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#endif

//Add by austin, remove linux page from pfn range
static inline int mpool_io_remap_range(struct vm_area_struct *vma, unsigned long addr,
                unsigned long pfn, unsigned long size, pgprot_t prot);

//-------------------------------------------------------------------------------------------------
// Local Variables
//-------------------------------------------------------------------------------------------------

static struct class *mpool_class;

static MPoolModHandle MPoolDev=
{
    .s32MPoolMajor=               MDRV_MAJOR_MPOOL,
    .s32MPoolMinor=               MDRV_MINOR_MPOOL,
    .cDevice=
    {
        .kobj=                  {.name= MOD_MPOOL_NAME, },
        .owner  =               THIS_MODULE,
    },
    .MPoolFop=
    {
        .open=                  _MDrv_MPOOL_Open,
        .release=               _MDrv_MPOOL_Release,
        .mmap=                  _MDrv_MPOOL_MMap,
        .unlocked_ioctl=        _MDrv_MPOOL_Ioctl,
		#if defined(CONFIG_COMPAT)
		.compat_ioctl = Compat_MDrv_MPOOL_Ioctl,
		#endif
    },
};



//-------------------------------------------------------------------------------------------------
//  Debug Functions
//-------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
//  Local Functions
//-------------------------------------------------------------------------------------------------
static inline int mpool_io_remap_range(struct vm_area_struct *vma, unsigned long addr,
		    unsigned long pfn, unsigned long size, pgprot_t prot)
{
	unsigned long end = addr + PAGE_ALIGN(size);
	int err;
    vma->vm_flags |= VM_PFNMAP;

    do {
		/* pfn_valid(pfn) means the page is in linux memory
		 * we will also map linux memory to user_space
		 */
        //if(!pfn_valid(pfn))
		{
    		err = vm_insert_pfn(vma, addr, pfn);
    		if (err)
    			break;
        }
    }while(pfn++, addr += PAGE_SIZE, addr != end);

    return 0;
}

static int _MDrv_MPOOL_Open (struct inode *inode, struct file *filp)
{
    MMAP_FileData *mmapData;

    mmapData = kzalloc(sizeof(*mmapData), GFP_KERNEL);
    if (mmapData == NULL)
          return -ENOMEM;

    filp->private_data = mmapData;
    mmapData->u8MapCached = 1;
    mmapData->setflag = false;

    return 0;
}

static int _MDrv_MPOOL_Release(struct inode *inode, struct file *filp)
{
    MMAP_FileData *mmapData = filp->private_data ;
    kfree(mmapData);

    // iounmap(dev->dmaBuf) ;
    return 0;
}


static int _MDrv_MPOOL_MMap(struct file *filp, struct vm_area_struct *vma)
{
    MMAP_FileData *mmapData = filp->private_data;
    u32 miu0_len = 0x10000000;
	unsigned long BUS_BASE = 0;

    mutex_lock(&mpool_iomap_mutex);
    if(!mmapData->setflag)
		vma->vm_pgoff = mmapData->mpool_base >> PAGE_SHIFT;
    else
    {
    #if defined(CONFIG_MIPS)
        if(mmapData->mmap_miusel == 0)
            vma->vm_pgoff = mmapData->mmap_offset >> PAGE_SHIFT;
        else
            vma->vm_pgoff = (mmapData->mmap_offset+mmapData->mmap_interval) >> PAGE_SHIFT;
    #elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
        if(mmapData->mmap_miusel == 0)
            vma->vm_pgoff = (mmapData->mmap_offset+ARM_MIU0_BASE_ADDR) >> PAGE_SHIFT;
        else if(mmapData->mmap_miusel == 1)
            vma->vm_pgoff = (mmapData->mmap_offset+ARM_MIU1_BASE_ADDR) >> PAGE_SHIFT;
        else if(mmapData->mmap_miusel == 2)
            vma->vm_pgoff = (mmapData->mmap_offset+ARM_MIU2_BASE_ADDR) >> PAGE_SHIFT;
        else if(mmapData->mmap_miusel == 3)
            vma->vm_pgoff = (mmapData->mmap_offset+ARM_MIU3_BASE_ADDR) >> PAGE_SHIFT;
        else
            panic("miu%d not support\n",mmapData->mmap_miusel);
    #endif
    }

    /* set page to no cache */
    if((mmapData->u8MapCached == KER_CACHEMODE_CACHE)
#if defined(CONFIG_MP_MMAP_MMAP_BOUNDARY_PROTECT)
        || (mmapData->u8MapCached == KER_CACHEMODE_CACHE_PROTECT)
#endif
        )
    {
        #if defined(CONFIG_MIPS)
        pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
        pgprot_val(vma->vm_page_prot) |= _page_cachable_default;
        #elif defined(CONFIG_ARM)
        //vma->vm_page_prot=__pgprot_modify(vma->vm_page_prot, L_PTE_MT_MASK,L_PTE_MT_WRITEBACK);
        vma->vm_page_prot=__pgprot_modify(vma->vm_page_prot,L_PTE_MT_MASK,L_PTE_MT_DEV_CACHED);
		#elif defined(CONFIG_ARM64)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
		pgprot_val(vma->vm_page_prot) = pgprot_val(pgprot_cached(vma->vm_page_prot));
#else
		pgprot_val(vma->vm_page_prot) = pgprot_cached(vma->vm_page_prot);
#endif
        #endif
    }
#if defined(CONFIG_MP_MMAP_MMAP_BOUNDARY_PROTECT)
    else if (mmapData->u8MapCached == KER_CACHEMODE_TRAP)
    {
        #if defined(CONFIG_MIPS)
        pgprot_val(vma->vm_page_prot) = pgprot_val(PAGE_NONE);
        #elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
        pgprot_val(vma->vm_page_prot) = __PAGE_NONE;
        #endif
		printk("\033[35mUsing TRAP, vma->vm_flags is %lu, vma->vm_page_prot is %lu\033[m\n", vma->vm_flags, pgprot_val(vma->vm_page_prot));
    }
#endif
    else
    {
        #if defined(CONFIG_MIPS)
        pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
        pgprot_val(vma->vm_page_prot) |= _CACHE_UNCACHED;
        #elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
        if((mmapData->u8MapCached == KER_CACHEMODE_UNCACHE_BUFFERABLE)
#if defined(CONFIG_MP_MMAP_MMAP_BOUNDARY_PROTECT)
            || (mmapData->u8MapCached == KER_CACHEMODE_UNCACHE_BUFFERABLE_PROTECT)
#endif
            )
        {
		 #if defined(CONFIG_ARM)
//pgprot_val(vma->vm_page_prot) = pgprot_dmacoherent(vma->vm_page_prot);  //This solution is only to modify mpool mmaping rule,
//The Correct solution is to enable config_ARM_DMA_MEM_BUFFERABLE, but if enable config_ARM_DMA_MEM_BUFFERABLE , system will trigger the app to crash. 
//if enable config_ARM_DMA_MEM_BUFFERABLE maybe have coherence, so app will crash now. but this solution only modify mpoool mmaping, mpool always have flush pipe.
			pgprot_val(vma->vm_page_prot) = __pgprot_modify(vma->vm_page_prot, L_PTE_MT_MASK, L_PTE_MT_BUFFERABLE | L_PTE_XN);
		 #else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
			pgprot_val(vma->vm_page_prot) = pgprot_val(pgprot_dmacoherent(vma->vm_page_prot));
#else
			pgprot_val(vma->vm_page_prot) = pgprot_dmacoherent(vma->vm_page_prot);
#endif
	     #endif
        }
        else
        {
		 #if defined(CONFIG_ARM)
//pgprot_val(vma->vm_page_prot) = pgprot_dmacoherent(vma->vm_page_prot); //This solution is only to modify mpool mmaping rule,
//The Correct solution is to enable config_ARM_DMA_MEM_BUFFERABLE, but if enable config_ARM_DMA_MEM_BUFFERABLE , system will trigger the app to crash.
//if enable config_ARM_DMA_MEM_BUFFERABLE maybe have coherence, so app will crash now. but this solution only modify mpoool mmaping, mpool always have flush pipe. 
	      pgprot_val(vma->vm_page_prot)= __pgprot_modify(vma->vm_page_prot, L_PTE_MT_MASK, L_PTE_MT_BUFFERABLE | L_PTE_XN);
		 #else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,32)
			pgprot_val(vma->vm_page_prot) = pgprot_val(pgprot_dmacoherent(vma->vm_page_prot));
#else
			pgprot_val(vma->vm_page_prot) = pgprot_dmacoherent(vma->vm_page_prot);
#endif
	     #endif
        }
        #endif
    }

    if(mmapData->setflag)
    {
		if(mmapData->mmap_miusel == 0)
		{
			#if defined(CONFIG_MIPS)
			BUS_BASE = MIPS_MIU0_BUS_BASE;
			#if defined(CONFIG_MSTAR_KENYA) || defined(CONFIG_MSTAR_KERES)
			if(mmapData->mmap_offset >= 0x10000000)
				BUS_BASE += 0x40000000;
			#endif
			#elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
			BUS_BASE = ARM_MIU0_BUS_BASE;
			#endif
		}
		else if(mmapData->mmap_miusel == 1)
		{
			#if defined(CONFIG_MIPS)
			BUS_BASE = MIPS_MIU1_BUS_BASE;
			#elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
			BUS_BASE = ARM_MIU1_BUS_BASE;
			#endif
		}
		else if(mmapData->mmap_miusel == 2)
		{
			#if defined(CONFIG_ARM) || defined(CONFIG_ARM64)
			BUS_BASE = ARM_MIU2_BUS_BASE;
			#endif
		}
		else if(mmapData->mmap_miusel == 3)
		{
			#if defined(CONFIG_ARM) || defined(CONFIG_ARM64)
			BUS_BASE = ARM_MIU3_BUS_BASE;
			#endif
		}
        else
        {
            panic("miu%d not support\n",mmapData->mmap_miusel);
        }

#ifdef CONFIG_MP_MMAP_MMAP_BOUNDARY_PROTECT
		if ((mmapData->u8MapCached == KER_CACHEMODE_UNCACHE_NONBUFFERABLE_PROTECT) ||
			(mmapData->u8MapCached == KER_CACHEMODE_CACHE_PROTECT) ||
			(mmapData->u8MapCached == KER_CACHEMODE_UNCACHE_BUFFERABLE_PROTECT)) {
			pgprot_t temp_prot;

            #if defined(CONFIG_MIPS)
			printk("\033[35m[MMAP]Boundary Protect for MIPS\033[m\n");

			// we  divide into 3 sectios to do mpool_io_remap_range
            mmapData->mmap_size -= 0x2000;
            pgprot_val(temp_prot) = pgprot_val(vma->vm_page_prot);

            // 1. Upper Boundary
            pgprot_val(vma->vm_page_prot) = pgprot_val(PAGE_NONE);
			if(io_remap_pfn_range(vma, vma->vm_start,
                    (BUS_BASE+mmapData->mmap_offset) >> PAGE_SHIFT, 0x1000,
                            vma->vm_page_prot))
            {
                mutex_unlock(&mpool_iomap_mutex);
                return -EAGAIN;
            }

            // 2. Lower Boundary
            pgprot_val(vma->vm_page_prot) = pgprot_val(PAGE_NONE);
			if(io_remap_pfn_range(vma, vma->vm_start+mmapData->mmap_size+0x1000,
                    (BUS_BASE+mmapData->mmap_offset+mmapData->mmap_size-0x1000) >> PAGE_SHIFT, 0x1000,
                            vma->vm_page_prot))
            {
                mutex_unlock(&mpool_iomap_mutex);
                return -EAGAIN;
            }

            // 3. Main Area
            pgprot_val(vma->vm_page_prot) = pgprot_val(temp_prot);
			if(io_remap_pfn_range(vma, vma->vm_start+0x1000,
                    (BUS_BASE+mmapData->mmap_offset) >> PAGE_SHIFT, mmapData->mmap_size,
                            vma->vm_page_prot))
            {
                mutex_unlock(&mpool_iomap_mutex);
                return -EAGAIN;
            }
            #elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
			printk("\033[35m[MMAP]Boundary Protect for ARM\033[m\n");

            // we  divide into 3 sectios to do mpool_io_remap_range
            mmapData->mmap_size -= 0x2000;
            temp_prot = vma->vm_page_prot;

            // 1. Upper Boundary
            pgprot_val(vma->vm_page_prot) = __PAGE_NONE;
            if(mpool_io_remap_range(vma, vma->vm_start,
                            (BUS_BASE+mmapData->mmap_offset) >> PAGE_SHIFT, 0x1000,
                                    vma->vm_page_prot))
            {
            	mutex_unlock(&mpool_iomap_mutex);
                return -EAGAIN;
            }

            // 2. Lower Boundary
            pgprot_val(vma->vm_page_prot) = __PAGE_NONE;
            if(mpool_io_remap_range(vma, vma->vm_start+mmapData->mmap_size+0x1000,
            				(BUS_BASE+mmapData->mmap_offset+mmapData->mmap_size-0x1000) >> PAGE_SHIFT, 0x1000,
                            		vma->vm_page_prot))
            {
                mutex_unlock(&mpool_iomap_mutex);
                return -EAGAIN;
            }

            // 3. Main Area
            pgprot_val(vma->vm_page_prot) = temp_prot;
            if(mpool_io_remap_range(vma, vma->vm_start+0x1000,
                            (BUS_BASE+mmapData->mmap_offset) >> PAGE_SHIFT, mmapData->mmap_size,
            						vma->vm_page_prot))
            {
                mutex_unlock(&mpool_iomap_mutex);
                return -EAGAIN;
            }
            #endif
		}
        else
#endif
		{
            #if defined(CONFIG_MIPS)
            if(io_remap_pfn_range(vma, vma->vm_start,
                    (BUS_BASE+mmapData->mmap_offset) >> PAGE_SHIFT, mmapData->mmap_size,
                            vma->vm_page_prot))
            #elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
            if(mpool_io_remap_range(vma, vma->vm_start,
                    (BUS_BASE+mmapData->mmap_offset) >> PAGE_SHIFT, mmapData->mmap_size,
                            vma->vm_page_prot))
            #endif
            {
                mutex_unlock(&mpool_iomap_mutex);
                return -EAGAIN;
            }
		}
	}
	else
	{


		unsigned long u32MIU0_MapStart = 0;
		unsigned long u32MIU0_MapSize = 0;
		unsigned long u32MIU1_MapStart = 0;
		unsigned long u32MIU1_MapSize = 0;


        //calculate map size & start
        if(mmapData->mpool_base<miu0_len)
        {
            u32MIU0_MapStart=mmapData->mpool_base;

            if((mmapData->mpool_base+mmapData->mpool_size)>miu0_len)
            {
                u32MIU0_MapSize=(miu0_len-mmapData->mpool_base);
                u32MIU1_MapSize=mmapData->mpool_size-u32MIU0_MapSize;
                #if defined(CONFIG_MIPS)
                u32MIU1_MapStart=MIPS_MIU1_BUS_BASE;
                #elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
                u32MIU1_MapStart=ARM_MIU1_BUS_BASE;
                #endif
            }
	    	else
            {
                u32MIU0_MapSize=mmapData->mpool_size;
                u32MIU1_MapSize=0;
                #if defined(CONFIG_MIPS)
                u32MIU1_MapStart=MIPS_MIU1_BUS_BASE;
                #elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
                u32MIU1_MapStart=ARM_MIU1_BUS_BASE;
                #endif
            }
        }
        else
        {
            u32MIU0_MapStart=0;
            u32MIU0_MapSize=0;
            #if defined(CONFIG_MIPS)
            u32MIU1_MapStart=(mmapData->mpool_base-miu0_len)+MIPS_MIU1_BUS_BASE;
            #elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
            u32MIU1_MapStart=ARM_MIU1_BUS_BASE;
            #endif
            u32MIU1_MapSize=mmapData->mpool_size;
        }
        //printk("MPOOL MAP INFORMATION:\n");
        //printk("    MIU0 Length=0x%08X\n",miu0_len);
        //printk("    MIU0 MAP:0x%08lX,0x%08lX\n",u32MIU0_MapStart,u32MIU0_MapSize);
        //printk("    MIU1 MAP:0x%08lX,0x%08lX\n",u32MIU1_MapStart,u32MIU1_MapSize);

        if(u32MIU0_MapSize)
        {
            if (mpool_io_remap_range(vma, vma->vm_start+ mmapData->mpool_base,
                                        u32MIU0_MapStart >> PAGE_SHIFT, u32MIU0_MapSize,
                                        vma->vm_page_prot))
            {
              mutex_unlock(&mpool_iomap_mutex);
              return -EAGAIN;
            }
        }

        if(u32MIU1_MapSize)
        {
			#if defined(CONFIG_MIPS)
           	if(io_remap_pfn_range(vma, vma->vm_start+u32MIU0_MapSize,
                                       MIPS_MIU1_BUS_BASE >> PAGE_SHIFT, u32MIU1_MapSize,
                                       vma->vm_page_prot))
			#elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
			if(mpool_io_remap_range(vma, vma->vm_start+u32MIU0_MapSize,
                                       ARM_MIU1_BUS_BASE >> PAGE_SHIFT, u32MIU1_MapSize,
                                       vma->vm_page_prot))
            #endif
			{
				mutex_unlock(&mpool_iomap_mutex);
				return -EAGAIN;
			}
		}
	}

	mutex_unlock(&mpool_iomap_mutex);
    return 0;
}

#if defined(CONFIG_COMPAT)
static long Compat_MDrv_MPOOL_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg){
	int	err= 0;

	switch(cmd)
	{
		case MPOOL_IOC_INFO:
		case MPOOL_IOC_FLUSHDCACHE:
		case MPOOL_IOC_SET:
		case MPOOL_IOC_KERNEL_DETECT:
		case MPOOL_IOC_VERSION:
		case MPOOL_IOC_FLUSHDCACHE_ALL:
		case MPOOL_IOC_GET_BLOCK_OFFSET:
		case MPOOL_IOC_PA2BA:
		case MPOOL_IOC_BA2PA:
		case MPOOL_IOC_SET_MAP_CACHE:
		{
			return filp->f_op->unlocked_ioctl(filp, cmd,(unsigned long)compat_ptr(arg));
		}
		case COMPAT_MPOOL_IOC_FLUSHDCACHE_PAVA:
		{
			compat_u64 u64;
			compat_size_t u;

			DrvMPool_Flush_Info_t32 __user *data32;
			DrvMPool_Flush_Info_t __user *data;
			data = compat_alloc_user_space(sizeof(*data));
			if (data == NULL)
				return -EFAULT;

			data32 = compat_ptr(arg);
			err = get_user(u, &data32->u32AddrVirt);
			err |= put_user(u, &data->u32AddrVirt);
			err |= get_user(u64, &data32->u32AddrPhys);
			err |= put_user(u64, &data->u32AddrPhys);
			err |= get_user(u64, &data32->u32Size);
			err |= put_user(u64, &data->u32Size);
			if (err)
				return err;

			return filp->f_op->unlocked_ioctl(filp, MPOOL_IOC_FLUSHDCACHE_PAVA,(unsigned long)data);
		}
#ifdef CONFIG_HAVE_HW_BREAKPOINT
		case MPOOL_IOC_SETWATCHPT:
			break;
		case MPOOL_IOC_GETWATCHPT:
			break;
#endif
		default:
			return -ENOIOCTLCMD;
	}
	return -ENOIOCTLCMD;
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
static long _MDrv_MPOOL_Ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
#else
static int _MDrv_MPOOL_Ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
#endif
{
    int         err= 0;
    int         ret= 0;

    MMAP_FileData *mmapData = filp->private_data ;

    /*
     * extract the type and number bitfields, and don't decode
     * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
     */
    if (MPOOL_IOC_MAGIC!= _IOC_TYPE(cmd))
    {
        return -ENOTTY;
    }

    /*
     * the direction is a bitmask, and VERIFY_WRITE catches R/W
     * transfers. `Type' is user-oriented, while
     * access_ok is kernel-oriented, so the concept of "read" and
     * "write" is reversed
     */
    if (_IOC_DIR(cmd) & _IOC_READ)
    {
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    }
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
    {
        err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    }
    if (err)
    {
        return -EFAULT;
    }

    // @FIXME: Use a array of function pointer for program readable and code size later
    switch(cmd)
    {
    //------------------------------------------------------------------------------
    // Signal
    //------------------------------------------------------------------------------
    case MPOOL_IOC_INFO:
        {
            DrvMPool_Info_t i ;
	    memset(&i, 0, sizeof(DrvMPool_Info_t));

            i.u32Addr = mmapData->mpool_base;
            i.u32Size = mmapData->mpool_size;
            MPOOL_DPRINTK("MPOOL_IOC_INFO i.u32Addr = %d\n", i.u32Addr);
            MPOOL_DPRINTK("MPOOL_IOC_INFO i.u32Size = %d\n", i.u32Size);

            ret= copy_to_user( (void *)arg, &i, sizeof(i) );
        }
        break;
    case MPOOL_IOC_FLUSHDCACHE:        
	    MDrv_MPOOL_IOC_FlushDache(arg);	
            
        
        break;
    case MPOOL_IOC_FLUSHDCACHE_PAVA:
        {

            DrvMPool_Flush_Info_t i ;
            ret= copy_from_user(&i, (void __user *)arg, sizeof(i));

           /*Compare "u32AddrPhys" with "miu_base" to decide if which miu is located*/
           if(i.u32AddrPhys >= ARM_MIU3_BASE_ADDR)
               Chip_Flush_Cache_Range_VA_PA(i.u32AddrVirt, (i.u32AddrPhys - ARM_MIU3_BASE_ADDR) + ARM_MIU3_BUS_BASE , i.u32Size);
           if((i.u32AddrPhys >= ARM_MIU2_BASE_ADDR) && (i.u32AddrPhys < ARM_MIU3_BASE_ADDR))
               Chip_Flush_Cache_Range_VA_PA(i.u32AddrVirt, (i.u32AddrPhys - ARM_MIU2_BASE_ADDR) + ARM_MIU2_BUS_BASE , i.u32Size);
           if((i.u32AddrPhys >= ARM_MIU1_BASE_ADDR) && (i.u32AddrPhys < ARM_MIU2_BASE_ADDR))
               Chip_Flush_Cache_Range_VA_PA(i.u32AddrVirt, (i.u32AddrPhys - ARM_MIU1_BASE_ADDR) + ARM_MIU1_BUS_BASE , i.u32Size);
           else
               Chip_Flush_Cache_Range_VA_PA(i.u32AddrVirt, i.u32AddrPhys + ARM_MIU0_BUS_BASE , i.u32Size);
    	}
        break ;
    case MPOOL_IOC_GET_BLOCK_OFFSET:
        {
            DrvMPool_Info_t i ;

	    memset(&i, 0, sizeof(DrvMPool_Info_t));
            ret= copy_from_user( &i, (void __user *)arg, sizeof(i) );
            #if defined(__aarch64__)
            MDrv_SYS_GetMMAP((int)i.u32Addr, &(i.u32Addr), &(i.u32Size)) ;
            #else
            MDrv_SYS_GetMMAP((int)i.u32Addr, (unsigned int *)&(i.u32Addr), (unsigned int *)&(i.u32Size)) ;
            #endif
            ret= copy_to_user( (void __user *)arg, &i, sizeof(i) );
        }
        break ;
    case MPOOL_IOC_SET_MAP_CACHE:
        {
            ret= copy_from_user(&mmapData->u8MapCached, (void __user *)arg, sizeof(mmapData->u8MapCached));
        }
        break;
    case MPOOL_IOC_SET:
        {
           	DrvMPool_Info_t i;

	    memset(&i, 0, sizeof(DrvMPool_Info_t));
           	ret= copy_from_user(&i, (void __user *)arg, sizeof(i));
            mmapData->setflag = true;
            mmapData->mmap_offset = i.u32Addr;
            mmapData->mmap_size = i.u32Size;
            mmapData->mmap_interval = i.u32Interval;
            mmapData->mmap_miusel = i.u8MiuSel;
        }
        break;

	case MPOOL_IOC_KERNEL_DETECT:
			{
				DrvMPool_Kernel_Info_t i;
                		i.u32lxAddr = linux_base;
				i.u32lxSize = linux_size;
				i.u32lx2Addr = linux2_base;
				i.u32lx2Size = linux2_size;

                printk("lxaddr = %08llx, lxsize = %08llx\n", i.u32lxAddr, i.u32lxSize);
                printk("lx2addr = %08llx, lx2size = %08llx\n", i.u32lx2Addr, i.u32lx2Size);
				ret= copy_to_user( (void *)arg, &i, sizeof(i) );
			}
			break;
    case MPOOL_IOC_VERSION:
        {
            ret= copy_to_user( (void *)arg, &mpool_version, sizeof(mpool_version) );
        }
	    break;

    case MPOOL_IOC_FLUSHDCACHE_ALL:
    {
#if !(defined(CONFIG_MSTAR_TITANIA3) || defined(CONFIG_MSTAR_TITANIA10) )
         Chip_Flush_Cache_All();
#endif
    }
    break ;
#ifdef CONFIG_HAVE_HW_BREAKPOINT
    //edit by york
    case MPOOL_IOC_SETWATCHPT:
    {
        DrvMPool_Watchpt_Info_t info;
        ret = copy_from_user(&info, (void __user *)arg, sizeof(info));
#ifdef CONFIG_ARM
{
	unsigned int tmp,WCR;

	if(info.rwx == 0)	/*read*/
		WCR = 0x1EF;
	else if(info.rwx == 1)	/*write*/
		WCR = 0x1F7;
        else			/*read,write*/
		WCR = 0x1FF;

	ARM_DBG_WRITE(c0, c0, 6, info.u32AddrVirt);
	tmp = (info.mask << 24)| WCR ;/*shift 24 is because the mask control bit is defined there*/
		 ARM_DBG_WRITE(c0, c0, 7, tmp);

	/*printk("The input 0 is:%#x and the mask bit is:%#x\n",tmp,info.mask);
	tmp = 0;
	tmp = info.u32AddrVirt | (1  << (info.mask * 4))
	asm volatile(
			input[0] = ;
        "mov	r1, %[i0]\n\t"									\
        "mov	%[o1], r1\n\t"									\
        : [o1] "=r"(out)									\
	: [i0] "g"(tmp)									\
        : "memory"                                                                              \
    	);
	printk("The input 0 is:%#x and  output[1] is :%#x, the size is:%#x\n",tmp,out,info.mask);*/
        printk("The register is written\n");
}
#elif defined(CONFIG_ARM64)
#else
        if(info.global == 1)
                 write_c0_watchhi0(0x40000000);
#endif

     }
     break ;
     case MPOOL_IOC_GETWATCHPT:
     {

	#ifdef CONFIG_ARM
	char str[200];
        DrvMPool_Wcvr_Info_t info;
	int m;
        ARM_DBG_READ(c0, c1, 6, info.wvr1);
	for(m = 0; m < 10000; m++);
        ARM_DBG_READ(c0, c0, 6, info.wvr0);
	for(m = 0; m < 10000; m++);
        ARM_DBG_READ(c0, c1, 7, info.wcr1);
	for(m = 0; m < 10000; m++);
        ARM_DBG_READ(c0, c0, 7, info.wcr0);
	for(m = 0; m < 10000; m++);
	sprintf(str,"ARM HW watchpoint register,the wvr0 is:%#x,wvr1 is:%#x,wcr0 is:%#x,wcr1 is:%#x",info.wvr0,info.wvr1,info.wcr0,info.wcr1);
	ret = copy_to_user( (void *)arg, str, sizeof(str) );
	#endif
     }
     break;
#endif //CONFIG_HAVE_HW_BREAKPOINT

	case MPOOL_IOC_PA2BA:
	{
		MS_PHY64 bus_address = 0;
		MS_PHY64 phy_address = 0;
		ret= copy_from_user(&phy_address, (void __user *)arg, sizeof(MS_PHY64));
#if ARM_MIU0_BASE_ADDR != 0
		if( (phy_address >= ARM_MIU0_BASE_ADDR) && (phy_address < ARM_MIU1_BASE_ADDR) ) // MIU0
#else
		if(phy_address < ARM_MIU1_BASE_ADDR) // MIU0
#endif
			bus_address = phy_address - ARM_MIU1_BASE_ADDR + ARM_MIU0_BUS_BASE;
		else if( (phy_address >= ARM_MIU1_BASE_ADDR) && (phy_address < ARM_MIU2_BASE_ADDR) )    // MIU1
			bus_address = phy_address - ARM_MIU1_BASE_ADDR + ARM_MIU1_BUS_BASE;
		else
			bus_address = phy_address - ARM_MIU2_BASE_ADDR + ARM_MIU2_BUS_BASE;    // MIU2

        if (bus_address == 0)
            return -EFAULT;

		ret |= copy_to_user( (void *)arg, (void __user*)bus_address, sizeof(MS_PHY64));
		break;
	}
	case MPOOL_IOC_BA2PA:
	{
		MS_PHY64 bus_address = 0;
		MS_PHY64 phy_address = 0;
		ret= copy_from_user(&bus_address, (void __user *)arg, sizeof(MS_PHY64));
		if( (bus_address >= ARM_MIU0_BUS_BASE) && (bus_address < ARM_MIU1_BUS_BASE) ) // MIU0
			phy_address = bus_address - ARM_MIU0_BUS_BASE + ARM_MIU0_BASE_ADDR;
		else if( (bus_address >= ARM_MIU1_BUS_BASE) && (bus_address < ARM_MIU2_BUS_BASE) ) // MIU1
			phy_address = bus_address - ARM_MIU1_BUS_BASE + ARM_MIU1_BASE_ADDR;
		else
			phy_address = bus_address - ARM_MIU2_BUS_BASE + ARM_MIU2_BASE_ADDR; // MIU2

        if (phy_address == 0)
            return -EFAULT;

		ret |= copy_to_user( (void *)arg, (void __user*)phy_address, sizeof(MS_PHY64) );
		break;
	}
    default:
        printk("Unknown ioctl command %d\n", cmd);
        return -ENOTTY;
    }
    return 0;
}


//extern unsigned int MDrv_SYS_GetDRAMLength(void);

MSYSTEM_STATIC int __init mod_mpool_init(void)
{
    int s32Ret;
    dev_t dev;

    //MDrv_SYS_GetMMAP(E_SYS_MMAP_LINUX_BASE, &mpool_size, &mpool_base);
    //mpool_size = MDrv_SYS_GetDRAMLength()-mpool_base ;

    #if defined(CONFIG_MIPS)
    //get_boot_mem_info(LINUX_MEM, &linux_base, &linux_size);
    //get_boot_mem_info(LINUX_MEM2, &linux2_base, &linux2_size);
    //get_boot_mem_info(MPOOL_MEM, &mpool_base, &mpool_size);
    //get_boot_mem_info(EMAC_MEM, &emac_base, &emac_size);
    #elif defined(CONFIG_ARM) || defined(CONFIG_ARM64)
    //add here later

    #endif

    //printk( "\nMpool base=0x%08X\n", mpool_base );
    //printk( "\nMpool size=0x%08X\n", mpool_size );

    mpool_class = class_create(THIS_MODULE, "mpool");
    if (IS_ERR(mpool_class))
    {
        return PTR_ERR(mpool_class);
    }

    if (MPoolDev.s32MPoolMajor)
    {
        dev = MKDEV(MPoolDev.s32MPoolMajor, MPoolDev.s32MPoolMinor);
        s32Ret = register_chrdev_region(dev, MOD_MPOOL_DEVICE_COUNT, MOD_MPOOL_NAME);
    }
    else
    {
        s32Ret = alloc_chrdev_region(&dev, MPoolDev.s32MPoolMinor, MOD_MPOOL_DEVICE_COUNT, MOD_MPOOL_NAME);
        MPoolDev.s32MPoolMajor = MAJOR(dev);
    }

    if ( 0 > s32Ret)
    {
        MPOOL_DPRINTK("Unable to get major %d\n", MPoolDev.s32MPoolMajor);
        class_destroy(mpool_class);
        return s32Ret;
    }

    cdev_init(&MPoolDev.cDevice, &MPoolDev.MPoolFop);
    if (0!= (s32Ret= cdev_add(&MPoolDev.cDevice, dev, MOD_MPOOL_DEVICE_COUNT)))
    {
        MPOOL_DPRINTK("Unable add a character device\n");
        unregister_chrdev_region(dev, MOD_MPOOL_DEVICE_COUNT);
        class_destroy(mpool_class);
        return s32Ret;
    }


    device_create(mpool_class, NULL, dev, NULL, MOD_MPOOL_NAME);
#ifdef CONFIG_MIPS
	bMPoolInit = 1;
#endif
    return 0;
}

int MDrv_MPOOL_IOC_FlushDache(unsigned long arg){
	return MHal_MPOOL_IOC_FlushDache(arg);
}
MSYSTEM_STATIC void __exit mod_mpool_exit(void)
{
    cdev_del(&MPoolDev.cDevice);
    unregister_chrdev_region(MKDEV(MPoolDev.s32MPoolMajor, MPoolDev.s32MPoolMinor), MOD_MPOOL_DEVICE_COUNT);

    device_destroy(mpool_class, MKDEV(MPoolDev.s32MPoolMajor, MPoolDev.s32MPoolMinor));
    class_destroy(mpool_class);
}
#if defined(CONFIG_MSTAR_MPOOL) || defined(CONFIG_MSTAR_MPOOL_MODULE)
module_init(mod_mpool_init);
module_exit(mod_mpool_exit);

MODULE_AUTHOR("MSTAR");
MODULE_DESCRIPTION("MPOOL driver");
MODULE_LICENSE("GPL");
#endif//#if defined(CONFIG_MSTAR_MPOOL) || defined(CONFIG_MSTAR_MPOOL_MODULE)
