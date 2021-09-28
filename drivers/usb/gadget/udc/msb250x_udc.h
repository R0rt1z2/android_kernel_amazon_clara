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

/*------------------------------------------------------------------------------
    PROJECT: MSB250x Linux BSP
    DESCRIPTION:
          MSB250x dual role USB device controllers


    HISTORY:
         6/11/2010     Calvin Hung    First Revision

-------------------------------------------------------------------------------*/
#ifndef _MSB250X_UDC_H
#define _MSB250X_UDC_H

/******************************************************************************
 * Include Files
 ******************************************************************************/
#include <linux/kernel.h>
#include <linux/cdev.h>
#include "ms_gvar.h"
#include "ms_udc.h"
/******************************************************************************
 * Constants
 ******************************************************************************/
#define USB_HIGH_SPEED 1
#define QC_BOARD 1
#define EP0_FIFO_SIZE 64
#define EP_FIFO_SIZE 512

#if defined(USB_HIGH_SPEED)
#define DEFAULT_POWER_STATE MSB250X_UDC_PWR_HS_EN
#else
#define DEFAULT_POWER_STATE 0x00
#endif

#ifndef MSB250X_UDC_MAJOR
#define MSB250X_UDC_MAJOR 0   /* dynamic major by default */
#endif

#ifndef MSB250X_UDC_NR_DEVS
#define MSB250X_UDC_NR_DEVS 1    /* msb250x_udc0 */
#endif

#define BIT0	0x0001
#define BIT1	0x0002
#define BIT2	0x0004
#define BIT3	0x0008
#define BIT4	0x0010
#define BIT5	0x0020
#define BIT6	0x0040
#define BIT7	0x0080
#define BIT8	0x0100


/******************************************************************************
 * Variables
 ******************************************************************************/
enum ep0_state
{
	EP0_IDLE,
	EP0_IN_DATA_PHASE,
	EP0_OUT_DATA_PHASE,
	EP0_END_XFER,
	EP0_STALL,
};

struct msb250x_ep
{
	struct list_head queue;
	unsigned long last_io;	/* jiffies timestamp */
	struct usb_gadget *gadget;
	struct msb250x_udc *dev;
	const struct usb_endpoint_descriptor *desc;
	struct usb_ep ep;
	u8 num;

	unsigned short fifo_size;
	u8 bEndpointAddress;
	u8 bmAttributes;
	u8 DmaRxMode1;
	u8 RxShort;
	u16 wMaxPacketSize;
	u8 ch;
#if defined(TIMER_PATCH)
	//for mode1 dma sw trigger irq
	u16 wtd_dma_count;
	u16 wtd_rx_count;
	int sw_ep_irq;
#endif

	unsigned halted : 1;
	unsigned already_seen : 1;
	unsigned setup_stage : 1;
};

struct msb250x_request
{
	struct list_head		queue;		/* ep's requests */
	struct usb_request		req;
};

struct msb250x_udc
{
	spinlock_t lock;

	struct msb250x_ep ep[MSB250X_ENDPOINTS];
	struct usb_gadget gadget;
	struct usb_gadget_driver	*driver;
	struct msb250x_request fifo_req;
	struct platform_device		*pdev;
	u8 fifo_buf[EP_FIFO_SIZE];
	u16 devstatus;
	int address;
	int ep0state;
	unsigned got_irq : 1;
	unsigned req_std : 1;
	unsigned req_config : 1;
	unsigned req_pending : 1;
	unsigned enabled:1;
	u8 vbus;
	u8 DmaRxMode;
	struct semaphore sem;     /* Mutual exclusion semaphore     */
	struct cdev cdev;	      /* Char device structure */
	wait_queue_head_t event_q; /* Wait event queue. Now, only connection status change event. */
	int conn_chg; /* flag for connect status change event. */
	unsigned active_suspend : 1;
#if defined(TIMER_PATCH)
	int remap_irq;
#endif

	struct device			dev;
	struct list_head		list;
	unsigned int	irq;			/* irq number */
};

/******************************************************************************
 * Function definition
 ******************************************************************************/
static inline u32 udc_read8(uintptr_t reg)
{
	//return INREG8(reg);
	return readb((void *)reg);
}

static inline void udc_write8(u32 value, uintptr_t reg)
{
	//OUTREG8(reg, value);
	writeb(value, (void *)reg);
}

static inline u32 udc_read16(uintptr_t reg)
{
	//return INREG16(reg);
	return readw((void *)reg);
}

static inline void udc_write16(u32 value, uintptr_t reg)
{
	//OUTREG16(reg, value);
	writew(value, (void *)reg);
}

s8 msb250x_udc_schedule_done(struct msb250x_ep *);

#endif /* _MSB250X_UDC_H */
