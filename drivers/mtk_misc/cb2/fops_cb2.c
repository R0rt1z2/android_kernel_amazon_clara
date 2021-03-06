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

/*-----------------------------------------------------------------------------
 *
 * $Author: dtvbm11 $
 * $Date: 2015/04/15 $
 * $RCSfile: fops_cb2.c,v $
 * $Revision: #1 $
 *
 *---------------------------------------------------------------------------*/

/** @file cb_mod.c
 *  Callback interface of MT53XX.
 */


//-----------------------------------------------------------------------------
// Include files
//-----------------------------------------------------------------------------
#include <linux/poll.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <asm/io.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include "cb2_low.h"
#include "mt53xx_cb2.h"
//-----------------------------------------------------------------------------
// Constant definitions
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Static Variables
//-----------------------------------------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
#define DECLARE_MUTEX_LOCKED(name) \
       struct semaphore name = __SEMAPHORE_INITIALIZER(name, 0)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 24)
#define DECLARE_MUTEX_LOCKED(name) __DECLARE_SEMAPHORE_GENERIC(name,0)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
#define USE_UNLOCK_IOCTL
#endif


#ifdef CB_TIEMR_DEBUG
typedef struct _private_data {
    HANDLE_T timer;
}private_data;
#endif


//#define CB_TIEMR_DEBUG
#define CB_TIMER_TIMEROUT 1000

//-----------------------------------------------------------------------------
// Static functions
//-----------------------------------------------------------------------------
#ifdef CB_TIEMR_DEBUG
static VOID _cb_timer_timeout(HANDLE_T  pt_tm_handle, VOID *pv_tag)
{
	CB_CALLBACK_EVENT_T *prCbEv = (CB_CALLBACK_EVENT_T *) pv_tag;

	if (prCbEv != NULL) {
		printk("_cb_timer_timeout Id(%d) Timer Out\n",
		       (int) prCbEv->rGetCb.eFctId);
	} else {
		printk("_cb_timer_timeout Timer Out\n");
	}
}
#endif


// Public functions
//-----------------------------------------------------------------------------


#ifndef USE_UNLOCK_IOCTL
int cb_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	     unsigned long arg)
#else
long cb_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
	int retval = 0;
	CB_SET_CALLBACK_T rSetCb;
	CB_CALLBACK_EVENT_T *prCbEv = NULL;
	int sig_return;
	CB_GET_PID_T rGetPidCb;
	CB_GET_CBQUEUE_COUNT_T rGetCbCount;
#if CB_MULTIPROCESS
	UINT32 u4DoneMask;
#endif

#ifdef CB_TIEMR_DEBUG
	private_data *pdata = (private_data *) file->private_data;
#endif

	switch (cmd) {
	case CB_SET_CALLBACK:
		if (!access_ok(VERIFY_READ, (void __user *) arg,
			       sizeof(CB_SET_CALLBACK_T))) {
			return -EFAULT;
		}

		if (copy_from_user(&rSetCb, (void __user *) arg,
				   sizeof(CB_SET_CALLBACK_T))) {
			return -EFAULT;
		}
#if CB_MULTIPROCESS_REPLACE
		retval =
		    _CB_RegMultiCbFct(rSetCb.eFctId, rSetCb.i4SubId,
				      rSetCb.rCbPid);
#else
		retval =
		    _CB_RegCbFct(rSetCb.eFctId, rSetCb.i4SubId,
				 rSetCb.rCbPid);
#endif
		break;
#if CB_MULTIPROCESS
	case CB_SET_MULTI_CALLBACK:
		if (!access_ok(VERIFY_READ, (void __user *) arg,
			       sizeof(CB_SET_CALLBACK_T))) {
			return -EFAULT;
		}

		if (copy_from_user(&rSetCb, (void __user *) arg,
				   sizeof(CB_SET_CALLBACK_T))) {
			return -EFAULT;
		}

		retval =
		    _CB_RegMultiCbFct(rSetCb.eFctId, rSetCb.i4SubId,
				      rSetCb.rCbPid);

		break;
#endif
	case CB_GET_CALLBACK:
		if (!access_ok(VERIFY_WRITE, (void __user *) arg,
			       sizeof(CB_SET_CALLBACK_T))) {
			//ASSERT(0);
			return -EFAULT;
		}
#ifdef CB_TIEMR_DEBUG
		x_timer_stop(pdata->timer);
#endif

#if CB_MULTIPROCESS
		sig_return =
		    _CB_PutThdIntoWQ((prCbEv =
				      _CB_GetEvent(&u4DoneMask)) != NULL);
#else
		sig_return =
		    _CB_PutThdIntoWQ((prCbEv = _CB_GetEvent()) != NULL);
#endif
		if (sig_return) {
			return -EINTR;
		}

		if (prCbEv != NULL) {
#ifdef CB_TIEMR_DEBUG
			x_timer_start(pdata->timer, CB_TIMER_TIMEROUT,
				      X_TIMER_FLAG_ONCE, _cb_timer_timeout,
				      (VOID *) prCbEv);
#endif
			//ASSERT((sizeof(CB_GET_CALLBACK_T) + prCbEv->rGetCb.i4TagSize) < CB_MAX_STRUCT_SIZE);

			if (copy_to_user
			    ((void __user *) arg, &(prCbEv->rGetCb),
			     sizeof(CB_GET_CALLBACK_T) +
			     prCbEv->rGetCb.i4TagSize)) {
				printk
				    ("copy_to_user error, user addr=0x%x\n",
				     (unsigned int) (void __user *) arg);
				retval = -EFAULT;
			}
#if CB_MULTIPROCESS
			_CB_FreeEvent(prCbEv, u4DoneMask);
#else
			kfree(prCbEv);
#endif
		}
		else {
			retval = -EFAULT;
		}
		break;

	case CB_UNSET_CALLBACK:
		if (!access_ok(VERIFY_READ, (void __user *) arg,
			       sizeof(CB_SET_CALLBACK_T))) {
			return -EFAULT;
		}

		if (copy_from_user(&rSetCb, (void __user *) arg,
				   sizeof(CB_SET_CALLBACK_T))) {
			return -EFAULT;
		}
#if CB_MULTIPROCESS_REPLACE
		retval =
		    _CB_UnRegMultiCbFct(rSetCb.eFctId, rSetCb.i4SubId,
					rSetCb.rCbPid);
#else
		retval =
		    _CB_UnRegCbFct(rSetCb.eFctId, rSetCb.i4SubId,
				   rSetCb.rCbPid);
#endif
		break;
#if CB_MULTIPROCESS
	case CB_UNSET_MULTI_CALLBACK:
		if (!access_ok(VERIFY_READ, (void __user *) arg,
			       sizeof(CB_SET_CALLBACK_T))) {
			return -EFAULT;
		}

		if (copy_from_user(&rSetCb, (void __user *) arg,
				   sizeof(CB_SET_CALLBACK_T))) {
			return -EFAULT;
		}

		retval =
		    _CB_UnRegMultiCbFct(rSetCb.eFctId, rSetCb.i4SubId,
					rSetCb.rCbPid);

		break;
#endif
	case CB_GET_PID:

		if (!access_ok(VERIFY_READ, (void __user *) arg,
			       sizeof(CB_GET_PID_T))) {
			return -EFAULT;
		}

		if (copy_from_user(&rGetPidCb, (void __user *) arg,
				   sizeof(CB_GET_PID_T))) {
			return -EFAULT;
		}

		rGetPidCb.rCbPid = _CB_GetPid(rGetPidCb.eFctId);
		//printk("GetPID %d\n",rGetPidCb.rCbPid);
		if (copy_to_user((void __user *) arg, &rGetPidCb,
				 sizeof(CB_GET_PID_T))) {
			printk("copy_to_user error, user addr=0x%x\n",
			       (unsigned int) (void __user *) arg);
			retval = -EFAULT;
		}
		break;

	case CB_GET_CBQUEUE_COUNT:

		if (!access_ok(VERIFY_READ, (void __user *) arg,
			       sizeof(CB_GET_CBQUEUE_COUNT_T))) {
			return -EFAULT;
		}

		if (copy_from_user(&rGetCbCount, (void __user *) arg,
				   sizeof(CB_GET_CBQUEUE_COUNT_T))) {
			return -EFAULT;
		}

		rGetCbCount.iCount = _CB_GetCbCount();
		printk("GetCbCount = %d\n", rGetCbCount.iCount);
		if (copy_to_user((void __user *) arg, &rGetCbCount,
				 sizeof(CB_GET_CBQUEUE_COUNT_T))) {
			printk("copy_to_user error, user addr=0x%x\n",
			       (unsigned int) (void __user *) arg);
			retval = -EFAULT;
		}
		break;

#if 0
	case CB_SET_TEMINATE:
		{
			int i4Dummy = 0;
			_CB_PutEvent(CB_MTAL_TEMINATE_TRIGGER,
				     (INT32) sizeof(int),
				     (void *) &i4Dummy);
		}
		break;
#endif

	default:
		//ASSERT(0);
		printk(KERN_ALERT "cb_ioctl: Error: Unknown cmd 0x%x\n", cmd);
		retval = -EFAULT;
		break;
	}

	return retval;
}


static int cb_open(struct inode *inode, struct file *file)
{
#ifdef CB_TIEMR_DEBUG
	private_data *pdata;
#endif

#ifdef CB_TIEMR_DEBUG
	pdata = kmalloc(sizeof(private_data), GFP_KERNEL);
	file->private_data = (void *) pdata;
	if (file->private_data == NULL) {
		return -1;
	}
	x_timer_create(&(pdata->timer));
#endif
	return 0;
}

extern void _CB_ClearPendingEventEntries(void);
extern void _CB_ClearCbIdArray(void);

static int cb_release(struct inode *inode, struct file *file)
{
	_CB_ClearPendingEventEntries();
	_CB_ClearCbIdArray();

#ifdef CB_TIEMR_DEBUG
	kfree(file->private_data);
#endif

	return 0;
}

struct file_operations cb_native_fops = {
#ifndef USE_UNLOCK_IOCTL
	.ioctl = cb_ioctl,
#else
	.unlocked_ioctl = cb_ioctl,
#endif
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl = cb_ioctl,
#endif
	.open = cb_open,
	.release = cb_release
};

EXPORT_SYMBOL(cb_native_fops);
