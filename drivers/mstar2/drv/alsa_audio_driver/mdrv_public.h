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

#ifndef _MDRV_ALSA_PUBLIC_HEADER
#define _MDRV_ALSA_PUBLIC_HEADER

/*
 * ============================================================================
 * Constant Definitions
 * ============================================================================
 */
#define MAD_TRUE    1
#define MAD_FALSE    0
#define MAD_MAX_DEVICES    8
#define MAD_MAX_SUBSTREAMS    8

#define MSTAR_SND_CARDS		2

#define MAD_PRINT(fmt, args...)    printk("[%010u][MAD_ALSA][%06d]     " fmt, jiffies_to_msecs(jiffies), __LINE__, ## args)

enum MStar_GET_CMD {
	/* Playback */
	E_PCM_PLAYBACK_GET_BUFFER_SIZE = 0,
	E_PCM_PLAYBACK_GET_PERIOD_SIZE,
	E_PCM_PLAYBACK_GET_SAMPLE_RATE,
	E_PCM_PLAYBACK_GET_CHANNEL_MODE,
	E_PCM_PLAYBACK_GET_MAX_CHANNEL,
	E_PCM_PLAYBACK_GET_CONSTRAINTS_COUNT,
	E_PCM_PLAYBACK_GET_CONSTRAINTS_LIST,
	E_PCM_PLAYBACK_GET_CONSTRAINTS_MASK,
	E_PCM_PLAYBACK_GET_BUFFER_INUSED_BYTES,
	E_PCM_PLAYBACK_GET_BUFFER_AVAIL_BYTES,
	E_PCM_PLAYBACK_GET_BUFFER_CONSUMED_BYTES,
	E_PCM_PLAYBACK_GET_DEVICE_STATUS,
	E_PCM_PLAYBACK_GET_STR_STATUS,

	/* Capture */
	E_PCM_CAPTURE_GET_NEW_PCM_AVAIL_BYTES,
	E_PCM_CAPTURE_GET_TOTAL_PCM_AVAIL_BYTES,
	E_PCM_CAPTURE_GET_DEVICE_STATUS,
};

enum MStar_SET_CMD {
	/* Playback */
	E_PCM_PLAYBACK_SET_SAMPLE_RATE = 0,
	E_PCM_PLAYBACK_SET_CHANNEL_MODE,

	/* Capture */
	E_PCM_CAPTURE_SET_CHANNEL_MODE,
	E_PCM_CAPTURE_SET_SAMPLE_RATE,
	E_PCM_CAPTURE_SET_BUFFER_SIZE,
	E_PCM_CAPTURE_SET_BIT_WIDTH,
};

enum {
	E_MONO = 1,
	E_STEREO = 2,
};

enum {
	E_STOP = 0,
	E_START,
	E_PAUSE,
	E_PAUSE_RELEASE,
	E_PREPARE,
	E_SUSPEND,
	E_RESUME,
};

struct MStar_MAD_Ops {
	int (*open)(void);
	int (*close)(void);
	int (*start)(void);
	int (*stop)(void);
	int (*resume)(void);
	int (*suspend)(void);
	unsigned int (*read)(void *buffer, unsigned int bytes);
	unsigned int (*write)(void *buffer, unsigned int bytes);
	int (*get)(int cmd, unsigned int *param);
	int (*set)(int cmd, unsigned int *param);
};

struct MStar_MAD_Info {
	char name[32];
	char version[32];
	char number;
	struct MStar_MAD_Ops *playback_pcm_ops[MAD_MAX_DEVICES];
	struct MStar_MAD_Ops *capture_pcm_ops[MAD_MAX_DEVICES];
};


/*
 * ============================================================================
 * Forward Declarations
 * ============================================================================
 */
extern int _mdrv_alsa_hook_device(struct MStar_MAD_Info *mad_info);
extern int _mdrv_alsa_unhook_device(void);

#endif /* _MDRV_ALSA_PUBLIC_HEADER */

