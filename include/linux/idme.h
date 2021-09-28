/*
 * Copyright (C) 2016 Amazom.com, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#ifndef _LINUX_IDME_H
#define _LINUX_IDME_H

#include <linux/types.h>

#define FOS_FLAGS_NONE                  (0x00000000ull)
#define FOS_FLAGS_ADB_ON                (0x00000001ull)
#define FOS_FLAGS_ADB_ROOT              (0x00000002ull)
#define FOS_FLAGS_CONSOLE_ON            (0x00000004ull)
#define FOS_FLAGS_RAMDUMP_ON            (0x00000008ull)
#define FOS_FLAGS_VERBOSITY_ON          (0x00000010ull)
#define FOS_FLAGS_ADB_AUTH_DISABLE      (0x00000020ull)
#define FOS_FLAGS_MAX                   (0x00000020ull)

extern u64 idme_get_fos_flags_value(void);

#endif
