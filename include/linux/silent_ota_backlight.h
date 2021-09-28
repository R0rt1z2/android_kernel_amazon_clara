/*
 * silent_ota_backlight.h
 *
 *
 * Copyright (C) Amazon Technologies Inc. All rights reserved.
 * TODO: Add additional contributor's names.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SILENT_OTA_BACKLIGHT_H
#define __SILENT_OTA_BACKLIGHT_H

unsigned int toggle_backlight(unsigned int);

enum OTA_BACKLIGHT_CTRL {

	OTA_TOGGLE_BACKLIGHT = 1,
	OTA_TURN_ON_BACKLIGHT,
	OTA_TURN_OFF_BACKLIGHT
};
#endif
