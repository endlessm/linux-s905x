/*
 * drivers/amlogic/media/osd/osd_logo.c
 *
 * Copyright (C) 2017 Amlogic, Inc. All rights reserved.
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


/* Linux Headers */
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/delay.h>

/* Amlogic Headers */
#include <linux/amlogic/media/vout/vout_notify.h>
#include <linux/amlogic/media/vout/hdmi_tx/hdmi_tx_module.h>


/* Local Headers */
#include "osd_hw.h"
#include "osd_log.h"


#undef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#define LOGO_DEV_OSD0 0x0
#define LOGO_DEV_OSD1 0x1
#define LOGO_DEBUG    0x1001
#define LOGO_LOADED   0x1002

static DEFINE_MUTEX(logo_lock);

struct para_pair_s {
	char *name;
	int value;
};


static struct para_pair_s logo_args[] = {
	{"osd0", LOGO_DEV_OSD0},
	{"osd1", LOGO_DEV_OSD1},
	{"debug", LOGO_DEBUG},
	{"loaded", LOGO_LOADED},
};


struct logo_info_s {
	int index;
	u32 vmode;
	u32 debug;
	u32 loaded;
} logo_info = {
	.index = -1,
	.vmode = VMODE_MAX,
	.debug = 0,
	.loaded = 0,
};

static int get_value_by_name(char *name, struct para_pair_s *pair, u32 cnt)
{
	u32 i = 0;
	int found = -1;

	for (i = 0; i < cnt; i++, pair++) {
		if (strcmp(pair->name, name) == 0) {
			found = pair->value;
			break;
		}
	}

	return found;
}

static int logo_info_init(char *para)
{
	u32 count = 0;
	int value = -1;

	count = sizeof(logo_args) / sizeof(logo_args[0]);
	value = get_value_by_name(para, logo_args, count);
	if (value >= 0) {
		switch (value) {
		case LOGO_DEV_OSD0:
			logo_info.index = LOGO_DEV_OSD0;
			break;
		case LOGO_DEV_OSD1:
			logo_info.index = LOGO_DEV_OSD1;
			break;
		case LOGO_DEBUG:
			logo_info.debug = 1;
			break;
		case LOGO_LOADED:
			logo_info.loaded = 1;
			break;
		default:
			break;
		}
		return 0;
	}

	return 0;
}

static int str2lower(char *str)
{
	while (*str != '\0') {
		*str = tolower(*str);
		str++;
	}
	return 0;
}

static int __init logo_setup(char *str)
{
	char *ptr = str;
	char sep[2];
	char *option;
	int count = 5;
	char find = 0;

	if (str == NULL)
		return -EINVAL;

	do {
		if (!isalpha(*ptr) && !isdigit(*ptr)) {
			find = 1;
			break;
		}
	} while (*++ptr != '\0');

	if (!find)
		return -EINVAL;

	logo_info.index = -1;
	logo_info.debug = 0;
	logo_info.loaded = 0;
	logo_info.vmode = VMODE_MAX;

	sep[0] = *ptr;
	sep[1] = '\0';
	while ((count--) && (option = strsep(&str, sep))) {
		pr_info("%s\n", option);
		str2lower(option);
		logo_info_init(option);
	}
	return 0;
}


int set_osd_logo_freescaler(void)
{
	const struct vinfo_s *vinfo;
	u32 index = logo_info.index;

	if (logo_info.loaded == 0)
		return 0;
	if (osd_get_logo_index() != logo_info.index) {
		pr_info("logo changed, return!\n");
		return -1;
	}

	osd_set_free_scale_mode_hw(index, 1);
	osd_set_free_scale_enable_hw(index, 0);

	osd_set_free_scale_axis_hw(index, 0, 0, 1919, 1079);
	osd_update_disp_axis_hw(index, 0, 1919, 0, 1079, 0, 0, 0);
	vinfo = get_current_vinfo();
	if (vinfo) {
		pr_info("outputmode changed to %s, reset osd%d scaler\n",
			vinfo->name, index);
		osd_set_window_axis_hw(index, 0, 0,
			(vinfo->width - 1), (vinfo->height - 1));
	} else {
		osd_set_window_axis_hw(index, 0, 0, 1919, 1079);
	}
	osd_set_free_scale_enable_hw(index, 0x10001);
	osd_enable_hw(index, 1);
	return 0;
}


void set_logo_loaded(void)
{
	logo_info.loaded = 0;
}

__setup("logo=", logo_setup);


int logo_work_init(void)
{
	if (logo_info.loaded == 0)
		return -1;
	osd_set_logo_index(logo_info.index);
	return 0;
}
