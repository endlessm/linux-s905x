/*
 * Copyright (C) 2014 Endless Mobile
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 *
 * Written by:
 *     Jasper St. Pierre <jstpierre@mecheye.net>
 */

#ifndef __MESON_PRIV_H__
#define __MESON_PRIV_H__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <drm/drmP.h>
#include <linux/amlogic/vout/vout_notify.h>

struct meson_drm_session_data {
	struct mutex mutex;
	int cache_operations_ongoing;
	int has_pending_level1_cache_flush;
};

void meson_drm_set_vmode(vmode_t mode);

int meson_ioctl_msync(struct drm_device *dev, void *data, struct drm_file *file);
int meson_ioctl_set_domain(struct drm_device *dev, void *data, struct drm_file *file);
int meson_ioctl_cache_operations_control(struct drm_device *dev, void *data, struct drm_file *file);

#endif
