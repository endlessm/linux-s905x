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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_flip_work.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_rect.h>
#include <drm/meson_drm.h>

#include "meson_cvbs.h"
#include "meson_hdmi.h"
#include "meson_modes.h"
#include "meson_priv.h"

#include "../../../../drivers/amlogic/display/osd/osd_io.h"
#include "../../../../drivers/amlogic/display/osd/osd_reg.h"
#include "../../../../drivers/amlogic/display/osd/osd.h"
#include "../../../../drivers/amlogic/display/vout/vpp.h"
#include <linux/amlogic/vout/vout_notify.h>
#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/amlogic/canvas/canvas.h>
#include <linux/amlogic/hdmi_tx/hdmi_info_global.h>
#include <linux/amlogic/hdmi_tx/hdmi_tx_module.h>

enum meson_connectors {
	MESON_CONNECTORS_HDMI      = 0x1,
	MESON_CONNECTORS_CVBS_NTSC = 0x2,
	MESON_CONNECTORS_CVBS_PAL  = 0x4,
};
static char enabled_connectors = 0;
module_param(enabled_connectors, byte, S_IRUGO | S_IWUSR);

#define DRIVER_NAME "meson"
#define DRIVER_DESC "Amlogic Meson DRM driver"

/* For debugging the driver, sometimes it helps to turn off fbdev
 * to make things simpler. */
#define NO_FBDEV 0

/* Canvas configuration. */

enum meson_canvas_wrap {
	MESON_CANVAS_WRAP_NONE = 0x00,
	MESON_CANVAS_WRAP_X    = 0x01,
	MESON_CANVAS_WRAP_Y    = 0x02,
};

enum meson_canvas_blkmode {
	MESON_CANVAS_BLKMODE_LINEAR = 0x00,
	MESON_CANVAS_BLKMODE_32x32  = 0x01,
	MESON_CANVAS_BLKMODE_64x64  = 0x02,
};

/* CRTC definition */

enum meson_underscan_type {
	UNDERSCAN_OFF,
	UNDERSCAN_ON,
};

struct meson_crtc {
	struct drm_crtc base;
	struct drm_pending_vblank_event *event;

	struct drm_property *prop_underscan;
	struct drm_property *prop_underscan_hborder;
	struct drm_property *prop_underscan_vborder;

	enum meson_underscan_type underscan_type;
	int underscan_hborder;
	int underscan_vborder;

	spinlock_t irq_lock;
};
#define to_meson_crtc(x) container_of(x, struct meson_crtc, base)

/* Plane */

enum osd_w0_bitflags {
	OSD_ENDIANNESS_BE = (0x00 << 15),
	OSD_ENDIANNESS_LE = (0x01 << 15),

	OSD_BLK_MODE_422 = (0x03 << 8),
	OSD_BLK_MODE_16  = (0x04 << 8),
	OSD_BLK_MODE_32  = (0x05 << 8),
	OSD_BLK_MODE_24  = (0x07 << 8),

	OSD_OUTPUT_COLOR_YUV = (0x00 << 7),
	OSD_OUTPUT_COLOR_RGB = (0x01 << 7),

	OSD_COLOR_MATRIX_32_RGBA = (0x00 << 2),
	OSD_COLOR_MATRIX_32_ARGB = (0x01 << 2),
	OSD_COLOR_MATRIX_32_ABGR = (0x02 << 2),
	OSD_COLOR_MATRIX_32_BGRA = (0x03 << 2),

	OSD_INTERLACE_ENABLED  = (0x01 << 1),
	OSD_INTERLACE_ODD      = (0x01 << 0),
	OSD_INTERLACE_EVEN     = (0x00 << 0),
};

/* Dumb metaprogramming, should replace with something better. */
#define OSD_REGISTERS				\
	M(CTRL_STAT)				\
	M(BLK0_CFG_W0)				\
	M(BLK0_CFG_W1)				\
	M(BLK0_CFG_W2)				\
	M(BLK0_CFG_W3)				\
	M(BLK0_CFG_W4)

struct osd_plane_def {
	bool uses_scaler;
	bool compensate_for_scaler;

	uint32_t canvas_index;

	uint32_t vpp_misc_postblend;

	struct {
#define M(n) uint32_t n;
OSD_REGISTERS
#undef M
	} reg;
};

struct osd_plane_registers {
#define M(n) uint32_t n;
OSD_REGISTERS
#undef M
};

enum meson_interlacing_strategy {
	/* We don't require interlacing -- scan as progressive. */
	MESON_INTERLACING_STRATEGY_NONE,

	/* We are interlacing out this plane using the OSD interlacer. */
	MESON_INTERLACING_STRATEGY_OSD,

	/* We are interlacing out this plane using the HW scaler, so
	 * scan this out as progressive. */
	MESON_INTERLACING_STRATEGY_SCALER,
};

struct meson_plane {
	struct drm_plane base;
	struct osd_plane_def *def;

	/* These are shadow registers that are updated
	 * at vblank time. The various atomic_commit
	 * functions set these and we copy them into the
	 * real set of mapped registers at runtime. */
	struct osd_plane_registers reg;

	enum meson_interlacing_strategy interlacing_strategy;

	bool fb_changed;
	bool visible;
};
#define to_meson_plane(x) container_of(x, struct meson_plane, base)

/* For CVBS mode, add a fixed underscan border that is
 * 7.5% of each display dimension */
#define CVBS_UNDERSCAN_MANGLE(x) ((x) * 15 / 200)

static void get_underscan_border(struct drm_plane_state *state,
				 int *hborder_p, int *vborder_p)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(state->crtc);
	int hborder = 0;
	int vborder = 0;

	if (meson_crtc->underscan_type == UNDERSCAN_ON) {
		hborder += meson_crtc->underscan_hborder;
		vborder += meson_crtc->underscan_vborder;
	}

	/* If we're on a CVBS mode, add in some constant underscan borders. */

	/* XXX: We're detecting CVBS through interlaced vs. not, but
	 * HDMI modes can be interlaced too! */
	if (state->crtc->mode.flags & DRM_MODE_FLAG_INTERLACE) {
		hborder += CVBS_UNDERSCAN_MANGLE(state->crtc_w);
		vborder += CVBS_UNDERSCAN_MANGLE(state->crtc_h);
	}

	*hborder_p = hborder;
	*vborder_p = vborder;
}

static bool get_scaler_rects(struct drm_crtc *crtc,
			     struct drm_rect *input,
			     struct drm_rect *output)
{
	struct drm_plane *plane = crtc->primary;
	struct drm_plane_state *state = plane->state;
	int hborder, vborder;

	input->x1 = 0;
	input->y1 = 0;
	input->x2 = state->crtc_w;
	input->y2 = state->crtc_h;

	*output = *input;

	get_underscan_border(state, &hborder, &vborder);

	output->x1 += hborder;
	output->x2 -= hborder;
	output->y1 += vborder;
	output->y2 -= vborder;

	return (!drm_rect_equals(input, output));
}

/* Scales from the range a1..a2 to b1..b2 */
static inline int scale_into(int v, int a1, int a2, int b1, int b2)
{
	return ((v - a1) * (b2 - b1) / (a2 - a1)) + b1;
}

static inline void scale_rect_into(struct drm_rect *dest,
				   struct drm_rect *input,
				   struct drm_rect *output)
{
	int offs;

	offs = scale_into(dest->x1, input->x1, input->x2, output->x1, output->x2) - dest->x1;
	dest->x1 += offs;
	dest->x2 += offs;

	offs = scale_into(dest->y1, input->y1, input->y2, output->y1, output->y2) - dest->y1;
	dest->y1 += offs;
	dest->y2 += offs;
}

static int meson_plane_atomic_check(struct drm_plane *plane,
				    struct drm_plane_state *state)
{
	struct drm_rect src = {
		.x1 = state->src_x,
		.y1 = state->src_y,
		.x2 = state->src_x + state->src_w,
		.y2 = state->src_y + state->src_h,
	};
	struct drm_rect dest = {
		.x1 = state->crtc_x,
		.y1 = state->crtc_y,
		.x2 = state->crtc_x + state->crtc_w,
		.y2 = state->crtc_y + state->crtc_h,
	};

	if (state->fb) {
		int ret;

		ret = drm_rect_calc_hscale(&src, &dest, DRM_PLANE_HELPER_NO_SCALING, DRM_PLANE_HELPER_NO_SCALING);
		if (ret < 0)
			return ret;

		ret = drm_rect_calc_vscale(&src, &dest, DRM_PLANE_HELPER_NO_SCALING, DRM_PLANE_HELPER_NO_SCALING);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/* Takes a fixed 16.16 number and converts it to integer. */
static inline int64_t fixed16_to_int(int64_t value)
{
	return value >> 16;
}

static void meson_plane_atomic_update(struct drm_plane *plane, struct drm_plane_state *old_state)
{
	struct meson_plane *meson_plane = to_meson_plane(plane);
	struct drm_plane_state *state = plane->state;
	struct meson_crtc *meson_crtc = to_meson_crtc(state->crtc);
	struct drm_rect src = {
		.x1 = (state->src_x),
		.y1 = (state->src_y),
		.x2 = (state->src_x + state->src_w),
		.y2 = (state->src_y + state->src_h),
	};
	struct drm_rect dest = {
		.x1 = state->crtc_x,
		.y1 = state->crtc_y,
		.x2 = state->crtc_x + state->crtc_w,
		.y2 = state->crtc_y + state->crtc_h,
	};
	struct drm_rect clip = {};
	bool is_scaling;
	unsigned long flags;

	if (state->fb) {
		struct drm_rect input, output;

		is_scaling = get_scaler_rects(state->crtc, &input, &output);

		if (meson_plane->def->compensate_for_scaler && is_scaling)
			scale_rect_into(&dest, &input, &output);

		if (meson_plane->def->uses_scaler)
			clip = input;
		else
			clip = output;

		if (state->crtc->mode.flags & DRM_MODE_FLAG_INTERLACE) {
			clip.y1 /= 2;
			clip.y2 /= 2;

			dest.y1 /= 2;
			dest.y2 /= 2;
		}

		meson_plane->visible = drm_rect_clip_scaled(&src, &dest, &clip,
							    DRM_PLANE_HELPER_NO_SCALING,
							    DRM_PLANE_HELPER_NO_SCALING);
	} else {
		meson_plane->visible = false;
	}

	if (meson_plane->visible) {
		/* If we're interlacing, then figure out what strategy we're
		 * going to use. */
		if (state->crtc->mode.flags & DRM_MODE_FLAG_INTERLACE) {
			/* If this plane is going to use the vertical scaler, then scan it out as
			 * progressive, as the scaler will take care of interlacing for us. */
			if (meson_plane->def->uses_scaler && is_scaling)
				meson_plane->interlacing_strategy = MESON_INTERLACING_STRATEGY_SCALER;
			else
				meson_plane->interlacing_strategy = MESON_INTERLACING_STRATEGY_OSD;
		} else {
			meson_plane->interlacing_strategy = MESON_INTERLACING_STRATEGY_NONE;
		}

		if (state->fb != old_state->fb)
			meson_plane->fb_changed = true;

		spin_lock_irqsave(&meson_crtc->irq_lock, flags);
		/* Enable OSD and BLK0. */
		meson_plane->reg.CTRL_STAT = ((1 << 21) |    /* Enable OSD */
					      (0xFF << 12) | /* Alpha is 0xFF */
					      (1 << 0)       /* Enable BLK0 */);

		/* Set up BLK0 to point to the right canvas */
		meson_plane->reg.BLK0_CFG_W0 = ((meson_plane->def->canvas_index << 16) |
						OSD_ENDIANNESS_LE | OSD_BLK_MODE_32 | OSD_OUTPUT_COLOR_RGB | OSD_COLOR_MATRIX_32_ARGB);

		if (meson_plane->interlacing_strategy == MESON_INTERLACING_STRATEGY_OSD)
			meson_plane->reg.BLK0_CFG_W0 |= OSD_INTERLACE_ENABLED;

		/* The format of these registers is (x2 << 16 | x1), where x2 is exclusive.
		 * e.g. +30x1920 would be (1949 << 16) | 30. */
		meson_plane->reg.BLK0_CFG_W1 = ((fixed16_to_int(src.x2) - 1) << 16) | fixed16_to_int(src.x1);
		meson_plane->reg.BLK0_CFG_W2 = ((fixed16_to_int(src.y2) - 1) << 16) | fixed16_to_int(src.y1);
		meson_plane->reg.BLK0_CFG_W3 = ((dest.x2 - 1) << 16) | dest.x1;
		meson_plane->reg.BLK0_CFG_W4 = ((dest.y2 - 1) << 16) | dest.y1;
		spin_unlock_irqrestore(&meson_crtc->irq_lock, flags);
	}
}

static const struct drm_plane_helper_funcs meson_plane_helper_funcs = {
	.atomic_check = meson_plane_atomic_check,
	.atomic_update = meson_plane_atomic_update,
};

static void meson_plane_destroy(struct drm_plane *plane)
{
	drm_plane_helper_disable(plane);
	drm_plane_cleanup(plane);
	kfree(plane);
}

static const struct drm_plane_funcs meson_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= meson_plane_destroy,
	.reset			= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
};

static const uint32_t supported_drm_formats[] = {
	DRM_FORMAT_ARGB8888,
};

static struct drm_plane *meson_plane_create(struct drm_device *dev,
					    enum drm_plane_type type,
					    struct osd_plane_def *osd_plane_def)
{
	struct meson_plane *meson_plane;
	struct drm_plane *plane;
	int ret;

	meson_plane = kzalloc(sizeof(*meson_plane), GFP_KERNEL);
	if (!meson_plane) {
		ret = -ENOMEM;
		goto fail;
	}

	plane = &meson_plane->base;

	meson_plane->def = osd_plane_def;

	drm_universal_plane_init(dev, plane, 0xFF,
				 &meson_plane_funcs,
				 supported_drm_formats,
				 ARRAY_SIZE(supported_drm_formats),
				 type);
	drm_plane_helper_add(plane, &meson_plane_helper_funcs);
	return plane;

fail:
	return ERR_PTR(ret);
}

/* CRTC */

static void meson_crtc_destroy(struct drm_crtc *crtc)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);
	drm_crtc_cleanup(crtc);
	kfree(meson_crtc);
}

static struct drm_prop_enum_list underscan_enum_list[] =
{	{ UNDERSCAN_OFF, "off" },
	{ UNDERSCAN_ON, "on" },
};

static int meson_crtc_set_property(struct drm_crtc *crtc,
				   struct drm_property *property,
				   uint64_t val)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);

	if (property == meson_crtc->prop_underscan) {
		meson_crtc->underscan_type = val;
	} else if (property == meson_crtc->prop_underscan_hborder) {
		meson_crtc->underscan_hborder = val;
	} else if (property == meson_crtc->prop_underscan_vborder) {
		meson_crtc->underscan_vborder = val;
	}

	return 0;
}

static const struct drm_crtc_funcs meson_crtc_funcs = {
	.set_config             = drm_atomic_helper_set_config,
	.destroy		= meson_crtc_destroy,
	.reset			= drm_atomic_helper_crtc_reset,
	.page_flip		= drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
	.set_property           = meson_crtc_set_property,
};

static void meson_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	hdmitx_dev_t *hdmitx_device = get_hdmitx_device();

	if (mode == DRM_MODE_DPMS_OFF)
		hdmitx_device->HWOp.CntlMisc(hdmitx_device, MISC_TMDS_PHY_OP, TMDS_PHY_DISABLE);
	else
		hdmitx_device->HWOp.CntlMisc(hdmitx_device, MISC_TMDS_PHY_OP, TMDS_PHY_ENABLE);
}

static void meson_crtc_prepare(struct drm_crtc *crtc)
{
	meson_crtc_dpms(crtc, DRM_MODE_DPMS_ON);
}

static void meson_crtc_disable(struct drm_crtc *crtc)
{
	meson_crtc_dpms(crtc, DRM_MODE_DPMS_OFF);
}

static void meson_crtc_commit(struct drm_crtc *crtc)
{
}

static bool meson_crtc_mode_fixup(struct drm_crtc *crtc,
				  const struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode)
{
	/* nothing needed */
	return true;
}

void meson_drm_set_vmode(vmode_t mode)
{
	/* Call into aml's vout driver. */

	/* AML's vout or HDMI driver really does not like when
	 * you change the mode to the same thing it already is
	 * for currently unknown reasons.
	 *
	 * Double-check that it hasn't changed before calling
	 * set_current_vmode and notifying the HDMI stack. */

	/* XXX: Replace aml's vout driver with something sensible. */

	if (mode == get_current_vmode())
		return;

	set_current_vmode(mode);
	vout_notifier_call_chain(VOUT_EVENT_MODE_CHANGE, &mode);
}

static void meson_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
}

static void meson_crtc_load_lut(struct drm_crtc *crtc)
{
}

static int meson_crtc_atomic_check(struct drm_crtc *crtc,
				   struct drm_crtc_state *state)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);

	/* If we're already page flipping and we get a new
	 * page flip, then reject. */
	if (meson_crtc->event != NULL && state->event != NULL)
		return -EINVAL;

	return 0;
}

static void meson_crtc_atomic_flush(struct drm_crtc *crtc)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);

	if (crtc->state->event != NULL) {
		/* Make sure we only have one async page flip */
		WARN_ON(meson_crtc->event != NULL);

		meson_crtc->event = crtc->state->event;
	}
}

static const struct drm_crtc_helper_funcs meson_crtc_helper_funcs = {
	.dpms           = meson_crtc_dpms,
	.prepare        = meson_crtc_prepare,
	.disable        = meson_crtc_disable,
	.commit         = meson_crtc_commit,
	.mode_fixup     = meson_crtc_mode_fixup,
	.mode_set       = drm_helper_crtc_mode_set,
	.mode_set_base  = drm_helper_crtc_mode_set_base,
	.mode_set_nofb  = meson_crtc_mode_set_nofb,
	.load_lut       = meson_crtc_load_lut,
	.atomic_check   = meson_crtc_atomic_check,
	.atomic_flush   = meson_crtc_atomic_flush,
};

/* Pick two canvases in the "user canvas" space that aren't
 * likely to compete. */
static struct osd_plane_def osd_plane_defs[] = {
	{
		.uses_scaler = true,
		.compensate_for_scaler = false,
		.canvas_index = 0x4e,
		.vpp_misc_postblend = VPP_OSD1_POSTBLEND,
		{
#define M(n) .n = VIU_OSD1_##n ,
			OSD_REGISTERS
#undef M
		}
	},
	{
		.uses_scaler = false,
		.compensate_for_scaler = true,
		.canvas_index = 0x4f,
		.vpp_misc_postblend = VPP_OSD2_POSTBLEND,
		{
#define M(n) .n = VIU_OSD2_##n ,
			OSD_REGISTERS
#undef M
		}
	},
};

struct drm_crtc *meson_crtc_create(struct drm_device *dev)
{
	struct meson_crtc *meson_crtc;
	struct drm_crtc *crtc;
	int ret;
	struct drm_plane *primary_plane, *cursor_plane;

	meson_crtc = kzalloc(sizeof(*meson_crtc), GFP_KERNEL);
	if (!meson_crtc)
		return NULL;

	spin_lock_init(&meson_crtc->irq_lock);
	primary_plane = meson_plane_create(dev,
					   DRM_PLANE_TYPE_PRIMARY,
					   &osd_plane_defs[0]);
	cursor_plane = meson_plane_create(dev,
					  DRM_PLANE_TYPE_CURSOR,
					  &osd_plane_defs[1]);

	crtc = &meson_crtc->base;
	ret = drm_crtc_init_with_planes(dev, crtc,
					primary_plane, cursor_plane,
					&meson_crtc_funcs);
	if (ret < 0)
		goto fail;

	drm_crtc_helper_add(crtc, &meson_crtc_helper_funcs);

	meson_crtc->prop_underscan = drm_property_create_enum(dev, 0, "underscan", underscan_enum_list, ARRAY_SIZE(underscan_enum_list));
	meson_crtc->prop_underscan_hborder = drm_property_create_range(dev, 0, "underscan hborder", 0, 128);
	meson_crtc->prop_underscan_vborder = drm_property_create_range(dev, 0, "underscan vborder", 0, 128);

	drm_object_attach_property(&meson_crtc->base.base, meson_crtc->prop_underscan, UNDERSCAN_OFF);
	drm_object_attach_property(&meson_crtc->base.base, meson_crtc->prop_underscan_hborder, 0);
	drm_object_attach_property(&meson_crtc->base.base, meson_crtc->prop_underscan_vborder, 0);

	meson_crtc->underscan_type = UNDERSCAN_OFF;
	meson_crtc->underscan_hborder = 0;
	meson_crtc->underscan_vborder = 0;

	return crtc;

fail:
	meson_crtc_destroy(crtc);
	return NULL;
}

/* DRM Driver */

struct meson_drm_private {
	struct drm_crtc *crtc;
	struct drm_fbdev_cma *fbdev;

	struct drm_atomic_state *cleanup_state;
	struct workqueue_struct *unref_wq;
	struct drm_flip_work unref_work;
};

static void meson_fb_output_poll_changed(struct drm_device *dev)
{
#if !NO_FBDEV
	struct meson_drm_private *priv = dev->dev_private;
	drm_fbdev_cma_hotplug_event(priv->fbdev);
#endif
}

static void cleanup_atomic_state(struct drm_device *dev, struct drm_atomic_state *state)
{
	drm_atomic_helper_cleanup_planes(dev, state);
	drm_atomic_state_free(state);
}

static int meson_atomic_commit(struct drm_device *dev,
			       struct drm_atomic_state *state,
			       bool async)
{
	struct meson_drm_private *priv = dev->dev_private;
	int ret;

	ret = drm_atomic_helper_prepare_planes(dev, state);
	if (ret < 0)
		return ret;

	/*
	 * This is the point of no return - everything below never fails except
	 * when the hw goes bonghits. Which means we can commit the new state on
	 * the software side now.
	 */
	drm_atomic_helper_swap_state(dev, state);

	drm_atomic_helper_commit_pre_planes(dev, state);
	drm_atomic_helper_commit_planes(dev, state);
	drm_atomic_helper_commit_post_planes(dev, state);

	if (async) {
		priv->cleanup_state = state;
	} else {
		drm_atomic_helper_wait_for_vblanks(dev, state);
		cleanup_atomic_state(dev, state);
	}

	return 0;
}

static const struct drm_mode_config_funcs mode_config_funcs = {
	.fb_create           = drm_fb_cma_create,
	.output_poll_changed = meson_fb_output_poll_changed,
	.atomic_check        = drm_atomic_helper_check,
	.atomic_commit       = meson_atomic_commit,
};

static void meson_unref_worker(struct drm_flip_work *work, void *val)
{
	struct drm_atomic_state *state = val;
	struct drm_device *dev = state->dev;

	cleanup_atomic_state(dev, state);
}

static void write_scaling_filter_coefs(const unsigned int *coefs,
				       bool is_horizontal)
{
	int i;

	osd_reg_write(VPP_OSD_SCALE_COEF_IDX, (is_horizontal ? 1 : 0) << 8);
	for (i = 0; i < 33; i++)
		osd_reg_write(VPP_OSD_SCALE_COEF, coefs[i]);
}

static unsigned int vpp_filter_coefs_4point_bspline[] = {
    0x15561500, 0x14561600, 0x13561700, 0x12561800,
    0x11551a00, 0x11541b00, 0x10541c00, 0x0f541d00,
    0x0f531e00, 0x0e531f00, 0x0d522100, 0x0c522200,
    0x0b522300, 0x0b512400, 0x0a502600, 0x0a4f2700,
    0x094e2900, 0x084e2a00, 0x084d2b00, 0x074c2c01,
    0x074b2d01, 0x064a2f01, 0x06493001, 0x05483201,
    0x05473301, 0x05463401, 0x04453601, 0x04433702,
    0x04423802, 0x03413a02, 0x03403b02, 0x033f3c02,
    0x033d3d03
};

/* Configure the VPP to act like how we expect it to. Other drivers,
 * like the ones included in U-Boot, might turn on weird features
 * like the HW scaler or special planes. Reset the VPP to a sane mode
 * that expects like we behave.
 */
static void reset_vpp(void)
{
	/* Turn off the HW scalers -- U-Boot turns these on and we
	 * need to clear them to make things work. */
	osd_reg_clr_mask(VPP_OSD_SC_CTRL0, 1 << 3);
	osd_reg_clr_mask(VPP_OSD_VSC_CTRL0, 1 << 24);
	osd_reg_clr_mask(VPP_OSD_HSC_CTRL0, 1 << 22);

	BUILD_BUG_ON(ARRAY_SIZE(vpp_filter_coefs_4point_bspline) != 33);
	/* Write in the proper filter coefficients. */
	write_scaling_filter_coefs(vpp_filter_coefs_4point_bspline, 0);
	write_scaling_filter_coefs(vpp_filter_coefs_4point_bspline, 1);

	/* Force all planes off -- U-Boot might configure them and
	 * we shouldn't have any stale planes. */
	osd_reg_clr_mask(VPP_MISC, VPP_OSD1_POSTBLEND | VPP_OSD2_POSTBLEND);
	osd_reg_clr_mask(VPP_MISC, VPP_VD1_POSTBLEND | VPP_VD2_POSTBLEND);

	/* Turn on POSTBLEND. */
	osd_reg_set_mask(VPP_MISC, VPP_POSTBLEND_EN);

	/* Put OSD2 (cursor) on top of OSD1. */
	osd_reg_set_mask(VPP_MISC, VPP_POST_FG_OSD2 | VPP_PRE_FG_OSD2);

	/* In its default configuration, the display controller can be starved
	 * of memory bandwidth when the CPU and GPU are busy, causing scanout
	 * to sometimes get behind where it should be (with parts of the
	 * display appearing momentarily shifted to the right).
	 * Increase the priority and burst size of RAM access using the same
	 * values as Amlogic's driver. */
	osd_reg_set_mask(VIU_OSD1_FIFO_CTRL_STAT,
			   1 << 0 | /* Urgent DDR request priority */
			   3 << 10 /* Increase burst length from 24 to 64 */
			   );
	osd_reg_set_mask(VIU_OSD2_FIFO_CTRL_STAT,
			   1 << 0 | /* Urgent DDR request priority */
			   3 << 10 /* Increase burst length from 24 to 64 */
			   );

	/* Increase the number of lines that the display controller waits
	 * after vsync before starting RAM access. This gives the vsync
	 * interrupt handler more time to update the registers, avoiding
	 * visual glitches. */
	osd_reg_set_bits(VIU_OSD1_FIFO_CTRL_STAT, 12, 5, 5);
	osd_reg_set_bits(VIU_OSD2_FIFO_CTRL_STAT, 12, 5, 5);
}

static ssize_t meson_get_underscan_hborder(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);
	struct meson_drm_private *priv = drm_dev->dev_private;
	struct meson_crtc *meson_crtc = to_meson_crtc(priv->crtc);

	return snprintf(buf, PAGE_SIZE, "%d\n", meson_crtc->underscan_hborder);
}

static ssize_t meson_set_underscan_hborder(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t size)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);
	struct meson_drm_private *priv = drm_dev->dev_private;
	struct meson_crtc *meson_crtc = to_meson_crtc(priv->crtc);

	sscanf(buf, "%d", &meson_crtc->underscan_hborder);
	meson_crtc->underscan_type = UNDERSCAN_ON;
	return size;
}

static DEVICE_ATTR(underscan_hborder, S_IRUGO | S_IWUGO, meson_get_underscan_hborder, meson_set_underscan_hborder);

static ssize_t meson_get_underscan_vborder(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);
	struct meson_drm_private *priv = drm_dev->dev_private;
	struct meson_crtc *meson_crtc = to_meson_crtc(priv->crtc);

	return snprintf(buf, PAGE_SIZE, "%d\n", meson_crtc->underscan_vborder);
}

static ssize_t meson_set_underscan_vborder(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t size)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);
	struct meson_drm_private *priv = drm_dev->dev_private;
	struct meson_crtc *meson_crtc = to_meson_crtc(priv->crtc);

	sscanf(buf, "%d", &meson_crtc->underscan_vborder);
	meson_crtc->underscan_type = UNDERSCAN_ON;
	return size;
}

static DEVICE_ATTR(underscan_vborder, S_IRUGO | S_IWUGO, meson_get_underscan_vborder, meson_set_underscan_vborder);

static int meson_load(struct drm_device *dev, unsigned long flags)
{
	struct platform_device *pdev = dev->platformdev;
	struct meson_drm_private *priv;
	int ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	reset_vpp();

	platform_set_drvdata(pdev, dev);

	dev->dev_private = priv;

	drm_mode_config_init(dev);
	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;
	/* Just a guess for now */
	dev->mode_config.max_width = 9999;
	dev->mode_config.max_height = 9999;
	dev->mode_config.funcs = &mode_config_funcs;

	priv->crtc = meson_crtc_create(dev);

	if (enabled_connectors == 0)
		/* Default: all enabled, CVBS mode selected via switch */
		enabled_connectors = MESON_CONNECTORS_HDMI;

	meson_hdmi_connector_create(dev, !!(enabled_connectors & MESON_CONNECTORS_HDMI));
#if 0
	meson_cvbs_init(dev);
#endif

	{
		struct drm_display_mode *mode;

		mode = drm_cvt_mode(dev, 720, 480, 60, false, true, false);
		mode->type |= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

		meson_cvbs_connector_create(dev, !!(enabled_connectors & MESON_CONNECTORS_CVBS_NTSC), mode);
	}

	{
		struct drm_display_mode *mode;

		mode = drm_cvt_mode(dev, 720, 576, 50, false, true, false);
		mode->type |= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

		meson_cvbs_connector_create(dev, !!(enabled_connectors & MESON_CONNECTORS_CVBS_PAL), mode);
	}

	ret = drm_vblank_init(dev, dev->mode_config.num_crtc);
	if (ret < 0) {
		/* XXX: Don't leak memory. */
		return ret;
	}

	/* set vout mode at startup to prevent the rest of
	 * amlogic's drivers from crashing... */
	meson_drm_set_vmode(VMODE_1080P);

	ret = drm_flip_work_init(&priv->unref_work, 16,
			"unref", meson_unref_worker);

	priv->unref_wq = alloc_ordered_workqueue("meson", 0);

	drm_irq_install(dev, INT_VIU_VSYNC);

	drm_mode_config_reset(dev);

	drm_kms_helper_poll_init(dev);

#if !NO_FBDEV
	priv->fbdev = drm_fbdev_cma_init(dev, 32,
					 dev->mode_config.num_crtc,
					 dev->mode_config.num_connector);
#endif

	device_create_file(dev->dev, &dev_attr_underscan_hborder);
	device_create_file(dev->dev, &dev_attr_underscan_vborder);

	return 0;
}

static int meson_unload(struct drm_device *dev)
{
	struct meson_drm_private *priv = dev->dev_private;

	drm_kms_helper_poll_fini(dev);
	drm_mode_config_cleanup(dev);
	drm_flip_work_cleanup(&priv->unref_work);

	kfree(dev->dev_private);
	dev->dev_private = NULL;

	return 0;
}

static int meson_open(struct drm_device *dev, struct drm_file *file)
{
	struct meson_drm_session_data *session_data;

	session_data = kzalloc(sizeof(*session_data), GFP_KERNEL);
	if (!session_data)
		return -ENOMEM;
	mutex_init(&session_data->mutex);

	file->driver_priv = session_data;

	return 0;
}

static void meson_postclose(struct drm_device *dev, struct drm_file *file)
{
	struct meson_drm_session_data *session_data = file->driver_priv;

	if (session_data)
		mutex_destroy(&session_data->mutex);
	kfree(session_data);
}

static void meson_lastclose(struct drm_device *dev)
{
#if !NO_FBDEV
	struct meson_drm_private *priv = dev->dev_private;
	drm_fbdev_cma_restore_mode(priv->fbdev);
#endif
}

static int meson_enable_vblank(struct drm_device *dev, int crtc)
{
	return 0;
}

static void meson_disable_vblank(struct drm_device *dev, int crtc)
{
}

static void meson_crtc_send_vblank_event(struct drm_crtc *crtc)
{
	struct meson_crtc *meson_crtc = to_meson_crtc(crtc);

	if (meson_crtc->event) {
		spin_lock(&crtc->dev->event_lock);
		drm_send_vblank_event(crtc->dev,
				      drm_crtc_index(crtc),
				      meson_crtc->event);
		meson_crtc->event = NULL;
		spin_unlock(&crtc->dev->event_lock);
	}
}

static void update_scaler(struct drm_crtc *crtc)
{
	struct meson_plane *meson_plane = to_meson_plane(crtc->primary);
	struct drm_plane_state *state = crtc->primary->state;
	struct drm_rect input, output;

	if (!meson_plane->visible)
		return;

	if (!state)
		return;

	if (get_scaler_rects(crtc, &input, &output)) {
		bool interlace = (meson_plane->interlacing_strategy == MESON_INTERLACING_STRATEGY_SCALER);

		if (interlace) {
			output.y1 /= 2;
			output.y2 /= 2;
		}

		/* Basic scaler config */
		osd_reg_write(VPP_OSD_SC_CTRL0,
			      (1 << 3) /* Enable scaler */ |
			      (0 << 2) /* Select OSD1 */);
		osd_reg_write(VPP_OSD_SCI_WH_M1,
			      ((drm_rect_width(&input) - 1) << 16) | (drm_rect_height(&input) - 1));
		osd_reg_write(VPP_OSD_SCO_H_START_END, ((output.x1) << 16) | (output.x2));
		osd_reg_write(VPP_OSD_SCO_V_START_END, ((output.y1) << 16) | (output.y2));

		/* HSC */
		if (input.x1 != output.x1 || input.x2 != output.x2) {
			int hf_phase_step = ((drm_rect_width(&input) << 18) / drm_rect_width(&output));
			osd_reg_write(VPP_OSD_HSC_PHASE_STEP, hf_phase_step << 6);

			osd_reg_write(VPP_OSD_HSC_CTRL0,
				      (4 << 0) /* osd_hsc_bank_length */ |
				      (4 << 3) /* osd_hsc_ini_rcv_num0 */ |
				      (1 << 8) /* osd_hsc_rpt_p0_num0 */ |
				      (1 << 22) /* Enable horizontal scaler */);
		} else {
			osd_reg_write(VPP_OSD_HSC_CTRL0, 0);
		}

		/* VSC */
		if (input.y1 != output.y1 || input.y2 != output.y2) {
			int vf_phase_step = ((drm_rect_height(&input) << 20) / drm_rect_height(&output));

			osd_reg_write(VPP_OSD_VSC_INI_PHASE, interlace ? (vf_phase_step >> 5) : 0);
			osd_reg_write(VPP_OSD_VSC_PHASE_STEP, vf_phase_step << 4);

			osd_reg_write(VPP_OSD_VSC_CTRL0,
				      (4 << 0) /* osd_vsc_bank_length */ |
				      (4 << 3) /* osd_vsc_top_ini_rcv_num0 */ |
				      (1 << 8) /* osd_vsc_top_rpt_p0_num0 */ |
				      (6 << 11) /* osd_vsc_bot_ini_rcv_num0 */ |
				      (2 << 16) /* osd_vsc_bot_rpt_p0_num0 */ |
				      ((interlace ? 1 : 0) << 23) /* osd_prog_interlace */ |
				      (1 << 24) /* Enable vertical scaler */);
		} else {
			osd_reg_write(VPP_OSD_VSC_CTRL0, 0);
		}
	} else {
		osd_reg_write(VPP_OSD_SC_CTRL0,
			      (0 << 3) /* Disable scaler */);
	}
}

static void update_plane_shadow_registers(struct drm_plane *plane)
{
	struct meson_plane *meson_plane = to_meson_plane(plane);
	struct drm_plane_state *state = plane->state;

	if (meson_plane->visible) {
		if (meson_plane->fb_changed) {
			struct drm_gem_cma_object *cma_bo;

			cma_bo = drm_fb_cma_get_gem_obj(state->fb, 0);

			/* Swap out the OSD canvas with the new addr. */
			canvas_setup(meson_plane->def->canvas_index,
				     cma_bo->paddr,
				     state->fb->pitches[0],
				     state->fb->height,
				     MESON_CANVAS_WRAP_NONE,
				     MESON_CANVAS_BLKMODE_LINEAR);

			meson_plane->fb_changed = false;
		}

		osd_reg_set_mask(VPP_MISC, meson_plane->def->vpp_misc_postblend);

		/* Copy the shadow registers into the real registers. */
#define M(n) osd_reg_write(meson_plane->def->reg.n, meson_plane->reg.n);
OSD_REGISTERS
#undef M
	} else {
		osd_reg_clr_mask(VPP_MISC, meson_plane->def->vpp_misc_postblend);
	}
}

static void update_interlaced_field(struct drm_plane *plane)
{
	struct meson_plane *meson_plane = to_meson_plane(plane);

	if (meson_plane->interlacing_strategy != MESON_INTERLACING_STRATEGY_NONE) {
		int field = osd_reg_read(ENCI_INFO_READ) & (1 << 29);
		meson_plane->reg.BLK0_CFG_W0 = ((meson_plane->reg.BLK0_CFG_W0 & ~0x01) |
						(field ? OSD_INTERLACE_ODD : OSD_INTERLACE_EVEN));
	}
}

static irqreturn_t meson_irq(int irq, void *arg)
{
	struct drm_device *dev = arg;
	struct meson_drm_private *priv = dev->dev_private;
	struct meson_crtc *meson_crtc = to_meson_crtc(priv->crtc);

	drm_handle_vblank(dev, 0);

	meson_crtc_send_vblank_event(priv->crtc);

	spin_lock(&meson_crtc->irq_lock);

	update_interlaced_field(priv->crtc->primary);
	update_interlaced_field(priv->crtc->cursor);

	update_plane_shadow_registers(priv->crtc->primary);
	update_plane_shadow_registers(priv->crtc->cursor);

	update_scaler(priv->crtc);

	spin_unlock(&meson_crtc->irq_lock);

	if (priv->cleanup_state) {
		drm_flip_work_queue(&priv->unref_work, priv->cleanup_state);
		priv->cleanup_state = NULL;

		drm_flip_work_commit(&priv->unref_work, priv->unref_wq);
	}

	return IRQ_HANDLED;
}

static void meson_gem_free_object(struct drm_gem_object *obj)
{
	drm_gem_cma_free_object(obj);
}

static int meson_ioctl_create_with_ump(struct drm_device *dev, void *data,
				       struct drm_file *file)
{
	struct drm_meson_gem_create_with_ump *args = data;
	struct drm_gem_cma_object *cma_obj;
	unsigned int size;
	DEFINE_DMA_ATTRS(dma_attrs);

	/* UMP requires a page-aligned size for its buffers. */
	size = PAGE_ALIGN (args->size);

	/* All allocations currently contiguous, to be improved later. */
	dma_set_attr(DMA_ATTR_FORCE_CONTIGUOUS, &dma_attrs);

	/* We do not need kernel virtual addresses */
	dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &dma_attrs);

	if (args->flags & DRM_MESON_GEM_CREATE_WITH_UMP_FLAG_SCANOUT) {
		/* No caching for scanout buffers */
		dma_set_attr(DMA_ATTR_WRITE_COMBINE, &dma_attrs);
	} else {
		/* Other buffers are textures and caches can be enabled. */
		WARN_ON(!(args->flags & DRM_MESON_GEM_CREATE_WITH_UMP_FLAG_TEXTURE));
		dma_set_attr(DMA_ATTR_NON_CONSISTENT, &dma_attrs);
	}

	cma_obj = drm_gem_cma_create_with_handle(file, dev, size, &args->handle, &dma_attrs);
	if (IS_ERR(cma_obj))
		return PTR_ERR(cma_obj);

	return PTR_ERR_OR_ZERO(cma_obj);
}

static const struct drm_ioctl_desc meson_ioctls[] = {
	DRM_IOCTL_DEF_DRV(MESON_GEM_CREATE_WITH_UMP, meson_ioctl_create_with_ump, DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(MESON_MSYNC, meson_ioctl_msync, DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(MESON_GEM_SET_DOMAIN, meson_ioctl_set_domain, DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(MESON_CACHE_OPERATIONS_CONTROL, meson_ioctl_cache_operations_control, DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
};

#ifdef CONFIG_DEBUG_FS
static struct drm_info_list meson_debugfs_list[] = {
	{ "fb",   drm_fb_cma_debugfs_show, 0 },
};

static int meson_debugfs_init(struct drm_minor *minor)
{
	struct drm_device *dev = minor->dev;
	int ret;

	ret = drm_debugfs_create_files(meson_debugfs_list,
			ARRAY_SIZE(meson_debugfs_list),
			minor->debugfs_root, minor);

	if (ret) {
		dev_err(dev->dev, "could not install meson_debugfs_list\n");
		return ret;
	}

	return ret;
}

static void meson_debugfs_cleanup(struct drm_minor *minor)
{
	drm_debugfs_remove_files(meson_debugfs_list,
				 ARRAY_SIZE(meson_debugfs_list), minor);
}
#endif

static const struct file_operations fops = {
	.owner              = THIS_MODULE,
	.open               = drm_open,
	.release            = drm_release,
	.unlocked_ioctl     = drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl       = drm_compat_ioctl,
#endif
	.poll               = drm_poll,
	.read               = drm_read,
	.llseek             = no_llseek,
	.mmap               = drm_gem_cma_mmap,
};

static struct drm_driver meson_driver = {
	.driver_features    = DRIVER_HAVE_IRQ | DRIVER_GEM | DRIVER_MODESET | DRIVER_PRIME,
	.load               = meson_load,
	.unload             = meson_unload,
	.open               = meson_open,
	.postclose          = meson_postclose,
	.lastclose          = meson_lastclose,
	.enable_vblank      = meson_enable_vblank,
	.disable_vblank     = meson_disable_vblank,
	.get_vblank_counter = drm_vblank_count,
	.irq_handler        = meson_irq,
	.set_busid          = drm_platform_set_busid,
	.fops               = &fops,
	.name               = DRIVER_NAME,
	.desc               = DRIVER_DESC,
	.date               = "20141113",
	.major              = 1,
	.minor              = 0,
	.gem_free_object    = meson_gem_free_object,
	.gem_vm_ops         = &drm_gem_cma_vm_ops,
	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_prime_import	= drm_gem_prime_import,
	.gem_prime_export	= drm_gem_prime_export,
	.gem_prime_get_sg_table	= drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap		= drm_gem_cma_prime_vmap,
	.gem_prime_vunmap	= drm_gem_cma_prime_vunmap,
	.gem_prime_mmap		= drm_gem_cma_prime_mmap,
	.dumb_create        = drm_gem_cma_dumb_create,
	.dumb_map_offset    = drm_gem_cma_dumb_map_offset,
	.dumb_destroy       = drm_gem_dumb_destroy,
	.ioctls             = meson_ioctls,
	.num_ioctls         = DRM_MESON_NUM_IOCTLS,

#ifdef CONFIG_DEBUG_FS
	.debugfs_init       = meson_debugfs_init,
	.debugfs_cleanup    = meson_debugfs_cleanup,
#endif
};

static int meson_pdev_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;

	/* No DT configuration? Bail. */
	if (!node)
		return -ENXIO;

	return drm_platform_init(&meson_driver, pdev);
}

static int meson_pdev_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id dt_match[] = {
	{ .compatible = "amlogic,mesongxl-display" },
	{}
};
MODULE_DEVICE_TABLE(of, dt_match);

static struct platform_driver meson_drm_platform_driver = {
	.probe      = meson_pdev_probe,
	.remove     = meson_pdev_remove,
	.driver     = {
		.owner  = THIS_MODULE,
		.name   = DRIVER_NAME,
		.of_match_table = dt_match,
	},
};

module_platform_driver(meson_drm_platform_driver);

MODULE_AUTHOR("Jasper St. Pierre <jstpierre@mecheye.net>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
