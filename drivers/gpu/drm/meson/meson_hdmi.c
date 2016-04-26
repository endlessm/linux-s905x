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
#include <drm/drmP.h>
#include <drm/drm_edid.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic_helper.h>

#include "meson_modes.h"
#include "meson_priv.h"

#include <linux/amlogic/hdmi_tx/hdmi_info_global.h>
#include <linux/amlogic/hdmi_tx/hdmi_tx_module.h>
#include "../../../../drivers/amlogic/hdmi/hdmi_tx_20/hw/mach_reg.h"
#include "../../../../drivers/amlogic/hdmi/hdmi_tx_20/hw/common.h"
#include "../../../../drivers/amlogic/hdmi/hdmi_tx_20/hw/hdmi_tx_reg.h"

/* Encoder */

static void meson_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static const struct drm_encoder_funcs meson_encoder_funcs = {
	.destroy        = meson_encoder_destroy,
};

static void meson_encoder_dpms(struct drm_encoder *encoder, int mode)
{
}

static bool meson_encoder_mode_fixup(struct drm_encoder *encoder,
				     const struct drm_display_mode *mode,
				     struct drm_display_mode *adjusted_mode)
{
	vmode_t vmode;
	vmode = drm_mode_to_vmode(adjusted_mode, MESON_MODES_HDMI);
	return (vmode != VMODE_MAX);
}

static void meson_encoder_prepare(struct drm_encoder *encoder)
{
}

static void meson_encoder_commit(struct drm_encoder *encoder)
{
}

static void meson_encoder_mode_set(struct drm_encoder *encoder,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode)
{
	vmode_t vmode;
	vmode = drm_mode_to_vmode(adjusted_mode, MESON_MODES_HDMI);
	meson_drm_set_vmode(vmode);

	/* Make sure to unblank our display */
	hd_write_reg(P_VPU_HDMI_DATA_OVR, 0);
}

static const struct drm_encoder_helper_funcs meson_encoder_helper_funcs = {
	.dpms           = meson_encoder_dpms,
	.mode_fixup     = meson_encoder_mode_fixup,
	.prepare        = meson_encoder_prepare,
	.commit         = meson_encoder_commit,
	.mode_set       = meson_encoder_mode_set,
};

static struct drm_encoder *meson_encoder_create(struct drm_device *dev)
{
	struct drm_encoder *encoder;
	int ret;

	encoder = kzalloc(sizeof(*encoder), GFP_KERNEL);
	if (!encoder)
		return NULL;

	encoder->possible_crtcs = 1;
	ret = drm_encoder_init(dev, encoder, &meson_encoder_funcs, DRM_MODE_ENCODER_TMDS);
	if (ret < 0)
		goto fail;

	drm_encoder_helper_add(encoder, &meson_encoder_helper_funcs);
	return encoder;

fail:
	meson_encoder_destroy(encoder);
	return NULL;
}

/* Connector */

struct meson_connector {
	struct drm_connector base;
	struct drm_encoder *encoder;
	struct delayed_work hotplug_work;
	bool enabled;
};
#define to_meson_connector(x) container_of(x, struct meson_connector, base)

static void meson_connector_destroy(struct drm_connector *connector)
{
	struct meson_connector *meson_connector = to_meson_connector(connector);
	drm_connector_cleanup(connector);
	kfree(meson_connector);
}

static enum drm_connector_status meson_connector_detect(struct drm_connector *connector, bool force)
{
	struct meson_connector *meson_connector = to_meson_connector(connector);
	if (!meson_connector->enabled)
		return connector_status_disconnected;

	return read_hpd_gpio() ? connector_status_connected : connector_status_disconnected;
}

static int meson_connector_get_modes(struct drm_connector *connector)
{
	struct hdmitx_dev *hdev = get_hdmitx_dev();
	struct edid *edid;

	hdmitx_get_edid(hdev);
	set_disp_mode_auto();

	if (!drm_edid_block_valid(hdev->EDID_buf, 0, true))
		return false;

	edid = (struct edid *) hdev->EDID_buf;

	drm_mode_connector_update_edid_property(connector, edid);

	return drm_add_edid_modes(connector, edid);
}

static int meson_connector_mode_valid(struct drm_connector *connector, struct drm_display_mode *mode)
{
	return (drm_mode_to_vmode(mode, MESON_MODES_HDMI) < VMODE_MAX) ? MODE_OK : MODE_BAD;
}

static struct drm_encoder *meson_connector_best_encoder(struct drm_connector *connector)
{
	struct meson_connector *meson_connector = to_meson_connector(connector);
	return meson_connector->encoder;
}

static const struct drm_connector_funcs meson_connector_funcs = {
	.destroy		= meson_connector_destroy,
	.detect			= meson_connector_detect,
	.dpms			= drm_helper_connector_dpms,
	.fill_modes		= drm_helper_probe_single_connector_modes,
	.reset			= drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_connector_destroy_state,
};

static const struct drm_connector_helper_funcs meson_connector_helper_funcs = {
	.get_modes          = meson_connector_get_modes,
	.mode_valid         = meson_connector_mode_valid,
	.best_encoder       = meson_connector_best_encoder,
};

static void hdmi_hotplug_work_func(struct work_struct *work)
{
	struct meson_connector *meson_connector =
		container_of(work, struct meson_connector, hotplug_work.work);
	struct drm_connector *connector = &meson_connector->base;
	struct drm_device *dev = connector->dev;
	static bool post_uboot = true;
	hdmitx_dev_t *hdev = get_hdmitx_device();


	/* Clear interrupt status flags. We don't actually care what
	 * the INTR was about. */
//	hdmi_wr_reg(OTHER_BASE_ADDR + HDMI_OTHER_INTR_STAT_CLR, 0xF);

	if (hdev->hdmitx_event & (HDMI_TX_HPD_PLUGIN)) {
		/*
		 * FIXME: everything works perfectly fine when a HPD_PLUGOUT event
		 * is detected before a HPD_PLUGIN event. This is of course always
		 * true _except_ when we boot with an HDMI cable already attached.
		 * In this case we only detect a HPD_PLUGIN event at boot. Unfortunately
		 * when U-Boot uses the HDMI to display the S905 logo it
		 * leaves the hardware in an unconsistent state. To recover from this
		 * situation we force a fake HPD_PLUGOUT event before issuing the
		 * HPD_PLUGIN to reset the hardware, this is done only once the first
		 * time we see the HPD_PLUGIN eventk.
		 */
		if (post_uboot)
			hdmitx_hpd_plugout(hdev);

		hdmitx_hpd_plugin(hdev);
	} else if (hdev->hdmitx_event & (HDMI_TX_HPD_PLUGOUT))
		hdmitx_hpd_plugout(hdev);

	post_uboot = false;

	/* This interrupt means one of three things: HPD rose, HPD fell,
	 * or EDID has changed. For all three, emit a hotplug event. */
	drm_kms_helper_hotplug_event(dev);
}

static irqreturn_t meson_hdmi_intr_handler(int irq, void *user_data)
{
	struct drm_connector *connector = user_data;
	struct meson_connector *meson_connector = to_meson_connector(connector);
	hdmitx_dev_t *hdev = get_hdmitx_device();
	unsigned int data32 = 0;

	data32 = hdmitx_rd_reg(HDMITX_TOP_INTR_STAT);

	if (hdev->hpd_lock == 1) {
		hdmitx_wr_reg(HDMITX_TOP_INTR_STAT_CLR, 0xf);
		hdmi_print(IMP, HPD "HDMI hpd locked\n");
		goto next;
	}
	/* check HPD status */
	if ((data32 & (1 << 1)) && (data32 & (1 << 2))) {
		if (hdmitx_hpd_hw_op_gxbb(HPD_READ_HPD_GPIO))
			data32 &= ~(1 << 2);
		else
			data32 &= ~(1 << 1);
	}

	if (data32 & (1 << 1)) {
		hdev->hdmitx_event |= HDMI_TX_HPD_PLUGIN;
		hdev->hdmitx_event &= ~HDMI_TX_HPD_PLUGOUT;
	}

	if (data32 & (1 << 2)) {
		hdev->hdmitx_event |= HDMI_TX_HPD_PLUGOUT;
		hdev->hdmitx_event &= ~HDMI_TX_HPD_PLUGIN;
	}

next:
	hdmitx_wr_reg(HDMITX_TOP_INTR_STAT_CLR, data32 | 0x6);

	mod_delayed_work(system_wq, &meson_connector->hotplug_work,
			 msecs_to_jiffies(200));

	return IRQ_HANDLED;
}

struct drm_connector *meson_hdmi_connector_create(struct drm_device *dev,
						  bool enabled)
{
	struct meson_connector *meson_connector;
	struct drm_connector *connector;
        struct drm_encoder *encoder;
	int ret, irq_hpd;

	/* Clear the VIC field of the AVI InfoFrame, which the boot loader
	 * might have configured. This has been seen to cause EDID read
	 * failures and "No signal" reported by the output.
	 *
	 * I'm not sure if the presence of a value here has some hardware-level
	 * effect, or just changes the behaviour of the hdmitx code, but I
	 * suspect the latter.
	 */
//	hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR + 0x04, 0);

	encoder = meson_encoder_create(dev);
	if (!encoder)
		return NULL;

	meson_connector = kzalloc(sizeof(*meson_connector), GFP_KERNEL);
	if (!meson_connector)
		return NULL;

	connector = &meson_connector->base;
	meson_connector->encoder = encoder;
	meson_connector->enabled = enabled;

	drm_connector_init(dev, connector, &meson_connector_funcs, DRM_MODE_CONNECTOR_HDMIA);
	drm_connector_helper_add(connector, &meson_connector_helper_funcs);

	connector->interlace_allowed = 0;
	connector->doublescan_allowed = 0;

	ret = drm_mode_connector_attach_encoder(connector, encoder);
	if (ret)
		goto fail;

	INIT_DELAYED_WORK(&meson_connector->hotplug_work, hdmi_hotplug_work_func);

	irq_hpd = platform_get_irq_byname(dev->platformdev, "hdmitx_hpd");
	if (irq_hpd == -ENXIO) {
		pr_err("%s: ERROR: hdmitx hpd irq No not found\n" ,
			__func__);
		return NULL;
	}

	ret = devm_request_threaded_irq(dev->dev, irq_hpd, NULL, meson_hdmi_intr_handler,
					IRQF_ONESHOT, dev_name(dev->dev), connector);
	if (ret < 0)
		goto fail;

	drm_connector_register(connector);
	return connector;

fail:
	meson_connector_destroy(connector);
	return NULL;
}
