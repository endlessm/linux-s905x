/*
 * drivers/amlogic/media/vout/cvbs/cvbs_out.c
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
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/ctype.h>
/*#include <linux/major.h>*/
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#ifdef CONFIG_INSTABOOT
#include <linux/syscore_ops.h>
#endif

/* Amlogic Headers */
#include <linux/amlogic/cpu_version.h>
#include <linux/amlogic/media/vout/vout_notify.h>
#include <linux/amlogic/media/vout/vinfo.h>
#include <linux/amlogic/media/vout/vdac_dev.h>
#ifdef CONFIG_AMLOGIC_VPU
#include <linux/amlogic/media/vpu/vpu.h>
#endif

/* Local Headers */
#include "cvbsregs.h"
#include "cvbs_out.h"
#include "enc_clk_config.h"
#include "cvbs_out_reg.h"
#include "cvbs_log.h"
#ifdef CONFIG_AMLOGIC_WSS
#include "wss.h"
#endif

static struct vinfo_s cvbs_info[] = {
	{ /* MODE_480CVBS*/
		.name              = "480cvbs",
		.mode              = VMODE_CVBS,
		.width             = 720,
		.height            = 480,
		.field_height      = 240,
		.aspect_ratio_num  = 4,
		.aspect_ratio_den  = 3,
		.sync_duration_num = 60,
		.sync_duration_den = 1,
		.video_clk         = 27000000,
		.viu_color_fmt     = COLOR_FMT_YUV444,
		.viu_mux           = VIU_MUX_ENCI,
	},
	{ /* MODE_576I */
		.name              = "576cvbs",
		.mode              = VMODE_CVBS,
		.width             = 720,
		.height            = 576,
		.field_height      = 288,
		.aspect_ratio_num  = 4,
		.aspect_ratio_den  = 3,
		.sync_duration_num = 50,
		.sync_duration_den = 1,
		.video_clk         = 27000000,
		.viu_color_fmt     = COLOR_FMT_YUV444,
		.viu_mux           = VIU_MUX_ENCI,
	},
};

#define PIN_MUX_REG_0 0x202c
#define P_PIN_MUX_REG_0 CBUS_REG_ADDR(PIN_MUX_REG_0)
static struct disp_module_info_s disp_module_info __nosavedata;
static struct disp_module_info_s *info __nosavedata;

static void vdac_power_level_store(char *para);
SET_CVBS_CLASS_ATTR(vdac_power_level, vdac_power_level_store);

static void bist_test_store(char *para);

static void cvbs_debug_store(char *para);
SET_CVBS_CLASS_ATTR(debug, cvbs_debug_store);

#ifdef CONFIG_AMLOGIC_WSS
struct class_attribute class_CVBS_attr_wss = __ATTR(wss, 0644,
			aml_CVBS_attr_wss_show, aml_CVBS_attr_wss_store);
#endif /*CONFIG_AMLOGIC_WSS*/

static int cvbs_vdac_power_level;

static DEFINE_MUTEX(setmode_mutex);
static DEFINE_MUTEX(CC_mutex);

static enum cvbs_mode_e local_cvbs_mode;
static void cvbs_config_vdac(unsigned int flag, unsigned int cfg);

#ifdef CONFIG_CVBS_PERFORMANCE_COMPATIBILITY_SUPPORT
static void cvbs_performance_enhancement(enum cvbs_mode_e mode);
#endif
static void cvbs_cntl_output(unsigned int open);
static void cvbs_performance_config(unsigned int index);

#if 0
static int get_vdac_power_level(void)
{
	return cvbs_vdac_power_level;
}

static int check_cpu_type(unsigned int cpu_type)
{
	int ret = 0;

	ret = (get_meson_cpu_version(MESON_CPU_VERSION_LVL_MAJOR) == cpu_type);

	return ret;
}
#endif

/* static int get_cpu_minor(void)
 * {
 *	return get_meson_cpu_version(MESON_CPU_VERSION_LVL_MINOR);
 * }
 */

static unsigned int vdac_cfg_valid = 0, vdac_cfg_value;
static unsigned int cvbs_get_trimming_version(unsigned int flag)
{
	unsigned int version = 0xff;

	if ((flag & 0xf0) == 0xa0)
		version = 5;
	else if ((flag & 0xf0) == 0x40)
		version = 2;
	else if ((flag & 0xc0) == 0x80)
		version = 1;
	else if ((flag & 0xc0) == 0x00)
		version = 0;
	return version;
}

static void cvbs_config_vdac(unsigned int flag, unsigned int cfg)
{
	unsigned char version = 0;

	vdac_cfg_value = cfg & 0x7;
	version = cvbs_get_trimming_version(flag);
	/* flag 1/0 for validity of vdac config */
	if ((version == 1) || (version == 2) || (version == 5))
		vdac_cfg_valid = 1;
	else
		vdac_cfg_valid = 0;
	cvbs_log_info("cvbs trimming.%d.v%d: 0x%x, 0x%x\n",
		      vdac_cfg_valid, version, flag, cfg);
}

static void cvbs_cntl_output(unsigned int open)
{
	unsigned int cntl0 = 0, cntl1 = 0;

	if (open == 0) { /* close */
		cntl0 = 0;
		cntl1 = 8;
		vdac_set_ctrl0_ctrl1(cntl0, cntl1);

		/* must enable adc bandgap, the adc ref signal for demod */
		vdac_enable(0, 0x8);
	} else if (open == 1) { /* open */
#if 0
		if (cpu_after_eq(MESON_CPU_MAJOR_ID_TXL))
			cntl0 = 0x620001;
		else
#endif
		if (cpu_after_eq(MESON_CPU_MAJOR_ID_GXL))
			cntl0 = 0xb0001;
		else
			cntl0 = 0x1;
		cntl1 = (vdac_cfg_valid == 0) ? 0 : vdac_cfg_value;
		cvbs_log_info("vdac open.%d = 0x%x, 0x%x\n",
			      vdac_cfg_valid, cntl0, cntl1);
		vdac_set_ctrl0_ctrl1(cntl0, cntl1);

		/*vdac ctrl for cvbsout/rf signal,adc bandgap*/
		vdac_enable(1, 0x8);
	}
}

/* 0xff for none config from uboot */
static unsigned int cvbs_performance_index = 0xff;
static void cvbs_performance_config(unsigned int index)
{
	cvbs_performance_index = index;
}

#ifdef CONFIG_CVBS_PERFORMANCE_COMPATIBILITY_SUPPORT
static void cvbs_performance_enhancement(enum cvbs_mode_e mode)
{
	unsigned int index = cvbs_performance_index;
	unsigned int max = 0;
	unsigned int type = 0;
	const struct reg_s *s = NULL;

	if (mode != MODE_576CVBS)
		return;
	if (index == 0xff)
		return;
#if 0
	if (check_cpu_type(MESON_CPU_MAJOR_ID_GXBB)) {
		max = sizeof(cvbsregs_576cvbs_performance_gxbb)
			/ sizeof(struct reg_s *);
		index = (index >= max) ? 0 : index;
		s = cvbsregs_576cvbs_performance_gxbb[index];
		type = 4;
	} else if (check_cpu_type(MESON_CPU_MAJOR_ID_GXTVBB)) {
		max = sizeof(cvbsregs_576cvbs_performance_gxtvbb)
			/ sizeof(struct reg_s *);
		index = (index >= max) ? 0 : index;
		s = cvbsregs_576cvbs_performance_gxtvbb[index];
		type = 5;
	} else if (cpu_after_eq(MESON_CPU_MAJOR_ID_TXL)) {
		max = sizeof(cvbsregs_576cvbs_performance_txl)
			/ sizeof(struct reg_s *);
		index = (index >= max) ? 0 : index;
		s = cvbsregs_576cvbs_performance_txl[index];
		type = 8;
	} else
#endif
	if (cpu_after_eq(MESON_CPU_MAJOR_ID_GXL)) {
		if (is_meson_gxl_package_905L()) {
			max = sizeof(cvbsregs_576cvbs_performance_905l)
				/ sizeof(struct reg_s *);
			index = (index >= max) ? 0 : index;
			s = cvbsregs_576cvbs_performance_905l[index];
			type = 7;
		} else {
			max = sizeof(cvbsregs_576cvbs_performance_905x)
				/ sizeof(struct reg_s *);
			index = (index >= max) ? 0 : index;
			s = cvbsregs_576cvbs_performance_905x[index];
			type = 6;
		}
	}

	if (s == NULL) {
		cvbs_log_err("can't find performance table!\n");
		return;
	}

	cvbs_log_info("cvbs performance type = %d, table = %d\n", type, index);
	while (s && MREG_END_MARKER != s->reg) {
		cvbs_out_reg_write(s->reg, s->val);
		s++;
	}
}
#endif/* end of CVBS_PERFORMANCE_COMPATIBILITY_SUPPORT */

static int cvbs_out_set_venc(enum cvbs_mode_e mode)
{
	const struct reg_s *s;
	int ret;

	/* setup encoding regs */
	s = cvbsregsTab[mode].enc_reg_setting;
	if (!s) {
		cvbs_log_info("display mode %d get enc set NULL\n", mode);
		ret = -1;
	} else {
		while (s->reg != MREG_END_MARKER) {
			cvbs_out_reg_write(s->reg, s->val);
			s++;
		}
		ret = 0;
	}
	/* cvbs_log_info("%s[%d]\n", __func__, __LINE__); */
	return ret;
}

static void cvbs_out_enable_cvbs_gate(void)
{
	/*do it by vpu ctrl*/
#if 0
	cvbs_out_hiu_set_mask(HHI_GCLK_OTHER,
		(1<<DAC_CLK) | (1<<GCLK_VENCI_INT) |
		(1<<VCLK2_VENCI1) | (1<<VCLK2_VENCI));
#endif
	cvbs_out_hiu_set_mask(HHI_VID_CLK_CNTL2,
		(1<<ENCI_GATE_VCLK) | (1<<VDAC_GATE_VCLK));
}

static void cvbs_out_disable_clk(void)
{
	disable_vmode_clk();
}

static void cvbs_out_disable_venc(void)
{
	if (get_cpu_type() >= MESON_CPU_MAJOR_ID_GXBB)
		cvbs_cntl_output(0);

	cvbs_out_reg_write(ENCI_VIDEO_EN, 0);
}

static char *cvbs_out_bist_str[] = {
	"Fix Value",   /* 0 */
	"Color Bar",   /* 1 */
	"Thin Line",   /* 2 */
	"Dot Grid",    /* 3 */
};

int cvbs_out_setmode(void)
{
	int ret;

	if (local_cvbs_mode >= MODE_MAX)
		cvbs_log_err("cvbs_out_setmode:cvbsmode error.");
	mutex_lock(&setmode_mutex);
	cvbs_log_info("SET cvbs mode:%s(%d)\n",
		(local_cvbs_mode == 0) ? "480cvbs" :
		((local_cvbs_mode == 1) ? "576cvbs" :
		"invalid"), local_cvbs_mode);
	cvbs_cntl_output(0);
	cvbs_out_reg_write(VENC_VDAC_SETTING, 0xff);
	/* Before setting clk for CVBS, disable ENCP/I to avoid hungup */
	cvbs_out_reg_write(ENCP_VIDEO_EN, 0);
	cvbs_out_reg_write(ENCI_VIDEO_EN, 0);
	set_vmode_clk();
	ret = cvbs_out_set_venc(local_cvbs_mode);
	if (ret) {
		mutex_lock(&setmode_mutex);
		return -1;
	}
#ifdef CONFIG_CVBS_PERFORMANCE_COMPATIBILITY_SUPPORT
	cvbs_performance_enhancement(local_cvbs_mode);
#endif
	cvbs_out_enable_cvbs_gate();
	msleep(1000);
	cvbs_cntl_output(1);

	mutex_unlock(&setmode_mutex);
	return 0;
}

static int cvbs_open(struct inode *inode, struct file *file)
{
	struct disp_module_info_s *dinfo;
	/* Get the per-device structure that contains this cdev */
	dinfo = container_of(&inode->i_cdev, struct disp_module_info_s, cdev);
	file->private_data = dinfo;
	return 0;
}

static int cvbs_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long cvbs_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	unsigned int CC_2byte_data = 0;
	void __user *argp = (void __user *)arg;

	cvbs_log_info("[cvbs..] %s: cmd_nr = 0x%x\n",
			__func__, _IOC_NR(cmd));
	if (_IOC_TYPE(cmd) != _TM_V) {
		cvbs_log_err("%s invalid command: %u\n", __func__, cmd);
		return -ENOTTY;
	}
	switch (cmd) {
	case VOUT_IOC_CC_OPEN:
		cvbs_out_reg_setb(ENCI_VBI_SETTING, 0x3, 0, 2);
		break;
	case VOUT_IOC_CC_CLOSE:
		cvbs_out_reg_setb(ENCI_VBI_SETTING, 0x0, 0, 2);
		break;
	case VOUT_IOC_CC_DATA: {
		struct vout_CCparm_s parm = {0};

		mutex_lock(&CC_mutex);
		if (copy_from_user(&parm, argp,
				sizeof(struct vout_CCparm_s))) {
			cvbs_log_err("VOUT_IOC_CC_DATAinvalid parameter\n");
			ret = -EFAULT;
			mutex_unlock(&CC_mutex);
			break;
		}
		/*cc standerd:nondisplay control byte + display control byte*/
		/*our chip high-low 16bits is opposite*/
		CC_2byte_data = parm.data2 << 8 | parm.data1;
		if (parm.type == 0)
			cvbs_out_reg_write(ENCI_VBI_CCDT_EVN, CC_2byte_data);
		else if (parm.type == 1)
			cvbs_out_reg_write(ENCI_VBI_CCDT_ODD, CC_2byte_data);
		else
			cvbs_log_err("CC type:%d,Unknown.\n", parm.type);
		cvbs_log_info("VOUT_IOC_CC_DATA..type:%d,0x%x\n",
					parm.type, CC_2byte_data);
		mutex_unlock(&CC_mutex);
		break;
	}
	default:
		ret = -ENOIOCTLCMD;
		cvbs_log_err("%s %d is not supported command\n",
				__func__, cmd);
		break;
	}
	cvbs_log_info("cvbs_ioctl..out.ret=0x%lx\n", ret);
	return ret;
}

#ifdef CONFIG_COMPAT
static long cvbs_compat_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	unsigned long ret;

	arg = (unsigned long)compat_ptr(arg);
	ret = cvbs_ioctl(file, cmd, arg);
	return ret;
}
#endif

static const struct file_operations am_cvbs_fops = {
	.open	= cvbs_open,
	.read	= NULL,/* am_cvbs_read, */
	.write	= NULL,
	.unlocked_ioctl	= cvbs_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = cvbs_compat_ioctl,
#endif
	.release	= cvbs_release,
	.poll		= NULL,
};

static const struct vinfo_s *get_valid_vinfo(char  *mode)
{
	struct vinfo_s *vinfo = NULL;
	int  i, count = ARRAY_SIZE(cvbs_info);
	int mode_name_len = 0;

	/*cvbs_log_info("get_valid_vinfo..out.mode:%s\n", mode);*/
	for (i = 0; i < count; i++) {
		if (strncmp(cvbs_info[i].name, mode,
			    strlen(cvbs_info[i].name)) == 0) {
			if ((vinfo == NULL)
			    || (strlen(cvbs_info[i].name) > mode_name_len)) {
				vinfo = &cvbs_info[i];
				mode_name_len = strlen(cvbs_info[i].name);
				local_cvbs_mode = i;
				break;
			}
		}
	}
	if (vinfo)
		strcpy(vinfo->ext_name, mode);
	else
		local_cvbs_mode = MODE_MAX;
	return vinfo;
}

static struct vinfo_s *cvbs_get_current_info(void)
{
	return info->vinfo;
}

enum cvbs_mode_e get_local_cvbs_mode(void)
{
	return local_cvbs_mode;
}
EXPORT_SYMBOL(get_local_cvbs_mode);

#ifdef CONFIG_AMLOGIC_VPU
static void cvbs_out_vpu_power_ctrl(int status)
{
	if (info->vinfo == NULL)
		return;
	if (status) {
		request_vpu_clk_vmod(info->vinfo->video_clk, VPU_VENCI);
		switch_vpu_mem_pd_vmod(VPU_VENCI, VPU_MEM_POWER_ON);
	} else {
		switch_vpu_mem_pd_vmod(VPU_VENCI, VPU_MEM_POWER_DOWN);
		release_vpu_clk_vmod(VPU_VENCI);
	}
}

static void cvbs_out_vpu_gate_ctrl(int status)
{
	if (info->vinfo == NULL)
		return;
	if (status)
		switch_vpu_clk_gate_vmod(VPU_VENCI, VPU_CLK_GATE_ON);
	else
		switch_vpu_clk_gate_vmod(VPU_VENCI, VPU_CLK_GATE_OFF);
}
#endif

static int cvbs_set_current_vmode(enum vmode_e mode)
{
	enum vmode_e tvmode;

	tvmode = mode & VMODE_MODE_BIT_MASK;
	if (tvmode != VMODE_CVBS)
		return -EINVAL;
	if (local_cvbs_mode == MODE_MAX) {
		cvbs_log_info("local_cvbs_mode err:%d!\n", local_cvbs_mode);
		return 1;
	}
	info->vinfo = &cvbs_info[local_cvbs_mode];

	cvbs_log_info("mode is %d,sync_duration_den=%d,sync_duration_num=%d\n",
		      tvmode, info->vinfo->sync_duration_den,
		      info->vinfo->sync_duration_num);

#ifdef CONFIG_AMLOGIC_VPU
	cvbs_out_vpu_power_ctrl(1);
	cvbs_out_vpu_gate_ctrl(1);
#endif
	cvbs_out_reg_write(VPP_POSTBLEND_H_SIZE, info->vinfo->width);
	if (mode & VMODE_INIT_BIT_MASK) {
		cvbs_log_info("already display in uboot\n");
		return 0;
	}
	cvbs_out_setmode();

	return 0;
}

static enum vmode_e cvbs_validate_vmode(char *mode)
{
	const struct vinfo_s *info = get_valid_vinfo(mode);

	if (info)
		return VMODE_CVBS;
	return VMODE_MAX;
}
static int cvbs_vmode_is_supported(enum vmode_e mode)
{
	mode &= VMODE_MODE_BIT_MASK;
	if (mode == VMODE_CVBS)
		return true;
	return false;
}
static int cvbs_module_disable(enum vmode_e cur_vmod)
{
#ifdef CONFIG_AMLOGIC_VPU
	cvbs_out_vpu_power_ctrl(0);
	cvbs_out_vpu_gate_ctrl(0);
#endif
	/* video_dac_disable(); */
	return 0;
}

#ifdef CONFIG_PM
static int cvbs_suspend(void)
{
	/* TODO */
	/* video_dac_disable(); */
	cvbs_cntl_output(0);
	return 0;
}

static int cvbs_resume(void)
{
	/* TODO */
	/* video_dac_enable(0xff); */
	cvbs_set_current_vmode(info->vinfo->mode);
	return 0;
}
#endif

static struct vout_server_s cvbs_server = {
	.name = "vout_cvbs_server",
	.op = {
		.get_vinfo = cvbs_get_current_info,
		.set_vmode = cvbs_set_current_vmode,
		.validate_vmode = cvbs_validate_vmode,
		.vmode_is_supported = cvbs_vmode_is_supported,
		.disable = cvbs_module_disable,
		.set_vframe_rate_hint = NULL,
		.set_vframe_rate_end_hint = NULL,
		.set_vframe_rate_policy = NULL,
		.get_vframe_rate_policy = NULL,
#ifdef CONFIG_PM
		.vout_suspend = cvbs_suspend,
		.vout_resume = cvbs_resume,
#endif
	},
};

static void _init_cvbsout(void)
{
	if (info->vinfo == NULL)
		info->vinfo = &cvbs_info[MODE_480CVBS];
}

/* **************************************************** */
static void bist_test_store(char *para)
{
	unsigned long num;
	enum vmode_e mode;
	unsigned int start, width;
	int ret;

	mode = info->vinfo->mode;
	if (mode != VMODE_CVBS) {
		pr_info("NOT VMODE_CVBS,RETURN\n");
		return;
	}
	ret = kstrtoul(para, 10, (unsigned long *)&num);
	if (num > 3) {
		switch (local_cvbs_mode) {
		case MODE_480CVBS:
		case MODE_576CVBS:
			cvbs_out_reg_write(ENCI_TST_EN, 0);
			break;
		default:
			cvbs_out_reg_setb(ENCP_VIDEO_MODE_ADV, 1, 3, 1);
			cvbs_out_reg_write(VENC_VIDEO_TST_EN, 0);
			break;
		}
		pr_info("disable bist pattern\n");
	} else {
		switch (local_cvbs_mode) {
		case MODE_480CVBS:
		case MODE_576CVBS:
			cvbs_out_reg_write(ENCI_TST_CLRBAR_STRT, 0x112);
			cvbs_out_reg_write(ENCI_TST_CLRBAR_WIDTH, 0xb4);
			cvbs_out_reg_write(ENCI_TST_MDSEL, (unsigned int)num);
			cvbs_out_reg_write(ENCI_TST_EN, 1);
			break;
		default:
			start = cvbs_out_reg_read(ENCP_VIDEO_HAVON_BEGIN);
			width = info->vinfo->width / 9;
			cvbs_out_reg_write(VENC_VIDEO_TST_CLRBAR_STRT, start);
			cvbs_out_reg_write(VENC_VIDEO_TST_CLRBAR_WIDTH, width);
			cvbs_out_reg_write(VENC_VIDEO_TST_MDSEL, 1);
			cvbs_out_reg_setb(ENCP_VIDEO_MODE_ADV, 0, 3, 1);
			cvbs_out_reg_write(VENC_VIDEO_TST_EN, 1);
			break;
		}
		pr_info("show bist pattern %ld: %s\n",
			num, cvbs_out_bist_str[num]);
	}
}

static void vdac_power_level_store(char *para)
{
	unsigned long level = 0;
	int ret = 0;

	ret = kstrtoul(para, 10, (unsigned long *)&level);
	cvbs_vdac_power_level = level;
}

static void dump_clk_registers(void)
{
	unsigned int clk_regs[] = {
		/* hiu 10c8 ~ 10cd */
		HHI_HDMI_PLL_CNTL,
		HHI_HDMI_PLL_CNTL2,
		HHI_HDMI_PLL_CNTL3,
		HHI_HDMI_PLL_CNTL4,
		HHI_HDMI_PLL_CNTL5,
		HHI_HDMI_PLL_CNTL6,

		/* hiu 1068 */
		HHI_VID_PLL_CLK_DIV,

		/* hiu 104a, 104b*/
		HHI_VIID_CLK_DIV,
		HHI_VIID_CLK_CNTL,

		/* hiu 1059, 105f */
		HHI_VID_CLK_DIV,
		HHI_VID_CLK_CNTL,
	};
	unsigned int i, max;

	max = sizeof(clk_regs)/sizeof(unsigned int);
	pr_info("\n total %d registers of clock path for hdmi pll:\n", max);
	for (i = 0; i < max; i++) {
		pr_info("hiu [0x%x] = 0x%x\n", clk_regs[i],
			cvbs_out_hiu_read(clk_regs[i]));
	}
}

static void cvbs_performance_regs_dump(void)
{
	unsigned int performance_regs_enci[] = {
			VENC_VDAC_DAC0_GAINCTRL,
			ENCI_SYNC_ADJ,
			ENCI_VIDEO_BRIGHT,
			ENCI_VIDEO_CONT,
			ENCI_VIDEO_SAT,
			ENCI_VIDEO_HUE,
			ENCI_YC_DELAY,
			VENC_VDAC_DAC0_FILT_CTRL0,
			VENC_VDAC_DAC0_FILT_CTRL1
		};
	unsigned int performance_regs_vdac[] = {
			HHI_VDAC_CNTL0,
			HHI_VDAC_CNTL1
		};
	int i, size;

	size = sizeof(performance_regs_enci)/sizeof(unsigned int);
	pr_info("------------------------\n");
	for (i = 0; i < size; i++) {
		pr_info("vcbus [0x%x] = 0x%x\n", performance_regs_enci[i],
			cvbs_out_reg_read(performance_regs_enci[i]));
	}

	size = sizeof(performance_regs_vdac)/sizeof(unsigned int);
	pr_info("------------------------\n");
	for (i = 0; i < size; i++) {
		pr_info("hiu [0x%x] = 0x%x\n", performance_regs_vdac[i],
			cvbs_out_hiu_read(performance_regs_vdac[i]));
	}
	pr_info("------------------------\n");
}

enum {
	CMD_REG_READ,
	CMD_REG_READ_BITS,
	CMD_REG_DUMP,
	CMD_REG_WRITE,
	CMD_REG_WRITE_BITS,

	CMD_CLK_DUMP,
	CMD_CLK_MSR,

	CMD_BIST,

	/* config a set of performance parameters:
	 *0 for sarft,
	 *1 for telecom,
	 *2 for chinamobile
	 */
	CMD_VP_SET,

	/* get the current perfomance config */
	CMD_VP_GET,

	CMD_HELP,

	CMD_MAX
} debug_cmd_t;

#define func_type_map(a) {\
	if (!strcmp(a, "h")) {\
		str_type = "hiu";\
		func_read = cvbs_out_hiu_read;\
		func_write = cvbs_out_hiu_write;\
		func_getb = cvbs_out_hiu_getb;\
		func_setb = cvbs_out_hiu_setb;\
	} \
	else if (!strcmp(a, "v")) {\
		str_type = "vcbus";\
		func_read = cvbs_out_reg_read;\
		func_write = cvbs_out_reg_write;\
		func_getb = cvbs_out_reg_getb;\
		func_setb = cvbs_out_reg_setb;\
	} \
}

static void cvbs_debug_store(char *buf)
{
	unsigned int ret = 0;
	unsigned long addr, start, end, value, length, old;
	unsigned int argc;
	char *p = NULL, *para = NULL,
		*argv[6] = {NULL, NULL, NULL, NULL, NULL, NULL};
	char *str_type = NULL;
	unsigned int i, cmd;
	unsigned int (*func_read)(unsigned int)  = NULL;
	void (*func_write)(unsigned int, unsigned int) = NULL;
	unsigned int (*func_getb)(unsigned int,
		unsigned int, unsigned int) = NULL;
	void (*func_setb)(unsigned int, unsigned int,
		unsigned int, unsigned int) = NULL;

	p = kstrdup(buf, GFP_KERNEL);
	for (argc = 0; argc < 6; argc++) {
		para = strsep(&p, " ");
		if (para == NULL)
			break;
		argv[argc] = para;
	}

	if (!strcmp(argv[0], "r"))
		cmd = CMD_REG_READ;
	else if (!strcmp(argv[0], "rb"))
		cmd = CMD_REG_READ_BITS;
	else if (!strcmp(argv[0], "dump"))
		cmd = CMD_REG_DUMP;
	else if (!strcmp(argv[0], "w"))
		cmd = CMD_REG_WRITE;
	else if (!strcmp(argv[0], "wb"))
		cmd = CMD_REG_WRITE_BITS;
	else if (!strncmp(argv[0], "clkdump", strlen("clkdump")))
		cmd = CMD_CLK_DUMP;
	else if (!strncmp(argv[0], "clkmsr", strlen("clkmsr")))
		cmd = CMD_CLK_MSR;
	else if (!strncmp(argv[0], "bist", strlen("bist")))
		cmd = CMD_BIST;
	else if (!strncmp(argv[0], "vpset", strlen("vpset")))
		cmd = CMD_VP_SET;
	else if (!strncmp(argv[0], "vpget", strlen("vpget")))
		cmd = CMD_VP_GET;
	else if (!strncmp(argv[0], "help", strlen("help")))
		cmd = CMD_HELP;
	else {
		print_info("[%s] invalid cmd = %s!\n", __func__, argv[0]);
		goto DEBUG_END;
	}

	switch (cmd) {
	case CMD_REG_READ:
		if (argc != 3) {
			print_info("[%s] cmd_reg_read format: r c/h/v address_hex\n",
				__func__);
			goto DEBUG_END;
		}

		func_type_map(argv[1]);
		ret = kstrtoul(argv[2], 16, &addr);

		print_info("read %s[0x%x] = 0x%x\n",
			str_type, (unsigned int)addr, func_read(addr));

		break;

	case CMD_REG_READ_BITS:
		if (argc != 5) {
			print_info("[%s] cmd_reg_read_bits format:\n"
			"\trb c/h/v address_hex start_dec length_dec\n",
				__func__);
			goto DEBUG_END;
		}

		func_type_map(argv[1]);
		ret = kstrtoul(argv[2], 16, &addr);
		ret = kstrtoul(argv[3], 10, &start);
		ret = kstrtoul(argv[4], 10, &length);

		if (length == 1)
			print_info("read_bits %s[0x%x] = 0x%x, bit[%d] = 0x%x\n",
			str_type, (unsigned int)addr,
			func_read(addr), (unsigned int)start,
			func_getb(addr, start, length));
		else
			print_info("read_bits %s[0x%x] = 0x%x, bit[%d-%d] = 0x%x\n",
			str_type, (unsigned int)addr,
			func_read(addr),
			(unsigned int)start+(unsigned int)length-1,
			(unsigned int)start,
			func_getb(addr, start, length));
		break;

	case CMD_REG_DUMP:
		if (argc != 4) {
			print_info("[%s] cmd_reg_dump format: dump c/h/v start_dec end_dec\n",
				__func__);
			goto DEBUG_END;
		}

		func_type_map(argv[1]);

		ret = kstrtoul(argv[2], 16, &start);
		ret = kstrtoul(argv[3], 16, &end);

		for (i = start; i <= end; i++)
			print_info("%s[0x%x] = 0x%x\n",
				str_type, i, func_read(i));

		break;

	case CMD_REG_WRITE:
		if (argc != 4) {
			print_info("[%s] cmd_reg_write format: w value_hex c/h/v address_hex\n",
				__func__);
			goto DEBUG_END;
		}

		func_type_map(argv[2]);

		ret = kstrtoul(argv[1], 16, &value);
		ret = kstrtoul(argv[3], 16, &addr);

		func_write(addr, value);
		print_info("write %s[0x%x] = 0x%x\n", str_type,
			(unsigned int)addr, (unsigned int)value);
		break;

	case CMD_REG_WRITE_BITS:
		if (argc != 6) {
			print_info("[%s] cmd_reg_wrute_bits format:\n"
			"\twb value_hex c/h/v address_hex start_dec length_dec\n",
				__func__);
			goto DEBUG_END;
		}

		func_type_map(argv[2]);

		ret = kstrtoul(argv[1], 16, &value);
		ret = kstrtoul(argv[3], 16, &addr);
		ret = kstrtoul(argv[4], 10, &start);
		ret = kstrtoul(argv[5], 10, &length);

		old = func_read(addr);
		func_setb(addr, value, start, length);
		print_info("write_bits %s[0x%x] old = 0x%x, new = 0x%x\n",
			str_type, (unsigned int)addr,
			(unsigned int)old, func_read(addr));
		break;

	case CMD_CLK_DUMP:
		dump_clk_registers();
		break;

	case CMD_CLK_MSR:
		/* todo */
		print_info("cvbs: debug_store: clk_msr todo!\n");
		break;

	case CMD_BIST:
		if (argc != 2) {
			print_info("[%s] cmd_bist format:\n"
			"\tbist 0/1/2/3/off\n", __func__);
			goto DEBUG_END;
		}

		bist_test_store(argv[1]);
		break;

	case CMD_VP_SET:
		if (argc != 2) {
			print_info("[%s] cmd_vp_set format:\n"
			"\tvpset 0/1/2\n", __func__);
			goto DEBUG_END;
		}
		if (info->vinfo->mode != VMODE_CVBS) {
			print_info("NOT VMODE_CVBS,Return\n");
			return;
		}
		ret = kstrtoul(argv[1], 16, &value);
		cvbs_performance_index = (value > 2) ? 0 : value;
		cvbs_performance_enhancement(local_cvbs_mode);
		cvbs_performance_regs_dump();
		break;

	case CMD_VP_GET:
		print_info("current performance index: %d\n",
			cvbs_performance_index);
		cvbs_performance_regs_dump();
		break;

	case CMD_HELP:
		print_info("command format:\n"
		"\tr c/h/v address_hex\n"
		"\trb c/h/v address_hex start_dec length_dec\n"
		"\tdump c/h/v start_dec end_dec\n"
		"\tw value_hex c/h/v address_hex\n"
		"\twb value_hex c/h/v address_hex start_dec length_dec\n"
		"\tbist 0/1/2/3/off\n"
		"\tclkdump\n");
		break;
	default:
		break;
	}

DEBUG_END:
	kfree(p);
}

static  struct  class_attribute   *cvbs_attr[] = {
	&class_CVBS_attr_vdac_power_level,
	&class_CVBS_attr_debug,
#ifdef CONFIG_AMLOGIC_WSS
	&class_CVBS_attr_wss,
#endif
};



static int create_cvbs_attr(struct disp_module_info_s *info)
{
	/* create base class for display */
	int i;
	int ret = 0;

	info->base_class = class_create(THIS_MODULE, CVBS_CLASS_NAME);
	if (IS_ERR(info->base_class)) {
		ret = PTR_ERR(info->base_class);
		goto fail_create_class;
	}
	/* create class attr */
	for (i = 0; i < ARRAY_SIZE(cvbs_attr); i++) {
		if (class_create_file(info->base_class, cvbs_attr[i]) < 0)
			goto fail_class_create_file;
	}
	/*cdev_init(info->cdev, &am_cvbs_fops);*/
	info->cdev = cdev_alloc();
	info->cdev->ops = &am_cvbs_fops;
	info->cdev->owner = THIS_MODULE;
	ret = cdev_add(info->cdev, info->devno, 1);
	if (ret)
		goto fail_add_cdev;
	info->dev = device_create(info->base_class, NULL, info->devno,
			NULL, CVBS_NAME);
	if (IS_ERR(info->dev)) {
		ret = PTR_ERR(info->dev);
		goto fail_create_device;
	} else {
		cvbs_log_info("create cdev %s\n", CVBS_NAME);
	}
	return 0;

fail_create_device:
	cvbs_log_info("[cvbs.] : cvbs device create error.\n");
	cdev_del(info->cdev);
fail_add_cdev:
	cvbs_log_info("[cvbs.] : cvbs add device error.\n");
	kfree(info);
fail_class_create_file:
	cvbs_log_info("[cvbs.] : cvbs class create file error.\n");
	for (i = 0; i < ARRAY_SIZE(cvbs_attr); i++)
		class_remove_file(info->base_class, cvbs_attr[i]);
	class_destroy(info->base_class);
fail_create_class:
	cvbs_log_info("[cvbs.] : cvbs class create error.\n");
	unregister_chrdev_region(info->devno, 1);
	return ret;
}
/* **************************************************** */

#ifdef CONFIG_INSTABOOT
struct class  *info_base_class;
static int cvbsconf_suspend(void)
{
	info_base_class = info->base_class;
	return 0;
}

static void cvbsconf_resume(void)
{
	info->base_class = info_base_class;
}

static struct syscore_ops cvbsconf_ops = {
	.suspend = cvbsconf_suspend,
	.resume = cvbsconf_resume,
	.shutdown = NULL,
};
#endif

static int cvbsout_probe(struct platform_device *pdev)
{
	int  ret;
#ifdef CONFIG_INSTABOOT
	INIT_LIST_HEAD(&cvbsconf_ops.node);
	register_syscore_ops(&cvbsconf_ops);
#endif
	local_cvbs_mode = MODE_MAX;
	cvbs_out_ioremap();
	info = &disp_module_info;
	cvbs_log_info("%s\n", __func__);
	ret = alloc_chrdev_region(&info->devno, 0, 1, CVBS_NAME);
	if (ret < 0) {
		cvbs_log_err("alloc_chrdev_region error\n");
		return  ret;
	}
	_init_cvbsout();
	cvbs_log_err("chrdev devno %d for disp\n", info->devno);
	if (vout_register_server(&cvbs_server))
		cvbs_log_err("register cvbs module server fail\n");
	else
		cvbs_log_info("register cvbs module server ok\n");
	ret = create_cvbs_attr(info);
	if (ret < 0) {
		cvbs_log_err("create_cvbs_attr error\n");
		return -1;
	}
	cvbs_log_info("%s OK\n", __func__);
	return 0;
}

static int cvbsout_remove(struct platform_device *pdev)
{
	int i;

	if (info->base_class) {
		for (i = 0; i < ARRAY_SIZE(cvbs_attr); i++)
			class_remove_file(info->base_class, cvbs_attr[i]);
		class_destroy(info->base_class);
	}
	if (info) {
		cdev_del(info->cdev);
		kfree(info);
	}
	vout_unregister_server(&cvbs_server);
	cvbs_log_info("%s\n", __func__);
	return 0;
}

static void cvbsout_shutdown(struct platform_device *pdev)
{
	cvbs_out_disable_venc();
	cvbs_out_disable_clk();

#ifdef CONFIG_AMLOGIC_VPU
	cvbs_out_vpu_power_ctrl(0);
	cvbs_out_vpu_gate_ctrl(0);
#endif
}

#ifdef CONFIG_OF
static const struct of_device_id cvbsout_dt_match[] = {
	{
		.compatible = "amlogic, cvbsout",
	},
	{},
};
#endif

static struct platform_driver cvbsout_driver = {
	.probe = cvbsout_probe,
	.remove = cvbsout_remove,
	.shutdown = cvbsout_shutdown,
	.driver = {
		.name = "cvbsout",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = cvbsout_dt_match,
#endif
	},
};

static int __init cvbs_init_module(void)
{
	/* cvbs_log_info("%s module init\n", __func__); */
	if (platform_driver_register(&cvbsout_driver)) {
		cvbs_log_err("%s failed to register module\n", __func__);
		return -ENODEV;
	}
	return 0;
}

static __exit void cvbs_exit_module(void)
{
	/* cvbs_log_info("%s module exit\n", __func__); */
	platform_driver_unregister(&cvbsout_driver);
}

static int __init vdac_config_bootargs_setup(char *line)
{
	unsigned long cfg = 0x0;
	int ret = 0;

	cvbs_log_info("cvbs trimming line = %s\n", line);
	ret = kstrtoul(line, 16, (unsigned long *)&cfg);
	cvbs_config_vdac((cfg & 0xff00) >> 8, cfg & 0xff);
	return 1;
}

__setup("vdaccfg=", vdac_config_bootargs_setup);

static int __init cvbs_performance_setup(char *line)
{
	unsigned long cfg = 0x0;
	int ret = 0;

	cvbs_log_info("cvbs performance line = %s\n", line);
	ret = kstrtoul(line, 10, (unsigned long *)&cfg);
	cvbs_performance_config(cfg);
	return 0;
}
__setup("cvbsdrv=", cvbs_performance_setup);

arch_initcall(cvbs_init_module);
module_exit(cvbs_exit_module);

MODULE_AUTHOR("Platform-BJ <platform.bj@amlogic.com>");
MODULE_DESCRIPTION("TV Output Module");
MODULE_LICENSE("GPL");
