/*
 * drivers/amlogic/ethernet/phy/amlogic.c
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

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/phy.h>

#define  SMI_ADDR_TSTWRITE    23

MODULE_DESCRIPTION("amlogic internal ethernet phy driver");
MODULE_AUTHOR("Yizhou Jiang");
MODULE_LICENSE("GPL");
void set_a3_config(struct phy_device *phydev)
{
	int value = 0;

	phy_write(phydev, 0x17, 0xa900);
	phy_write(phydev, 0x14, 0x4414);
	phy_write(phydev, 0x14, 0x8680);
	value = phy_read(phydev, 0x15);
}

void internal_wol_init(struct phy_device *phydev)
{
	int val;
	unsigned char *mac_addr;

	mac_addr = phydev->attached_dev->dev_addr;
	/*chose wol register bank*/
	val = phy_read(phydev, 0x14);
	val |= 0x800;
	val &= ~0x1000;
	phy_write(phydev, 0x14, val);/*write data to wol register bank*/
	/*write mac address*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, mac_addr[0] | mac_addr[1] << 8);
	phy_write(phydev, 0x14, 0x4800 | 0x00);
	phy_write(phydev, SMI_ADDR_TSTWRITE, mac_addr[2] | mac_addr[3] << 8);
	phy_write(phydev, 0x14, 0x4800 | 0x01);
	phy_write(phydev, SMI_ADDR_TSTWRITE, mac_addr[4] | mac_addr[5] << 8);
	phy_write(phydev, 0x14, 0x4800 | 0x02);
	/*enable wol*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x9);
	phy_write(phydev, 0x14, 0x4800 | 0x03);
	/*enable interrupt*/
	phy_write(phydev, 0x1E, 0xe00);
}

void internal_config(struct phy_device *phydev)
{
	int value;
	/*set reg27[12] = 1*/
	value = phy_read(phydev, 0x1b);
	phy_write(phydev, 0x1b, value | 0x1000);
	phy_write(phydev, 0x11, 0x0080);
	/*Enable Analog and DSP register Bank access by*/
	phy_write(phydev, 0x14, 0x0000);
	phy_write(phydev, 0x14, 0x0400);
	phy_write(phydev, 0x14, 0x0000);
	phy_write(phydev, 0x14, 0x0400);
	/*Write Analog register 23*/
	phy_write(phydev, 0x17, 0x8E0D);
	phy_write(phydev, 0x14, 0x4417);
	/*Enable fractional PLL*/
	phy_write(phydev, 0x17, 0x0005);
	phy_write(phydev, 0x14, 0x5C1B);
	/*Programme fraction FR_PLL_DIV1*/
	phy_write(phydev, 0x17, 0x029A);
	phy_write(phydev, 0x14, 0x5C1D);
	/*programme fraction FR_PLL_DiV1*/
	phy_write(phydev, 0x17, 0xAAAA);
	phy_write(phydev, 0x14, 0x5C1C);
	phy_write(phydev, 0x17, 0x000c);
	phy_write(phydev, 0x14, 0x4418);
	phy_write(phydev, 0x17, 0x1A0C);
	phy_write(phydev, 0x14, 0x4417); /* A6_CONFIG */
	phy_write(phydev, 0x17, 0x6400);
	phy_write(phydev, 0x14, 0x441A); /* A8_CONFIG */
	pr_info("internal phy init\n");
}

void reset_internal_phy(struct phy_device *phydev)
{
	int value;
	/*get value of bit 15:8*/
	/*if get 1, means power down reset or warm reset*/
	if (phydev->drv->features & 0xff00) {
		pr_info("power down and up\n");
		value = phy_read(phydev, MII_BMCR);
		phy_write(phydev, MII_BMCR, value | BMCR_PDOWN);
		msleep(50);
		value = phy_read(phydev, MII_BMCR);
		phy_write(phydev, MII_BMCR, value & ~BMCR_PDOWN);
		msleep(50);
	}

	phy_write(phydev, MII_BMCR, BMCR_RESET);
	msleep(50);
	internal_config(phydev);
	pr_info("reset phy\n");
}

static int internal_phy_read_status(struct phy_device *phydev)
{
	int err;
	int reg31 = 0;
	int wol_reg12;
	int linkup = 0;
	int val;
	static int reg12_error_count;
	/* Update the link, but return if there was an error */
	/* Bit 15: READ*/
	/*Bit 14: Write*/
	/*Bit 12:11: BANK_SEL (0: DSP, 1: WOL, 3: BIST)*/
	/*Bit 10: Test Mode*/
	/*Bit 9:5: Read Address*/
	/*Bit 4:0: Write Address*/
	/*read wol bank reg12*/
	val = ((1 << 15) | (1 << 11) | (1 << 10) | (12 << 5));
	phy_write(phydev, 0x14, val);
	wol_reg12 = phy_read(phydev, 0x15);
	if (phydev->link) {
		if ((wol_reg12 & 0x1000))
			reg12_error_count = 0;
		if (!(wol_reg12 & 0x1000)) {
			reg12_error_count++;
			pr_info("wol_reg12[12]==0, error\n");
		}
		if (reg12_error_count >= (phydev->drv->features & 0xff)) {
			reg12_error_count = 0;
			reset_internal_phy(phydev);
		}
	} else {
		reg12_error_count = 0;
	}
	linkup = phydev->link;
	err = genphy_update_link(phydev);
	if (err)
		return err;

	phydev->lp_advertising = 0;

	if (phydev->autoneg == AUTONEG_ENABLE) {
		/*read internal phy reg 0x1f*/
		reg31 = phy_read(phydev, 0x1f);
		/*bit 12 auto negotiation done*/
		if (reg31 | 0x1000) {
			phydev->pause = 0;
			phydev->asym_pause = 0;
			phydev->speed = SPEED_10;
			phydev->duplex = DUPLEX_HALF;
			/*bit 4:2 speed indication*/
			reg31 &= 0x1c;
			/*value 001: 10M/half*/
			if (reg31 == 0x4) {
				phydev->speed = SPEED_10;
				phydev->duplex = DUPLEX_HALF;
			}
			/*value 101: 10M/full*/
			if (reg31 == 0x14) {
				phydev->speed = SPEED_10;
				phydev->duplex = DUPLEX_FULL;
			}
			/*value 010: 100M/half*/
			if (reg31 == 0x8) {
				phydev->speed = SPEED_100;
				phydev->duplex = DUPLEX_HALF;
			}
			/*value 110: 100M/full*/
			if (reg31 == 0x18) {
				phydev->speed = SPEED_100;
				phydev->duplex = DUPLEX_FULL;
			}
		}
	} else {
		int bmcr = phy_read(phydev, MII_BMCR);

		if (bmcr < 0)
			return bmcr;

		if (bmcr & BMCR_FULLDPLX)
			phydev->duplex = DUPLEX_FULL;
		else
			phydev->duplex = DUPLEX_HALF;

		if (bmcr & BMCR_SPEED1000)
			phydev->speed = SPEED_1000;
		else if (bmcr & BMCR_SPEED100)
			phydev->speed = SPEED_100;
		else
			phydev->speed = SPEED_10;

		phydev->pause = 0;
		phydev->asym_pause = 0;
	}
	/*every time link up, set a3 config*/
	if ((linkup == 0) && (phydev->link == 1)) {
		if (phydev->speed == SPEED_100)
			set_a3_config(phydev);
	}

	return 0;
}

static int internal_config_init(struct phy_device *phydev)
{
	/*internal_wol_init(phydev);*/
	internal_config(phydev);
	return genphy_config_init(phydev);
}

static int internal_phy_resume(struct phy_device *phydev)
{
	int rc;

	rc = genphy_resume(phydev);
	phy_init_hw(phydev);
	return rc;
}

static struct phy_driver amlogic_internal_driver[] = { {
	.phy_id	= 0x01814400,
	.name		= "amlogic internal phy",
	.phy_id_mask	= 0x0fffffff,
	.config_init	= internal_config_init,
	/*1 means power down reset, 0 means marm reset*/
	/*bit 0-7,value f:count_sec=15*/
	.features	= 0x10f,
	.config_aneg	= genphy_config_aneg,
	.read_status	= internal_phy_read_status,
	.suspend	= genphy_suspend,
	.resume		= internal_phy_resume,
} };

module_phy_driver(amlogic_internal_driver);

static struct mdio_device_id __maybe_unused amlogic_tbl[] = {
	{ 0x01814400, 0xfffffff0 },
	{ }
};

MODULE_DEVICE_TABLE(mdio, amlogic_tbl);
