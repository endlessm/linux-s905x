/* drivers/amlogic/ethernet/phy/amlogic.c
 *
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
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
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <linux/amlogic/aml_pmu4.h>

#define  SMI_ADDR_TSTCNTL     20
#define  SMI_ADDR_TSTREAD1    21
#define  SMI_ADDR_TSTREAD2    22
#define  SMI_ADDR_TSTWRITE    23

#define  WR_ADDR_A0CFG        0x11
#define  WR_ADDR_A1CFG        0x12
#define  WR_ADDR_A2CFG        0x13
#define  WR_ADDR_A3CFG        0x14
#define  WR_ADDR_A4CFG        0x15
#define  WR_ADDR_A5CFG        0x16
#define  WR_ADDR_A6CFG        0x17
#define  WR_ADDR_A7CFG        0x18
#define  WR_ADDR_A8CFG        0x1a
#define  WR_ADDR_A9CFG        0x1b
#define  WR_ADDR_A10CFG       0x1c
#define  WR_ADDR_A11CFG       0x1d

#define  RD_ADDR_A3CFG        (0x14 << 5)
#define  RD_ADDR_A4CFG        (0x15 << 5)
#define  RD_ADDR_A5CFG        (0x16 << 5)
#define  RD_ADDR_A6CFG        (0x17 << 5)

#define  TSTCNTL_RD           ((1 << 15) | (1 << 10))
#define  TSTCNTL_WR           ((1 << 14) | (1 << 10))

#define MII_INTERNAL_ISF 29 /* Interrupt Source Flags */
#define MII_INTERNAL_IM  30 /* Interrupt Mask */
#define MII_INTERNAL_CTRL_STATUS 17 /* Mode/Status Register */
#define MII_INTERNAL_SPECIAL_MODES 18 /* Special Modes Register */

#define MII_INTERNAL_ISF_INT1 (1<<1) /* Auto-Negotiation Page Received */
#define MII_INTERNAL_ISF_INT2 (1<<2) /* Parallel Detection Fault */
#define MII_INTERNAL_ISF_INT3 (1<<3) /* Auto-Negotiation LP Ack */
#define MII_INTERNAL_ISF_INT4 (1<<4) /* Link Down */
#define MII_INTERNAL_ISF_INT5 (1<<5) /* Remote Fault Detected */
#define MII_INTERNAL_ISF_INT6 (1<<6) /* Auto-Negotiation complete */
#define MII_INTERNAL_ISF_INT7 (1<<7) /* ENERGYON */

#define MII_INTERNAL_ISF_INT_ALL (0x0e)

#define MII_INTERNAL_ISF_INT_PHYLIB_EVENTS \
	(MII_INTERNAL_ISF_INT6 | MII_INTERNAL_ISF_INT4 | \
	 MII_INTERNAL_ISF_INT7)

#define MII_INTERNAL_EDPWRDOWN (1 << 13) /* EDPWRDOWN */
#define MII_INTERNAL_ENERGYON  (1 << 1)  /* ENERGYON */

#define MII_INTERNAL_MODE_MASK      0xE0
#define MII_INTERNAL_MODE_POWERDOWN 0xC0 /* Power Down mode */
#define MII_INTERNAL_MODE_ALL       0xE0 /* All capable mode */
static void initTSTMODE(struct phy_device *phydev)
{
	phy_write(phydev, SMI_ADDR_TSTCNTL, 0x0400);
	phy_write(phydev, SMI_ADDR_TSTCNTL, 0x0000);
	phy_write(phydev, SMI_ADDR_TSTCNTL, 0x0400);
}

static void closeTSTMODE(struct phy_device *phydev)
{
	phy_write(phydev, SMI_ADDR_TSTCNTL, 0x0000);
}


static void init_internal_phy(struct phy_device *phydev)
{
	initTSTMODE(phydev);
 /*write tstcntl addr val*/
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x1354);
	/*write addr 0*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x38);
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A0CFG);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x0c00);
	/*write addr 0x12*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A1CFG);
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x3e00);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A2CFG);
	/*write addr 0x13*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0xf902);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A3CFG);
	/*write addr 0x14*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x3412);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A4CFG);
	/*write addr 0x15*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x2636);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A5CFG);
	/*write addr 0x16*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 3);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A7CFG);
	/*write addr 0x18*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x108);
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A9CFG);
	/*write addr 0x1b*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0xda00);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A11CFG);
	/*write addr 0x1d*/
	closeTSTMODE(phydev);
}


static void init_internal_phy_10B(struct phy_device *phydev)
{

	initTSTMODE(phydev);
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x0000);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR);
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x38);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A0CFG);
	/*write addr 0x11*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x0c00);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A1CFG);
	/*write addr 0x12*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x3e00);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A2CFG);
	/*write addr 0x13*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0xf902);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A3CFG);
	/*write addr 0x14*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x3412);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A4CFG);
	/*write addr 0x15*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x2236);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A5CFG);
	/*write addr 0x16*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 3);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A7CFG);
	/*write addr 0x18*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x108);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A9CFG);
	/*write addr 0x1b*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0xda00);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A11CFG);
	/*write addr 0x1d*/
	closeTSTMODE(phydev);
}

static void init_internal_phy_100B(struct phy_device *phydev)
{

	initTSTMODE(phydev);
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x9354);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|0x00);
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x38);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A0CFG);
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x0c00);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A1CFG);
	/*write addr 0x12*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x3e00);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A2CFG);
	/*write addr 0x13*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0xf902);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A3CFG);
	/*write addr 0x14*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x3412);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A4CFG);
	/*write addr 0x15*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0xa406);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A5CFG);
	/*write addr 0x16*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x0003);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A7CFG);
	/*write addr 0x18*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x00a6);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A9CFG);
	/*write addr 0x1b*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0xda00);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A11CFG);
	/*write addr 0x1d*/
	closeTSTMODE(phydev);
}

/*	0x00	1354		1354	0000	9354
 *	0x13	3600		3400	3410	3400
 *	0x14	5100		7900	7900	7900
 *	0x15	441c		3404	3404	3404
 *	0x16	8406		8646	0246	8446
 *	0x18	0003		0003	0003	0003
 *	0x1b	00a0		40a0	40a4	40a6
 *	0x1d	0a00		0200	0200	0200
*/
static void init_pmu4_phy(struct phy_device *phydev)
{
	initTSTMODE(phydev);
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x1354);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR);
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x3400);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A2CFG);
	/*write addr 0x13*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x7900);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A3CFG);
	/*write addr 0x14*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x3404);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A4CFG);
	/*write addr 0x15*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0xa636);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A5CFG);
	/*write addr 0x16*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 5);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A7CFG);
	/*write addr 0x18*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x0108);
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A9CFG);
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x200);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A11CFG);
	/*write addr 0x1d*/
	closeTSTMODE(phydev);
}


static void init_pmu4_phy_10B(struct phy_device *phydev)
{
	initTSTMODE(phydev);
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x0000);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR);

	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x3410);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A2CFG);
	/*write addr 0x13*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x7900);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A3CFG);
	/*write addr 0x14*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x3404);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A4CFG);
	/*write addr 0x15*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x8246);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A5CFG);
	/*write addr 0x16*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 5);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A7CFG);
	/*write addr 0x18*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x40a4);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A9CFG);
	/*write addr 0x1b*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x200);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A11CFG);
	/*write addr 0x1d*/
	closeTSTMODE(phydev);
}
/*	0x00	1354		1354	0000	9354
 *	0x13	3600		3400	3410	3400
 *	0x14	5100		7900	7900	7900
 *	0x15	441c		3404	3404	3404
 *	0x16	8406		8646	0246	8446
 *	0x18	0003		0003	0003	0003
 *	0x1b	00a0		40a0	40a4	40a6
 *	0x1d	0a00		0200	0200	0200
*/
static void init_pmu4_phy_100B(struct phy_device *phydev)
{
	initTSTMODE(phydev);
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x9354);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|0x00);
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x3000);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A2CFG);
	/*write addr 0x13*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0xb902);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A3CFG);
	/*write addr 0x14*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x3404);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A4CFG);
	/*write addr 0x15*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x8446);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A5CFG);
	/*write addr 0x16*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x0005);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A7CFG);
	/*write addr 0x18*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x40a6);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A9CFG);
	/*write addr 0x1b*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x200);
	/*write val*/
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR|WR_ADDR_A11CFG);
	/*write addr 0x1d*/
	closeTSTMODE(phydev);
}

static int amlogic_phy_config_intr(struct phy_device *phydev)
{
	int rc = phy_write(phydev, MII_INTERNAL_IM,
			((PHY_INTERRUPT_ENABLED == phydev->interrupts)
			 ? MII_INTERNAL_ISF_INT_PHYLIB_EVENTS
			 : 0));

	return rc < 0 ? rc : 0;
}

static int amlogic_phy_ack_interrupt(struct phy_device *phydev)
{
	int rc = phy_read(phydev, MII_INTERNAL_ISF);

	return rc < 0 ? rc : 0;
}

static int amlogic_phy_config_init(struct phy_device *phydev)
{
	int rc = phy_read(phydev, MII_INTERNAL_SPECIAL_MODES);
	if (rc < 0)
		return rc;
	init_internal_phy(phydev);
	/* If the AML PHY is in power down mode, then set it
	 * in all capable mode before using it.
	 */
	if ((rc & MII_INTERNAL_MODE_MASK) == MII_INTERNAL_MODE_POWERDOWN) {
		int timeout = 50000;

		/* set "all capable" mode and reset the phy */
		rc |= MII_INTERNAL_MODE_ALL;
		phy_write(phydev, MII_INTERNAL_SPECIAL_MODES, rc);
		phy_write(phydev, MII_BMCR, BMCR_RESET);

		/* wait end of reset (max 500 ms) */
		do {
			udelay(10);
			if (timeout-- == 0)
				return -1;
			rc = phy_read(phydev, MII_BMCR);
		} while (rc & BMCR_RESET);
	}

	rc = phy_read(phydev, MII_INTERNAL_CTRL_STATUS);
	if (rc < 0)
		return rc;
	rc = phy_write(phydev, MII_INTERNAL_CTRL_STATUS,
		       rc & ~MII_INTERNAL_EDPWRDOWN);
	if (rc < 0)
		return rc;

	return amlogic_phy_ack_interrupt(phydev);
}
static int pmu4_phy_config_init(struct phy_device *phydev)
{
	int rc = phy_read(phydev, MII_INTERNAL_SPECIAL_MODES);
	if (rc < 0)
		return rc;

	init_pmu4_phy(phydev);
	/* If the AML PHY is in power down mode, then set it
	 * in all capable mode before using it.
	 */

	if ((rc & MII_INTERNAL_MODE_MASK) == MII_INTERNAL_MODE_POWERDOWN) {
		int timeout = 50000;

		/* set "all capable" mode and reset the phy */
		rc |= MII_INTERNAL_MODE_ALL;
		phy_write(phydev, MII_INTERNAL_SPECIAL_MODES, rc);
		phy_write(phydev, MII_BMCR, BMCR_RESET);

		/* wait end of reset (max 500 ms) */
		do {
			udelay(10);
			if (timeout-- == 0)
				return -1;
			rc = phy_read(phydev, MII_BMCR);
		} while (rc & BMCR_RESET);
	}

	rc = phy_read(phydev, MII_INTERNAL_CTRL_STATUS);
	if (rc < 0)
		return rc;

	/*Enable energy detect mode for this AML Transceivers*/
	rc = phy_write(phydev, MII_INTERNAL_CTRL_STATUS,
		       rc & ~MII_INTERNAL_EDPWRDOWN);
	if (rc < 0)
		return rc;
	return amlogic_phy_ack_interrupt(phydev);
}

/* This workaround will manually toggle the PHY on/off upon calls to read_status
 * in order to generate link test pulses if the link is down.  If a link partner
 * is present, it will respond to the pulses, which will cause the ENERGYON bit
 * to be set and will cause the EDPD mode to be exited.
 */
static int internal_read_status(struct phy_device *phydev)
{
	int err = genphy_read_status(phydev);
	if (phydev->speed == SPEED_10)
		init_internal_phy_10B(phydev);
	if (phydev->speed == SPEED_100)
		init_internal_phy_100B(phydev);
	if (!(AUTONEG_ENABLE == phydev->autoneg)) {
		if (!phydev->link) {
			/* Disable EDPD to wake up PHY */
			int rc = phy_read(phydev,
				MII_INTERNAL_CTRL_STATUS);
			if (rc < 0)
				return rc;

			rc = phy_write(phydev, MII_INTERNAL_CTRL_STATUS,
					rc & ~MII_INTERNAL_EDPWRDOWN);
			if (rc < 0)
				return rc;

			/* Sleep 64 ms to allow ~5 link
			 * test pulses to be sent
			 */
			msleep(64);
			/* Re-enable EDPD */
			rc = phy_read(phydev, MII_INTERNAL_CTRL_STATUS);
			if (rc < 0)
				return rc;

			rc = phy_write(phydev, MII_INTERNAL_CTRL_STATUS,
					rc | MII_INTERNAL_EDPWRDOWN);
			if (rc < 0)
				return rc;
		}
	}
	return err;
}
static int pmu4_read_status(struct phy_device *phydev)
{
	int err;
	int value;
	int timeout = 50000;
	uint8_t pmu4_ver = 0;
	uint8_t val;

	if (phydev->link) {
			aml_pmu4_read(0x7B, &val);
			aml_pmu4_write(0x7B, val|0x4);
	} else {
			aml_pmu4_read(0x7B, &val);
			aml_pmu4_write(0x7B, val&(~0x4));
	}

	/* strange state in PMU4v1, reset to return normal */
	err = aml_pmu4_version(&pmu4_ver);
	if (err == 0 && pmu4_ver == PMU4_VA_ID) {
		value = phy_read(phydev, MII_BMSR);
		if ((value & BMSR_ANEGCOMPLETE) && (!(value & BMSR_LSTATUS))) {
			phy_write(phydev, MII_BMCR, BMCR_RESET);
			/* wait end of reset (max 500 ms) */
			do {
				udelay(10);
				if (timeout-- == 0)
					return -1;
				value = phy_read(phydev, MII_BMCR);
			} while (value & BMCR_RESET);
		}
	}

	err = genphy_read_status(phydev);
	if (phydev->speed == SPEED_10)
		init_pmu4_phy_10B(phydev);
	if (phydev->speed == SPEED_100)
		init_pmu4_phy_100B(phydev);
	if (!(AUTONEG_ENABLE == phydev->autoneg)) {
		if (!phydev->link) {
			/* Disable EDPD to wake up PHY */
			int rc = phy_read(phydev, MII_INTERNAL_CTRL_STATUS);
			if (rc < 0)
				return rc;

			rc = phy_write(phydev, MII_INTERNAL_CTRL_STATUS,
					rc & ~MII_INTERNAL_EDPWRDOWN);
			if (rc < 0)
				return rc;
			/* Sleep 64 ms to allow ~5
			 * link test pulses to be sent
			 */
			msleep(64);
			/* Re-enable EDPD */
			rc = phy_read(phydev, MII_INTERNAL_CTRL_STATUS);
			if (rc < 0)
				return rc;

			rc = phy_write(phydev, MII_INTERNAL_CTRL_STATUS,
					rc | MII_INTERNAL_EDPWRDOWN);
			if (rc < 0)
				return rc;
		}
	}
	return err;
}

static void internal_config(struct phy_device *phydev)
{
	/*enable auto mdix*/
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
	phy_write(phydev, SMI_ADDR_TSTWRITE, mac_addr[0]|mac_addr[1]<<8);
	phy_write(phydev, 0x14, 0x4800|0x00);
	phy_write(phydev, SMI_ADDR_TSTWRITE, mac_addr[2]|mac_addr[3]<<8);
	phy_write(phydev, 0x14, 0x4800|0x01);
	phy_write(phydev, SMI_ADDR_TSTWRITE, mac_addr[4]|mac_addr[5]<<8);
	phy_write(phydev, 0x14, 0x4800|0x02);
	/*enable wol*/
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0x9);
	phy_write(phydev, 0x14, 0x4800|0x03);
	/*enable interrupt*/
	phy_write(phydev, 0x1E, 0xe00);
}

static int internal_config_init(struct phy_device *phydev)
{

	int val;
	u32 features;
	internal_config(phydev);
	internal_wol_init(phydev);

	/* For now, I'll claim that the generic driver supports
	 * all possible port types
	 */
	features = (SUPPORTED_TP | SUPPORTED_MII
			| SUPPORTED_AUI | SUPPORTED_FIBRE |
			SUPPORTED_BNC);

	/* Do we support autonegotiation? */
	val = phy_read(phydev, MII_BMSR);
	if (val < 0)
		return val;

	if (val & BMSR_ANEGCAPABLE)
		features |= SUPPORTED_Autoneg;

	if (val & BMSR_100FULL)
		features |= SUPPORTED_100baseT_Full;
	if (val & BMSR_100HALF)
		features |= SUPPORTED_100baseT_Half;
	if (val & BMSR_10FULL)
		features |= SUPPORTED_10baseT_Full;
	if (val & BMSR_10HALF)
		features |= SUPPORTED_10baseT_Half;

	if (val & BMSR_ESTATEN) {
		val = phy_read(phydev, MII_ESTATUS);
		if (val < 0)
			return val;

		if (val & ESTATUS_1000_TFULL)
			features |= SUPPORTED_1000baseT_Full;
		if (val & ESTATUS_1000_THALF)
			features |= SUPPORTED_1000baseT_Half;
	}

	phydev->supported = features;
	phydev->advertising = features;

	return 0;

}

int internal_phy_resume(struct phy_device *phydev)
{
	int rc1, rc2;
	rc1 = genphy_resume(phydev);
	rc2 = phy_init_hw(phydev);
	return rc1|rc2;
}
int internal_phy_suspend(struct phy_device *phydev)
{
	/*do nothing here if you want WOL enabled*/
	int value;

	mutex_lock(&phydev->lock);

	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value | BMCR_PDOWN);

	mutex_unlock(&phydev->lock);

	return 0;
}


static int amlogic_phy_config_aneg(struct phy_device *phydev)
{

	return genphy_config_aneg(phydev);
}
static int amlogic_phy_suspend(struct phy_device *phydev)
{
	uint8_t val;
	/*enable 50M clock,or eth up will fail*/
	aml_pmu4_read(0x7B, &val);
	aml_pmu4_write(0x7B, val|0x4);
	return genphy_suspend(phydev);
}

static struct phy_driver amlogic_phy_driver[] = {
	{
		.phy_id		= 0x79898963,
		.phy_id_mask	= 0xffffffff,
		.name		= "AMLOGIC internal phy",

		.features	= (PHY_BASIC_FEATURES | SUPPORTED_Pause
				| SUPPORTED_Asym_Pause),
		.flags		= PHY_HAS_INTERRUPT | PHY_HAS_MAGICANEG,
		/* basic functions */
		.config_aneg	= &amlogic_phy_config_aneg,
		.read_status	= &internal_read_status,
		.config_init	= &amlogic_phy_config_init,
		/* IRQ related */
		.ack_interrupt	= &amlogic_phy_ack_interrupt,
		.config_intr	= &amlogic_phy_config_intr,

		.suspend	= genphy_suspend,
		.resume		= genphy_resume,

		.driver		= { .owner = THIS_MODULE, }
	},
	{
		.phy_id		= 0x20142014,
		.phy_id_mask	= 0xffffffff,
		.name		= "AMLOGIC pmu4 phy",

		.features	= (PHY_BASIC_FEATURES | SUPPORTED_Pause
				| SUPPORTED_Asym_Pause),
		.flags		= PHY_HAS_INTERRUPT | PHY_HAS_MAGICANEG,

		/* basic functions */
		.config_aneg	= &amlogic_phy_config_aneg,
		.read_status	= &pmu4_read_status,
		.config_init	= &pmu4_phy_config_init,

		/* IRQ related */
		.ack_interrupt	= &amlogic_phy_ack_interrupt,
		.config_intr	= &amlogic_phy_config_intr,

		.suspend	= amlogic_phy_suspend,
		.resume		= genphy_resume,

		.driver		= { .owner = THIS_MODULE, }
	} };
static int internal_phy_read_status(struct phy_device *phydev)
{
	int err;
	int reg31 = 0;

	/* Update the link, but return if there was an error */
	err = genphy_update_link(phydev);
	if (err)
		return err;

	phydev->lp_advertising = 0;

	if (AUTONEG_ENABLE == phydev->autoneg) {
		reg31 = phy_read(phydev, 0x1f);
		if (reg31 | 0x1000) {
			phydev->pause = 0;
			phydev->asym_pause = 0;
			phydev->speed = SPEED_10;
			phydev->duplex = DUPLEX_HALF;
			reg31 &= 0x1c;
			if (reg31 == 0x4) {
				phydev->speed = SPEED_10;
				phydev->duplex = DUPLEX_HALF;
			}
			if (reg31 == 0x14) {
				phydev->speed = SPEED_10;
				phydev->duplex = DUPLEX_FULL;

			}
			if (reg31 == 0x8) {
				phydev->speed = SPEED_100;

				phydev->duplex = DUPLEX_HALF;
			}
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

	return 0;
}

static struct phy_driver internal_phy = {
	.phy_id = 0x01814400,
	.name		= "gxl internal phy",
	.phy_id_mask	= 0x0fffffff,
	.config_init	= internal_config_init,
	.features	= 0,
	.config_aneg	= genphy_config_aneg,
	.read_status	= internal_phy_read_status,
	.suspend	= internal_phy_suspend,
	.resume = internal_phy_resume,
	.driver	= { .owner = THIS_MODULE, },
};

static int __init amlogic_init(void)
{
	int rc1, rc2;
	rc1 = phy_drivers_register(amlogic_phy_driver,
			ARRAY_SIZE(amlogic_phy_driver));
	rc2 = phy_driver_register(&internal_phy);

	return rc1 || rc2;
}

static void __exit amlogic_exit(void)
{
	phy_drivers_unregister(amlogic_phy_driver,
			ARRAY_SIZE(amlogic_phy_driver));
	phy_driver_unregister(&internal_phy);
}

MODULE_DESCRIPTION("Amlogic PHY driver");
MODULE_AUTHOR("Baoqi wang");
MODULE_LICENSE("GPL");

module_init(amlogic_init);
module_exit(amlogic_exit);

static struct mdio_device_id __maybe_unused amlogic_tbl[] = {
	{ 0x79898963, 0xffffffff },
	{ 0x20142014, 0xffffffff },
	{ }
};

MODULE_DEVICE_TABLE(mdio, amlogic_tbl);
