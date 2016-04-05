/*
 * Copyright (C) 2016, Endless Computers, Inc.
 * Author: Carlo Caione <carlo@endlessm.com>
 *
 * Adapted from drivers/hardkernel/odroid-sysfs.c
 * Copyright (C) 2014, Hardkernel Co,.Ltd
 * Author: Charles Park <charles.park@hardkernel.com>
 * Author: Dongjin Kim <tobetter@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/amlogic/iomap.h>

/*
 * Discover the boot device within MicroSD or eMMC
 * and return 1 for eMMC, otherwise 0.
 */
enum {
	BOOT_DEVICE_RESERVED = 0,
	BOOT_DEVICE_EMMC = 1,
	BOOT_DEVICE_NAND = 2,
	BOOT_DEVICE_SPI = 3,
	BOOT_DEVICE_SD = 4,
	BOOT_DEVICE_USB = 5,
	BOOT_DEVICE_MAX,
};

static int get_boot_device(void)
{
	int bootdev = aml_read_aobus(0x90 << 2) & 0xf;

	if (bootdev >= BOOT_DEVICE_MAX)
		return BOOT_DEVICE_RESERVED;

	return bootdev;
}

int board_boot_from_emmc(void)
{
	return !!(get_boot_device() == BOOT_DEVICE_EMMC);
}
EXPORT_SYMBOL(board_boot_from_emmc);
