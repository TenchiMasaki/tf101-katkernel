/*
 * arch/arm/mach-tegra/board-adam-bt.c
 *
 * Copyright (C) 2011 Eduardo José Tagle <ejtagle@tutopia.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/console.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/memblock.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/nand.h>
#include <mach/iomap.h>

#include "board.h"
#include "board-adam.h"
#include "clock.h"
#include "gpio-names.h"
#include "devices.h"

static struct resource adam_bcm4329_rfkill_resources[] = {
	{
		.name   = "bcm4329_nreset_gpio",
		.start  = ADAM_BT_RST,
		.end    = ADAM_BT_RST,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device adam_bcm4329_rfkill_device = {
	.name = "bcm4329_rfkill",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(adam_bcm4329_rfkill_resources),
	.resource       = adam_bcm4329_rfkill_resources,
};

void __init adam_bt_rfkill(void)
{
	/*Add Clock Resource*/
	int res = clk_add_alias("bcm4329_32k_clk", adam_bcm4329_rfkill_device.name, \
				"blink", NULL);
	printk("Initializing BT RFKILL, clk_add_alias: %d", res);
	res = platform_device_register(&adam_bcm4329_rfkill_device);
	if (res)
		printk("Error on BT RFKILL reg, result: %d", res);
	return;
}

static struct resource adam_bluesleep_resources[] = {
	[0] = {
		.name = "gpio_host_wake",
			.start  = TEGRA_GPIO_PU6,
			.end    = TEGRA_GPIO_PU6,
			.flags  = IORESOURCE_IO,
	},
	[1] = {
		.name = "host_wake",
			.start  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
			.end    = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
			.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	},
};

static struct platform_device adam_bluesleep_device = {
	.name           = "bluesleep",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(adam_bluesleep_resources),
	.resource       = adam_bluesleep_resources,
};

#ifdef CONFIG_BT_BLUEDROID
extern void bluesleep_setup_uart_port(struct platform_device *uart_dev);
#endif

void __init adam_setup_bluesleep(void)
{
	platform_device_register(&adam_bluesleep_device);
#ifdef CONFIG_BT_BLUEDROID
	bluesleep_setup_uart_port(&tegra_uartc_device);
#endif
	tegra_gpio_enable(TEGRA_GPIO_PU6);
	return;
}
