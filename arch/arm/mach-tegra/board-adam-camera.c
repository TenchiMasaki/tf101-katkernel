/*
 * arch/arm/mach-tegra/board-adam-camera.c
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
#include <linux/nvhost.h>

#include <media/tegra_v4l2_camera.h>
#include <media/soc_camera.h>
#include <media/s5k4cdgx.h>

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

int adam_s5k4cdgx_set_power(int enable);

#define S5K4CDGX_POWER_PIN TEGRA_GPIO_PBB5
#define S5K4CDGX_RESET_PIN TEGRA_GPIO_PD2

// TODO: clean these up into a common header
#define S5K4CDGX_MCLK_FREQ 24000000

static int clink_s5k4cdgx_set_power(struct device *dev, int power_on) {
  return adam_s5k4cdgx_set_power(power_on);
}

struct s5k4cdgx_platform_data adam_s5k4cdgx_data = {
	.mclk_frequency = S5K4CDGX_MCLK_FREQ,
	.bus_type = V4L2_MBUS_PARALLEL,
	.gpio_stby = { 
		.gpio = S5K4CDGX_POWER_PIN, 
		.level = 0, // active-low
	},
	.gpio_reset = { 
		.gpio = S5K4CDGX_RESET_PIN,
		.level = 0, // active-low
	},
	.nlanes = 1,
	.horiz_flip = false,
	.vert_flip = false,
};

struct s5k4cdgx_platform_data adam_s5k4cdgx_pdata = {
	.set_power = &adam_s5k4cdgx_set_power,
};

static struct i2c_board_info adam_i2c3_board_info_camera = {
    I2C_BOARD_INFO("S5K4CDGX",  0x78>>1),
  // platform data will get overwritten here with soc_camera_device
};

static struct soc_camera_link clink_s5k4cdgx = {
  .board_info = &adam_i2c3_board_info_camera,
  .i2c_adapter_id = 3,
  .power = &clink_s5k4cdgx_set_power,
  .priv = &adam_s5k4cdgx_data,
  // TODO: move regulators here
};

static struct platform_device adam_tegra_s5k4cdgx_device = {
  .name   = "soc-camera-pdrv",
  .id     = 0,
  .dev    = {
    .platform_data = &clink_s5k4cdgx,
  },
};

static struct platform_device tegra_camera_power_device = {
  // note the underscore
  .name   = "tegra_camera",
  .id     = 0,
};


/* In theory we might want to use this callback to reference the 
   tegra_camera driver from the soc_camera host driver instead of
   the i2c client driver */
static int adam_enable_camera(struct nvhost_device *ndev)
{
	struct soc_camera_host *ici = to_soc_camera_host(&ndev->dev);

	dev_dbg(&ndev->dev, "%s\n", __func__);

	return 0;
}

static void adam_disable_camera(struct nvhost_device *ndev)
{
	dev_dbg(&ndev->dev, "%s\n", __func__);
}

static struct tegra_camera_platform_data adam_camera_pdata = {
  .enable_camera = &adam_enable_camera,
  .disable_camera = &adam_disable_camera,
  .flip_h = 0,
  .flip_v = 0,
};

int __init adam_camera_register_devices(void)
{
  int ret;

  tegra_camera_device.dev.platform_data = &adam_camera_pdata;

  ret = platform_device_register(&tegra_camera_power_device);
  if(ret)
    return ret;

  ret = platform_device_register(&adam_tegra_s5k4cdgx_device);
  if(ret)
    return ret;

  ret = nvhost_device_register(&tegra_camera_device);
  if(ret)
    return ret;

  return 0;
}
