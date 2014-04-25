/*
 * Copyright (C) 2010-2011 NVIDIA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
#include <linux/i2c.h>
#include <linux/pda_power.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/tps6586x.h>
#include <linux/gpio.h>
#include <linux/io.h>

#include <mach/iomap.h>
#include <mach/irqs.h>

#include <generated/mach-types.h>
#include <linux/err.h>

#include "gpio-names.h"
#include "fuse.h"
#include "pm.h"
#include "wakeups-t2.h"
#include "board.h"
#include "board-adam.h"

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)

int __init adam_charge_init(void)
{
	gpio_request(ADAM_CHARGE_EN, "chg_enable");
	gpio_direction_output(ADAM_CHARGE_EN, 1);
	tegra_gpio_enable(ADAM_CHARGE_EN);
	return 0;
}

static struct regulator_consumer_supply tps658621_sm0_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};
static struct regulator_consumer_supply tps658621_sm1_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};
static struct regulator_consumer_supply tps658621_sm2_supply[] = {
	REGULATOR_SUPPLY("vdd_sm2", NULL),
};
static struct regulator_consumer_supply tps658621_ldo0_supply[] = {
	REGULATOR_SUPPLY("vdd_ldo0", NULL),
	REGULATOR_SUPPLY("p_cam_avdd", NULL),
};
static struct regulator_consumer_supply tps658621_ldo1_supply[] = {
	REGULATOR_SUPPLY("vdd_ldo1", NULL),
	REGULATOR_SUPPLY("avdd_pll", NULL),
};
static struct regulator_consumer_supply tps658621_ldo2_supply[] = {
	REGULATOR_SUPPLY("vdd_ldo2", NULL),
	REGULATOR_SUPPLY("vdd_rtc", NULL),
	REGULATOR_SUPPLY("vdd_aon", NULL),
};
static struct regulator_consumer_supply tps658621_ldo3_supply[] = {
	REGULATOR_SUPPLY("vdd_ldo3", NULL),
	REGULATOR_SUPPLY("avdd_usb", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
	REGULATOR_SUPPLY("vmmc", NULL),
};
static struct regulator_consumer_supply tps658621_ldo4_supply[] = {
	REGULATOR_SUPPLY("vdd_ldo4", NULL),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
};
static struct regulator_consumer_supply tps658621_ldo5_supply[] = {
	REGULATOR_SUPPLY("vdd_ldo5", NULL),
};
static struct regulator_consumer_supply tps658621_ldo6_supply[] = {
	REGULATOR_SUPPLY("vdd_ldo6", NULL),
//	REGULATOR_SUPPLY("vcsi", "tegra_camera"),
	REGULATOR_SUPPLY("avdd_vdac", NULL),
};
static struct regulator_consumer_supply tps658621_ldo7_supply[] = {
	REGULATOR_SUPPLY("vdd_ldo7", NULL),
	REGULATOR_SUPPLY("avdd_hdmi", NULL),
	REGULATOR_SUPPLY("vdd_fuse", NULL),
};
static struct regulator_consumer_supply tps658621_ldo8_supply[] = {
	REGULATOR_SUPPLY("vdd_ldo8", NULL),
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
};
static struct regulator_consumer_supply tps658621_ldo9_supply[] = {
	REGULATOR_SUPPLY("vdd_ldo9", NULL),
	REGULATOR_SUPPLY("avdd_2v85", NULL),
	REGULATOR_SUPPLY("vdd_ddr_rx", NULL),
	REGULATOR_SUPPLY("vddio_vi", NULL),
	REGULATOR_SUPPLY("vcsi",NULL),
};

/* Super power voltage rail for the SOC : VDD SOC
*/
static struct regulator_consumer_supply tps658621_soc_supply[] = {
	REGULATOR_SUPPLY("vdd_soc", NULL)
};

/* PLLE voltage rail : AVDD_PLLE -> VDD_1V05
   PEX_CLK voltage rail : AVDD_PLLE -> VDD_1V05
*/
static struct regulator_consumer_supply fixed_buck_tps62290_supply[] = {
	REGULATOR_SUPPLY("avdd_plle", NULL),
};

/* MIPI voltage rail (DSI_CSI): AVDD_DSI_CSI -> VDD_1V2
   Wlan : VCORE_WIFI (VDD_1V2)
*/
static struct regulator_consumer_supply fixed_ldo_tps72012_supply[] = {
	REGULATOR_SUPPLY("avdd_dsi_csi", NULL),
	REGULATOR_SUPPLY("vcore_wifi", NULL)
};

/* PEX_CLK voltage rail : PMU_GPIO-1 -> VDD_1V5
*/
static struct regulator_consumer_supply fixed_ldo_tps74201_supply[] = {
	REGULATOR_SUPPLY("vdd_pex_clk_2", NULL),
};

/* HDMI +5V for the pull-up for DDC : VDDIO_VID
   HDMI +5V for hotplug
   lcd rail (required for crt out) : VDDIO_VGA
*/
static struct regulator_consumer_supply fixed_ldo_tps2051B_supply[] = {
	REGULATOR_SUPPLY("vddio_vid", NULL),
	REGULATOR_SUPPLY("vddio_vga", NULL),
	REGULATOR_SUPPLY("vdd_camera", NULL),
};

static struct tps6586x_settings sm0_config = {
	.sm_pwm_mode = PWM_DEFAULT_VALUE,
	.slew_rate = SLEW_RATE_3520UV_PER_SEC,
};

static struct tps6586x_settings sm1_config = {
	/*
	 * Current TPS6586x is known for having a voltage glitch if current load
	 * changes from low to high in auto PWM/PFM mode for CPU's Vdd line.
	 */
	.sm_pwm_mode = PWM_ONLY,
	.slew_rate = SLEW_RATE_3520UV_PER_SEC,
};

#define REGULATOR_INIT(_id, _minmv, _maxmv, on, config)			\
	{								\
		.constraints = {					\
			.min_uV = (_minmv)*1000,			\
			.max_uV = (_maxmv)*1000,			\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					     REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					   REGULATOR_CHANGE_STATUS |	\
					   REGULATOR_CHANGE_VOLTAGE),	\
			.always_on = on,				\
			.apply_uV = 1,					\
		},							\
		.num_consumer_supplies = ARRAY_SIZE(tps658621_##_id##_supply),\
		.consumer_supplies = tps658621_##_id##_supply,		\
		.driver_data = config,					\
	}

#define FIXED_REGULATOR_INIT(_id, _mv, _aon, _bon)		\
	{													\
		.constraints = {								\
			.name = #_id,								\
			.min_uV = (_mv)*1000,						\
			.max_uV = (_mv)*1000,						\
			.valid_modes_mask = REGULATOR_MODE_NORMAL,	\
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,	\
			.always_on	= _aon, 						\
			.boot_on	= _bon, 						\
			},												\
			.num_consumer_supplies = ARRAY_SIZE( fixed_##_id##_supply),\
		.consumer_supplies = fixed_##_id##_supply,		\
	}


#define ON	1
#define OFF	0

static struct regulator_init_data sm0_data = REGULATOR_INIT(sm0, 725, 1500, ON, &sm0_config);
static struct regulator_init_data sm1_data = REGULATOR_INIT(sm1, 725, 1500, ON, &sm1_config);
static struct regulator_init_data sm2_data = REGULATOR_INIT(sm2, 3000, 4550, ON, NULL);
static struct regulator_init_data ldo0_data = REGULATOR_INIT(ldo0, 1250, 3300, OFF, NULL);
static struct regulator_init_data ldo1_data = REGULATOR_INIT(ldo1, 725, 1500, ON, NULL);
static struct regulator_init_data ldo2_data = REGULATOR_INIT(ldo2, 725, 1500, OFF, NULL);
static struct regulator_init_data ldo3_data = REGULATOR_INIT(ldo3, 1250, 3300, OFF, NULL);
static struct regulator_init_data ldo4_data = REGULATOR_INIT(ldo4, 1700, 2475, ON, NULL);
static struct regulator_init_data ldo5_data = REGULATOR_INIT(ldo5, 1250, 3300, ON, NULL);
static struct regulator_init_data ldo6_data = REGULATOR_INIT(ldo6, 1800, 1800, OFF, NULL);
static struct regulator_init_data ldo7_data = REGULATOR_INIT(ldo7, 1250, 3300, OFF, NULL);
static struct regulator_init_data ldo8_data = REGULATOR_INIT(ldo8, 1250, 3300, OFF, NULL);
static struct regulator_init_data ldo9_data = REGULATOR_INIT(ldo9, 1250, 3300, OFF, NULL);

static struct regulator_init_data soc_data = REGULATOR_INIT(soc, 1250, 3300, ON, NULL);

static struct regulator_init_data ldo_tps74201_data  
	= FIXED_REGULATOR_INIT(ldo_tps74201 , 1500, 0, 0 ); // 1500 (VDD1.5, enabled by PMU_GPIO[0] (0=enabled) - Turn it off as soon as we boot
static struct regulator_init_data buck_tps62290_data 
	= FIXED_REGULATOR_INIT(buck_tps62290, 1050, 1, 1 ); // 1050 (VDD1.05, AVDD_PEX ... enabled by PMU_GPIO[2] (1=enabled)
static struct regulator_init_data ldo_tps72012_data  
	= FIXED_REGULATOR_INIT(ldo_tps72012 , 1200, 0, 0 ); // 1200 (VDD1.2, VCORE_WIFI ...) enabled by PMU_GPIO[1] (1=enabled)
static struct regulator_init_data ldo_tps2051B_data  
	= FIXED_REGULATOR_INIT(ldo_tps2051B , 5000, 1, 1 ); // 5000 (VDDIO_VID), enabled by AP_GPIO Port T, pin2, 
														// (set as input to enable,outpul low to disable). Powers HDMI.
														// Wait 500uS to let it stabilize before returning . Probably also 
														// used for USB host. It should always be kept enabled. Force enabling
														// it at boot.


#define FIXED_REGULATOR_CONFIG(_id, _mv, _gpio, _activehigh, _itoen, _delay, _atboot, _data)	\
	{												\
		.supply_name 	= #_id,						\
		.microvolts  	= (_mv)*1000,				\
		.gpio        	= _gpio,					\
		.enable_high	= _activehigh,				\
		.startup_delay	= _delay,					\
		.enabled_at_boot= _atboot,					\
		.init_data		= &_data,					\
	}
		//.set_as_input_to_enable = _itoen,			\
/* The next 3 are fixed regulators controlled by PMU GPIOs */
static struct fixed_voltage_config ldo_tps74201_cfg  
	= FIXED_REGULATOR_CONFIG(ldo_tps74201  , 1500, PMU_GPIO0 , 0,0, 200000, 0, ldo_tps74201_data);
static struct fixed_voltage_config buck_tps62290_cfg
	= FIXED_REGULATOR_CONFIG(buck_tps62290 , 1050, PMU_GPIO2 , 1,0, 200000, 1, buck_tps62290_data);
static struct fixed_voltage_config ldo_tps72012_cfg
	= FIXED_REGULATOR_CONFIG(ldo_tps72012  , 1200, PMU_GPIO1 , 1,0, 200000, 1, ldo_tps72012_data);

/* the next one is controlled by a general purpose GPIO */
static struct fixed_voltage_config ldo_tps2051B_cfg
	= FIXED_REGULATOR_CONFIG(ldo_tps2051B  , 5000, ADAM_ENABLE_VDD_VID	, 1,1, 500000, 0, ldo_tps2051B_data);

static struct tps6586x_rtc_platform_data rtc_data = {
	.irq = TEGRA_NR_IRQS + TPS6586X_INT_RTC_ALM1,
	.start = {
		.year = 2009,
		.month = 1,
		.day = 1,
	},
	.cl_sel = TPS6586X_RTC_CL_SEL_1_5PF /* use lowest (external 20pF cap) */
};

#define TPS_REG(_id, _data)			\
	{					\
		.id = TPS6586X_ID_##_id,	\
		.name = "tps6586x-regulator",	\
		.platform_data = _data,		\
	}

static struct tps6586x_subdev_info tps_devs[] = {
	TPS_REG(SM_0, &sm0_data),
	TPS_REG(SM_1, &sm1_data),
	TPS_REG(SM_2, &sm2_data),
	TPS_REG(LDO_0, &ldo0_data),
	TPS_REG(LDO_1, &ldo1_data),
	TPS_REG(LDO_2, &ldo2_data),
	TPS_REG(LDO_3, &ldo3_data),
	TPS_REG(LDO_4, &ldo4_data),
	TPS_REG(LDO_5, &ldo5_data),
	TPS_REG(LDO_6, &ldo6_data),
	TPS_REG(LDO_7, &ldo7_data),
	TPS_REG(LDO_8, &ldo8_data),
	TPS_REG(LDO_9, &ldo9_data),
	{
		.id	= 0,
		.name	= "tps6586x-rtc",
		.platform_data = &rtc_data,
	},
};

static struct tps6586x_platform_data tps_platform = {
	.irq_base = TPS6586X_INT_BASE,
	.num_subdevs = ARRAY_SIZE(tps_devs),
	.subdevs = tps_devs,
	.gpio_base = PMU_GPIO_BASE,
};

static struct i2c_board_info __initdata adam_regulators[] = {
	{
		I2C_BOARD_INFO("tps6586x", 0x34),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &tps_platform,
	},
};

#define GPIO_FIXED_REG(_id,_data)		\
	{									\
		.name = "reg-fixed-voltage",	\
		.id = _id,						\
		.dev = { 						\
			.platform_data = & _data,	\
		}, 								\
	} 	

static struct platform_device adam_ldo_tps2051B_reg_device = 
	GPIO_FIXED_REG(3,ldo_tps2051B_cfg); /* id is 3, because 0-2 are already used in the PMU gpio controlled fixed regulators */

static void adam_board_suspend(int lp_state, enum suspend_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_SUSPEND_BEFORE_CPU))
		tegra_console_uart_suspend();
}

static void adam_board_resume(int lp_state, enum resume_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_RESUME_AFTER_CPU))
		tegra_console_uart_resume();
}

static struct tegra_suspend_platform_data adam_suspend_data = {
	/*
	 * Check power on time and crystal oscillator start time
	 * for appropriate settings.
	 */
	.cpu_timer	= 2000,
	.cpu_off_timer	= 100,
	.suspend_mode	= TEGRA_SUSPEND_LP1,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0xf,
	.corereq_high	= false,
	.sysclkreq_high	= true,
	.board_suspend = adam_board_suspend,
	.board_resume = adam_board_resume,
};

static void reg_off(const char *reg)
{
	int rc;
	struct regulator *regulator;

	regulator = regulator_get(NULL, reg);

	if (IS_ERR(regulator)) {
		pr_err("%s: regulator_get returned %ld\n", __func__,
		       PTR_ERR(regulator));
		return;
	}

	/* force disabling of regulator to turn off system */
	rc = regulator_force_disable(regulator);
	if (rc)
		pr_err("%s: regulator_disable returned %d\n", __func__, rc);
	regulator_put(regulator);
}

static void adam_power_off(void)
{
	/* Power down through NvEC */
	//nvec_poweroff();

	/* Turn off main supply */
	tps6586x_power_off();

	/* Then try by powering off supplies */
	reg_off("vdd_sm2");
	reg_off("vdd_core");
	reg_off("vdd_cpu");
	reg_off("vdd_soc");
	local_irq_disable();
	while (1) {
		dsb();
		__asm__ ("wfi");
	}
}

int __init adam_power_register_devices(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	void __iomem *chip_id = IO_ADDRESS(TEGRA_APB_MISC_BASE) + 0x804;
	u32 pmc_ctrl;
	u32 minor;

	minor = (readl(chip_id) >> 16) & 0xf;
	/* A03 (but not A03p) chips do not support LP0 */
	if (minor == 3 && !(tegra_spare_fuse(18) || tegra_spare_fuse(19)))
		adam_suspend_data.suspend_mode = TEGRA_SUSPEND_LP1;

	/* configure the power management controller to trigger PMU
	 * interrupts when low */
	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	/* register the poweroff callback */
	pm_power_off = adam_power_off;	

	i2c_register_board_info(4, adam_regulators, 1);

	//regulator_has_full_constraints();

	tegra_init_suspend(&adam_suspend_data);

	return 0;
}
