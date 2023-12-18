// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022 NXP
 * Copyright 2023 Variscite Ltd.
 */

#include <common.h>
#include <command.h>
#include <cpu_func.h>
#include <hang.h>
#include <image.h>
#include <init.h>
#include <log.h>
#include <spl.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/arch/imx93_pins.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch-mx7ulp/gpio.h>
#include <asm/mach-imx/syscounter.h>
#include <asm/mach-imx/ele_api.h>
#include <dm/uclass.h>
#include <dm/device.h>
#include <dm/uclass-internal.h>
#include <dm/device-internal.h>
#include <linux/delay.h>
#include <asm/arch/clock.h>
#include <asm/arch/ccm_regs.h>
#include <asm/arch/ddr.h>
#include <power/pmic.h>
#include <power/pca9450.h>
#include <asm/arch/trdc.h>

#include "../common/imx9_eeprom.h"

DECLARE_GLOBAL_DATA_PTR;

static struct var_eeprom eeprom = {0};

int spl_board_boot_device(enum boot_device boot_dev_spl)
{
	return BOOT_DEVICE_BOOTROM;
}

void spl_board_init(void)
{
	struct var_eeprom *ep = VAR_EEPROM_DATA;
	int ret;

	puts("Normal Boot\n");

	ret = ahab_start_rng();
	if (ret)
		printf("Fail to start RNG: %d\n", ret);

	/* Copy EEPROM contents to DRAM */
	memcpy(ep, &eeprom, sizeof(*ep));
}

void spl_dram_init(void)
{
	/* EEPROM initialization */
	var_eeprom_read_header(&eeprom);

	puts("Do you want to erase EEPROM?[Y/N]\n");
	mdelay(1000);
	if(tstc()!=0)
		if(getchar()=='Y')
		{
			printf("Erasing EEPROM\n");
			memset(&eeprom,0xFF,0xFF);
		}

	var_eeprom_adjust_dram(&eeprom, &dram_timing);
	ddr_init(&dram_timing);
}

#if CONFIG_IS_ENABLED(DM_PMIC_PCA9450)
int power_init_board(void)
{
	struct udevice *dev;
	int ret;
	unsigned int val = 0;

	ret = pmic_get("pmic@25", &dev);
	if (ret == -ENODEV) {
		puts("No pca9450@25\n");
		return 0;
	}
	if (ret != 0)
		return ret;

	/* BUCKxOUT_DVS0/1 control BUCK123 output */
	pmic_reg_write(dev, PCA9450_BUCK123_DVS, 0x29);

	/* enable DVS control through PMIC_STBY_REQ */
	pmic_reg_write(dev, PCA9450_BUCK1CTRL, 0x59);

	ret = pmic_reg_read(dev, PCA9450_PWR_CTRL);
	if (ret < 0)
		return ret;
	else
		val = ret;

	/* 0.9v for Over drive mode
		*/
	if (val & PCA9450_REG_PWRCTRL_TOFF_DEB) {
		pmic_reg_write(dev, PCA9450_BUCK1OUT_DVS0, 0x14);
		pmic_reg_write(dev, PCA9450_BUCK3OUT_DVS0, 0x14);
	} else {
		pmic_reg_write(dev, PCA9450_BUCK1OUT_DVS0, 0x18);
		pmic_reg_write(dev, PCA9450_BUCK3OUT_DVS0, 0x18);
	}

	/* set standby voltage to 0.65v */
	if (val & PCA9450_REG_PWRCTRL_TOFF_DEB)
		pmic_reg_write(dev, PCA9450_BUCK1OUT_DVS1, 0x0);
	else
		pmic_reg_write(dev, PCA9450_BUCK1OUT_DVS1, 0x4);

	/* I2C_LT_EN*/
	pmic_reg_write(dev, 0xa, 0x3);

	/* Enable load switch for ETH_3V3 */
	pmic_clrsetbits(dev, PCA9450_LOADSW_CTRL, 0, BIT(1) | BIT(0));

	/* Disable WDOG and System resets */
	pmic_reg_write(dev, PCA9450_RESET_CTRL, 0x01);

	return 0;
}
#endif

extern int imx9_probe_mu(void *ctx, struct event *event);
void board_init_f(ulong dummy)
{
	int ret;

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	timer_init();

	arch_cpu_init();

	board_early_init_f();

	spl_early_init();

	preloader_console_init();

	ret = imx9_probe_mu(NULL, NULL);
	if (ret) {
		printf("Fail to init ELE API\n");
	} else {
		printf("SOC: 0x%x\n", gd->arch.soc_rev);
		printf("LC: 0x%x\n", gd->arch.lifecycle);
	}
	power_init_board();

	set_arm_core_max_clk();

	/* Init power of mix */
	soc_power_init();

	/* Setup TRDC for DDR access */
	trdc_init();

	printf("Starting DRAM Init\n");
	/* DDR initialization */
	spl_dram_init();

	/* Put M33 into CPUWAIT for following kick */
	ret = m33_prepare();
	if (!ret)
		printf("M33 prepare ok\n");

	board_init_r(NULL, 0);
}
