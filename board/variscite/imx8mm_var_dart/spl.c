// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018-2019 NXP
 *
 */

#include <common.h>
#include <spl.h>
#include <asm/io.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/arch/imx8mm_pins.h>
#include <asm/arch/sys_proto.h>
#include <power/pmic.h>
#include <power/bd71837.h>
#include <asm/arch/clock.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <asm/arch/ddr.h>

#include "../common/imx8_eeprom.h"

DECLARE_GLOBAL_DATA_PTR;

enum {
	DART_MX8M_MINI,
	VAR_SOM_MX8M_MINI,
	UNKNOWN_BOARD,
};

enum {
	SOM_REV10,
	SOM_REV11,
	UNKNOWN_REV,
};

extern struct dram_timing_info dram_timing_ddr4, dram_timing_lpddr4;

#define GPIO_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1 | PAD_CTL_PUE | PAD_CTL_PE)
#define ID_GPIO 	IMX_GPIO_NR(2, 11)

static iomux_v3_cfg_t const id_pads[] = {
	IMX8MM_PAD_SD1_STROBE_GPIO2_IO11 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static int get_board_id(void)
{
	static int board_id = UNKNOWN_BOARD;

	if (board_id != UNKNOWN_BOARD)
		return board_id;

	imx_iomux_v3_setup_multiple_pads(id_pads, ARRAY_SIZE(id_pads));
	gpio_request(ID_GPIO, "board_id");
	gpio_direction_input(ID_GPIO);

	board_id = gpio_get_value(ID_GPIO) ? DART_MX8M_MINI : VAR_SOM_MX8M_MINI;

	return board_id;
}

static int get_som_rev(void)
{
	struct var_eeprom eeprom = {0};
	static int som_rev = UNKNOWN_REV;

	if (som_rev != UNKNOWN_REV)
		return som_rev;

	var_eeprom_read_header(&eeprom);

	if (!var_eeprom_is_valid(&eeprom)) {
		printf("No SOM rev info in EEPROM, assuming REV1.1+\n");
		som_rev = SOM_REV11;
	}
	else if (eeprom.somrev == 0)
		som_rev = SOM_REV10;
	else
		som_rev = SOM_REV11;

	return som_rev;
}

extern struct dram_timing_info dram_timing_default;
extern struct dram_timing_info dram_timing_lpddr4_4GB_IT;

void spl_dram_init(void)
{
	int id;
	struct var_eeprom eeprom = {0};
	volatile int *sdram_test_location=(int*)0x42000000;

	id = get_board_id();
	
	switch(id)
	{
		case DART_MX8M_MINI:
			printf("DART-MX8M-MINI Detected\n");
			var_eeprom_read_header(&eeprom);
			if (!var_eeprom_is_valid(&eeprom)) {
				printf("No DRAM info in EEPROM, Trying 4GB IT default DRAM config\n");
				ddr_init(&dram_timing_lpddr4_4GB_IT);
				sdram_test_location[0]=0xAAAAAAAA;
				sdram_test_location[1]=0x55555555;
				sdram_test_location[2]=0xA5A5A5A5;
				sdram_test_location[3]=0x5A5A5A5A;
				if((sdram_test_location[0]!=0xAAAAAAAA) ||
					(sdram_test_location[1]!=0x55555555) ||
					(sdram_test_location[2]!=0xA5A5A5A5) ||
					(sdram_test_location[3]!=0x5A5A5A5A)) {
						printf("LPDDR 4GB IT not found trying 512MB default!\n");
						printf("Using defaut DRAM config\n");
						ddr_init(&dram_timing_default);
				}
			}
			else {
				var_eeprom_adjust_dram(&eeprom, &dram_timing_lpddr4);
				ddr_init(&dram_timing_lpddr4);
				sdram_test_location[0]=0xAAAAAAAA;
				sdram_test_location[1]=0x55555555;
				sdram_test_location[2]=0xA5A5A5A5;
				sdram_test_location[3]=0x5A5A5A5A;
				if((sdram_test_location[0]!=0xAAAAAAAA) ||
					(sdram_test_location[1]!=0x55555555) ||
					(sdram_test_location[2]!=0xA5A5A5A5) ||
					(sdram_test_location[3]!=0x5A5A5A5A)) {
						printf("LPDDR Test Fail!\n");
						printf("Using defaut DRAM config\n");
						ddr_init(&dram_timing_default);
				}
			}
		break;
		case VAR_SOM_MX8M_MINI:
			printf("VAR_SOM_MX8M-MINI Detected\n");
			var_eeprom_read_header(&eeprom);
			var_eeprom_adjust_dram(&eeprom, &dram_timing_ddr4);
			ddr_init(&dram_timing_ddr4);
		break;
		default:
			printf("Undefined board ID\n");
		break;
	}
}

#define USDHC_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE |PAD_CTL_PE | \
			 PAD_CTL_FSEL2)

static iomux_v3_cfg_t const usdhc3_pads[] = {
	IMX8MM_PAD_NAND_WE_B_USDHC3_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_WP_B_USDHC3_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA04_USDHC3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA05_USDHC3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA06_USDHC3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA07_USDHC3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_RE_B_USDHC3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_CE2_B_USDHC3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_CE3_B_USDHC3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_CLE_USDHC3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc2_pads[] = {
	IMX8MM_PAD_SD2_CLK_USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_CMD_USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_DATA0_USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_DATA1_USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_DATA2_USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_DATA3_USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC2_BASE_ADDR, 0, 4},
	{USDHC3_BASE_ADDR, 0, 8},
};

int board_mmc_init(bd_t *bis)
{
	int i, ret;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * mmc0                    USDHC2
	 * mmc1                    USDHC3
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			init_clk_usdhc(1);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			imx_iomux_v3_setup_multiple_pads(usdhc2_pads,
						ARRAY_SIZE(usdhc2_pads));
			break;
		case 1:
			init_clk_usdhc(2);
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) than supported by the board\n", i + 1);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret)
			return ret;
	}

	return 0;
}

int board_mmc_getcd(struct mmc *mmc)
{
	return 1;
}

#ifdef CONFIG_POWER

#define PMIC_I2C_BUS	0

int power_init_board(void)
{
	struct pmic *p;
	int ret;

	ret = power_bd71837_init(I2C_PMIC);
	if (ret)
		printf("power init failed");

	p = pmic_get("BD71837");
	pmic_probe(p);

	/* decrease RESET key long push time from the default 10s to 10ms */
	pmic_reg_write(p, BD71837_PWRONCONFIG1, 0x0);

	/* unlock the PMIC regs */
	pmic_reg_write(p, BD71837_REGLOCK, 0x1);

	/* increase VDD_SOC to typical value 0.85v before first DRAM access */
	pmic_reg_write(p, BD71837_BUCK1_VOLT_RUN, 0x0f);

	/* increase VDD_DRAM to 0.975v for 3Ghz DDR */
	pmic_reg_write(p, BD71837_BUCK5_VOLT, 0x83);

	/* increase NVCC_DRAM_1V2 to 1.2v for DDR4 */
	if (get_board_id() == VAR_SOM_MX8M_MINI)
		pmic_reg_write(p, BD71837_BUCK8_VOLT, 0x28);

	/* lock the PMIC regs */
	pmic_reg_write(p, BD71837_REGLOCK, 0x11);

	return 0;
}
#endif

void spl_board_init(void)
{
#ifndef CONFIG_SPL_USB_SDP_SUPPORT
	/* Serial download mode */
	if (is_usb_boot()) {
		puts("Back to ROM, SDP\n");
		restore_boot_params();
	}
#endif
	puts("Normal Boot\n");
}

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	int id = get_board_id();

	if ((id == DART_MX8M_MINI) && !strcmp(name, "fsl-imx8mm-var-dart"))
		return 0;
	else if (id == VAR_SOM_MX8M_MINI) {
		int rev = get_som_rev();

		if ((rev == SOM_REV11) && !strcmp(name, "fsl-imx8mm-var-som"))
			return 0;
		else if ((rev == SOM_REV10) && !strcmp(name, "fsl-imx8mm-var-som-rev10"))
			return 0;
		else
			return -1;
	}
	else
		return -1;
}
#endif

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

static iomux_v3_cfg_t const uart1_pads[] = {
	IMX8MM_PAD_UART1_RXD_UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_UART1_TXD_UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const uart4_pads[] = {
	IMX8MM_PAD_UART4_RXD_UART4_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_UART4_TXD_UART4_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

extern struct mxc_uart *mxc_base;

int board_early_init_f(void)
{
	int id;
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

	id = get_board_id();
	if (id == DART_MX8M_MINI) {
		init_uart_clk(0);
		imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	}
	else if (id == VAR_SOM_MX8M_MINI) {
		init_uart_clk(3);
		mxc_base = (struct mxc_uart *)UART4_BASE_ADDR;
		imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
	}

	return 0;
}

#define I2C_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PE)

struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = IMX8MM_PAD_I2C1_SCL_I2C1_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = IMX8MM_PAD_I2C1_SCL_GPIO5_IO14 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 14),
	},
	.sda = {
		.i2c_mode = IMX8MM_PAD_I2C1_SDA_I2C1_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = IMX8MM_PAD_I2C1_SDA_GPIO5_IO15 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 15),
	},
};

void board_init_f(ulong dummy)
{
	int ret;

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	arch_cpu_init();

	board_early_init_f();

	timer_init();

	preloader_console_init();

	ret = spl_init();
	if (ret) {
		debug("spl_init() failed: %d\n", ret);
		hang();
	}

	enable_tzc380();

	/* I2C Bus 0 initialization */
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);

	/* PMIC initialization */
	power_init_board();

	/* DDR initialization */
	spl_dram_init();

	board_init_r(NULL, 0);
}
