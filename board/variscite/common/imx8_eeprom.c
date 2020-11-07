/*
 * Copyright (C) 2018-2019 Variscite Ltd.
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <common.h>
#include <command.h>
#include <dm.h>
#include <i2c.h>
#include <asm/io.h>

#ifdef CONFIG_ARCH_IMX8M
#include <asm/arch/imx8m_ddr.h>
#endif

#ifdef CONFIG_ARCH_IMX8
#include <asm/mach-imx/sci/sci.h>
#endif

#include "imx8_eeprom.h"

#ifdef CONFIG_ARCH_IMX8

DECLARE_GLOBAL_DATA_PTR;

#define CTL_CODE(function, method) ((4 << 16) | ((function) << 2) | (method))

#define METHOD_BUFFERED		0
#define METHOD_NEITHER		3

#define SOMINFO_READ_EEPROM	CTL_CODE(2100, METHOD_BUFFERED)
#define SOMINFO_WRITE_EEPROM	CTL_CODE(2101, METHOD_BUFFERED)

static int var_scu_eeprom_read(uint8_t *buf, uint32_t size)
{
	uint32_t command;
	sc_err_t ret = 0;
	sc_ipc_t ipc_handle;

	/* Open IPC channel */
	ret = sc_ipc_open(&ipc_handle, SC_IPC_CH);
	if (ret != SC_ERR_NONE) {
		printf("sc_ipc_open failed\n");
		return -EPERM;
	}

	command = SOMINFO_READ_EEPROM;

	/* Send command to SC firmware */
	memset(buf, 0, size);
	flush_dcache_all();
	invalidate_icache_all();
	ret = sc_misc_board_ioctl(ipc_handle, &command, (uint32_t *)&buf, &size);

	/* Close IPC channel */
	sc_ipc_close(ipc_handle);

	flush_dcache_all();
	invalidate_icache_all();

	return ret;
}

int var_scu_eeprom_read_header(struct var_eeprom *e)
{
	int ret;

	ret = var_scu_eeprom_read((uint8_t *)e, sizeof(struct var_eeprom));
	if (ret) {
		debug("SCU EEPROM read failed\n");
		return ret;
	}

	return 0;
}

int var_scu_eeprom_write(uint8_t *buf, uint32_t size)
{
	uint32_t command;
	sc_ipc_t ipc_handle = gd->arch.ipc_channel_handle;

	command = SOMINFO_WRITE_EEPROM;

	flush_dcache_all();
	invalidate_icache_all();

	return sc_misc_board_ioctl(ipc_handle, &command, (uint32_t *)&buf, &size);
}

#define CHAR_BIT 8

static int var_eeprom_set_mac(struct var_eeprom *e, char *value)
{
	int8_t i;
	uint64_t mac;
	char *endptr;
	uint8_t *p = e->mac;

	mac = simple_strtoul(value, &endptr, 16);
	if ((endptr == value) || (*endptr != '\0')) {
		printf("Invalid MAC\n");
		return -1;
	}

	for (i = 5; i >= 0; i--) {
		*p++ = mac >> (CHAR_BIT * i);
	}

	return 0;
}

static void remove_spaces(char *str1, char *str2)
{
	while (*str1) {
		if (*str1 == ' ')
			str1++;
		else
			*str2++ = *str1++;
	}
}

static char *find_part(const char *fullpartnum)
{
	char *first, *second;

	first = strchr(fullpartnum, '-');
	if (first) {
		second = strchr(first + 1, '-');
		if (second)
			return second + 1;
	}

	return NULL;
}

static int do_spear_eeprom(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret;
	char date[10];
	char *partnum;
	int partnum_len;
	uint8_t som_features;
	uint8_t som_revision;
	struct var_eeprom e;

	if (argc != 7)
		return CMD_RET_USAGE;

	ret = var_scu_eeprom_read((uint8_t *)&e, sizeof(e));
	if (ret) {
		printf("EEPROM read failed ret=%d\n", ret);
		return ret;
	}

	printf("\nThe following data will be written to EEPROM:\n");
	printf("\nPart number: \t\t%s\n", argv[1]);
	printf("Assembly: \t\t%s\n", argv[2]);
	printf("Production date: \t%s\n", argv[3]);
	printf("SOM features: \t\t%s\n", argv[4]);
	printf("SOM revision: \t\t%s\n", argv[5]);
	printf("MAC address: \t\t%s\n", argv[6]);

	memset(date, 0, sizeof(date));
	remove_spaces(argv[3], date);

	partnum = find_part(argv[1]);
	partnum_len = strlen(partnum);

	if (!partnum || partnum_len < sizeof(e.partnum)) {
		printf("Invalid P/N\n");
		return -1;
	}

	memcpy(e.partnum, partnum, sizeof(e.partnum));

	if ((partnum_len - sizeof(e.partnum)) < sizeof(e.partnum2))
		partnum_len = partnum_len - sizeof(e.partnum);
	else
		partnum_len = sizeof(e.partnum2);

	memset(e.partnum2, 0, sizeof(e.partnum2));
	memcpy(e.partnum2, partnum + sizeof(e.partnum), partnum_len);

	som_features = (uint8_t)simple_strtoul(argv[4], NULL, 16);
	som_revision = (uint8_t)simple_strtoul(argv[5], NULL, 16);

	e.somrev = som_revision;
	e.features = som_features;

	memcpy(e.assembly, argv[2] + 2, sizeof(e.assembly));
	memcpy(e.date, date, sizeof(e.date));
	ret = var_eeprom_set_mac(&e, argv[6]);
	if (ret)
		return ret;

	e.magic = swab16(VAR_EEPROM_MAGIC);

	ret = var_scu_eeprom_write((uint8_t *)&e, sizeof(e));
	if (ret) {
		printf("EEPROM write failed ret=%d\n", ret);
		return ret;
	}

	printf("\nEEPROM updated successfully\n");

	return 0;
}

static int do_eeprom_write(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret;
	uintptr_t addr;
	uint32_t size;

	if (argc != 3)
		return CMD_RET_USAGE;

	addr = simple_strtol(argv[1], NULL, 16);
	size = simple_strtol(argv[2], NULL, 16);

	ret = var_scu_eeprom_write((uint8_t *)addr, size);
	if (ret) {
		printf("EEPROM write failed ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static int do_eeprom_read(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret;
	uintptr_t addr;
	uint32_t size;

	if (argc != 3)
		return CMD_RET_USAGE;

	addr = simple_strtol(argv[1], NULL, 16);
	size = simple_strtol(argv[2], NULL, 16);

	ret = var_scu_eeprom_read((uint8_t *)addr, size);
	if (ret) {
		printf("EEPROM read failed ret=%d\n", ret);
		return ret;
	}

	return 0;
}


U_BOOT_CMD(spear_eeprom, 11, 1, do_spear_eeprom, "Update SPEAR-MX8 EEPROM",
		"<part-number> <assembly> <date> <som-conf> <som-rev> <mac>\n"
		"       - Update SPEAR-MX8 EEPROM"
)

U_BOOT_CMD(eeprom_write, 11, 1, do_eeprom_write, "Copy data from memory to EEPROM",
		"<address> <size> \n"
		"       - Copy data from memory to EEPROM"
)

U_BOOT_CMD(eeprom_read, 11, 1, do_eeprom_read, "Copy data from EEPROM to memory",
		"<address> <size> \n"
		"       - Copy data from EEPROM to memory"
)

#endif

#ifdef CONFIG_DM_I2C
static struct udevice *var_eeprom_init(void)
{
	int ret;
	struct udevice *bus, *dev;

	ret = uclass_get_device_by_seq(UCLASS_I2C, VAR_EEPROM_I2C_BUS, &bus);
	if (ret) {
		debug("%s: No bus %d\n", __func__, VAR_EEPROM_I2C_BUS);
		return NULL;
	}

	ret = dm_i2c_probe(bus, VAR_EEPROM_I2C_ADDR, 0, &dev);
	if (ret) {
		debug("%s: Can't find device id=0x%x, on bus %d\n",
			__func__, VAR_EEPROM_I2C_ADDR, VAR_EEPROM_I2C_BUS);
		return NULL;
	}

	return dev;
}

int var_eeprom_read_header(struct var_eeprom *e)
{
	int ret;
	struct udevice *edev;

	edev = var_eeprom_init();
	if (!edev) {
#ifdef CONFIG_ARCH_IMX8
		debug("var_eeprom_read_header: calling SCU to read EEPROM\n");
		return var_scu_eeprom_read_header(e);
#else
		debug("var_eeprom_read_header: I2C init failed\n");
		return -1;
#endif
	}

	/* Read EEPROM to memory */
	ret = dm_i2c_read(edev, 0, (void *)e, sizeof(*e));
	if (ret) {
		debug("EEPROM read failed, ret=%d\n", ret);
		return ret;
	}

	return 0;
}
#else
int var_eeprom_read_header(struct var_eeprom *e)
{
	int ret;

	/* Probe EEPROM */
	i2c_set_bus_num(VAR_EEPROM_I2C_BUS);
	ret = i2c_probe(VAR_EEPROM_I2C_ADDR);
	if (ret) {
#ifdef CONFIG_ARCH_IMX8
		debug("var_eeprom_read_header: calling SCU to read EEPROM\n");
		return var_scu_eeprom_read_header(e);
#else
		printf("EEPROM init failed\n");
		return -1;
#endif
	}

	/* Read EEPROM header to memory */
	ret = i2c_read(VAR_EEPROM_I2C_ADDR, 0, 1, (uint8_t *)e, sizeof(*e));
	if (ret) {
		printf("EEPROM read failed ret=%d\n", ret);
		return -1;
	}

	return 0;
}
#endif

int var_eeprom_get_mac(struct var_eeprom *e, u8 *buf)
{
	if (!var_eeprom_is_valid(e))
		return -1;

	memcpy(buf, e->mac, sizeof(e->mac));

	return 0;
}

int var_eeprom_get_dram_size(struct var_eeprom *e, u32 *size)
{
	u8 dramsize;

	if (!var_eeprom_is_valid(e))
		return -1;

	memcpy(&dramsize, (void *)&e->dramsize, sizeof(e->dramsize));

	if (e->version == 1)
		*size = dramsize * 1024;
	else
		*size = dramsize * 128;

	return 0;
}

void var_eeprom_print_prod_info(struct var_eeprom *e)
{
	u8 partnum[8] = {0};

	if (!var_eeprom_is_valid(e))
		return;

	memcpy(partnum, e->partnum, sizeof(e->partnum));

        /* Read second part of P/N  */
        if (e->version >= 3)
                memcpy(partnum + sizeof(e->partnum), e->partnum2, sizeof(e->partnum2));

#ifdef CONFIG_TARGET_IMX8M_VAR_DART
	printf("\nPart number: VSM-DT8M-%.*s\n", (int)sizeof(partnum), partnum);
#elif CONFIG_TARGET_IMX8MM_VAR_DART
	printf("\nPart number: VSM-DT8MM-%.*s\n", (int)sizeof(partnum), partnum);
#elif CONFIG_TARGET_IMX8QXP_VAR_SOM
	printf("\nPart number: VSM-MX8X-%.*s\n", (int)sizeof(partnum), partnum);
#elif CONFIG_TARGET_IMX8QM_VAR_SOM
	if (e->somrev & 0x40)
		printf("\nPart number: VSM-SP8-%.*s\n", (int)sizeof(partnum), partnum);
	else
		printf("\nPart number: VSM-MX8-%.*s\n", (int)sizeof(partnum), partnum);
#endif
	printf("Assembly: AS%.*s\n", (int)sizeof(e->assembly), (char *)e->assembly);

	printf("Production date: %.*s %.*s %.*s\n",
			4, /* YYYY */
			(char *)e->date,
			3, /* MMM */
			((char *)e->date) + 4,
			2, /* DD */
			((char *)e->date) + 4 + 3);

	printf("Serial Number: %02x:%02x:%02x:%02x:%02x:%02x\n",
		e->mac[0], e->mac[1], e->mac[2], e->mac[3], e->mac[4], e->mac[5]);

	debug("EEPROM version: 0x%x\n", e->version);
	debug("SOM features: 0x%x\n", e->features);
	debug("SOM revision: 0x%x\n", e->somrev);

	if (e->version == 1)
		debug("DRAM size: %d GiB\n\n", e->dramsize);
	else
		debug("DRAM size: %d GiB\n\n", (e->dramsize * 128) / 1024);
}

#if defined(CONFIG_ARCH_IMX8M) && defined(CONFIG_SPL_BUILD)
/*
 * Modify DRAM table based on adjustment table in EEPROM
 *
 * Assumption: register addresses in the adjustment table
 * follow the order of register addresses in the original table
 *
 * @adj_table_offset - offset of adjustment table from start of EEPROM
 * @adj_table_size   - number of rows in adjustment table
 * @table            - pointer to DDR table
 * @table_size       - number of rows in DDR table
 */
static void adjust_dram_table(u8 adj_table_offset, u8 adj_table_size,
				struct dram_cfg_param *table, u8 table_size)
{
	int i, j = 0;
	u8 off = adj_table_offset;
	struct dram_cfg_param adj_table_row;

	/* Iterate over adjustment table */
	for (i = 0; i < adj_table_size; i++) {

		/* Read next entry from adjustment table */
		i2c_read(VAR_EEPROM_I2C_ADDR, off, 1,
			(uint8_t *)&adj_table_row, sizeof(adj_table_row));

		/* Iterate over DDR table and adjust it */
		for (; j < table_size; j++) {
			if (table[j].reg == adj_table_row.reg) {
				debug("Adjusting reg=0x%x val=0x%x\n",
					adj_table_row.reg, adj_table_row.val);
				table[j].val = adj_table_row.val;
				break;
			}
		}

		off += sizeof(adj_table_row);
	}
}

/*
 * Modify DRAM tables based on adjustment tables in EEPROM
 *
 * @e - pointer to EEPROM header structure
 * @d - pointer to DRAM configuration structure
  */
void var_eeprom_adjust_dram(struct var_eeprom *e, struct dram_timing_info *d)
{
	int i;
	int *idx_map;
	u8 adj_table_size[DRAM_TABLE_NUM];

	/* Indices of fsp tables in the offset table */
	int b0_idx_map[] = {3, 4, 6};
	int b1_idx_map[] = {3, 4, 5, 6};

	/* Check EEPROM validity */
	if (!var_eeprom_is_valid(e))
		return;

	/* Check EEPROM version - only version 2+ has DDR adjustment tables */
	if (e->version < 2) {
		debug("EEPROM version is %d\n", e->version);
		return;
	}

	debug("EEPROM offset table\n");
	for (i = 0; i < DRAM_TABLE_NUM + 1; i++)
		debug("off[%d]=%d\n", i, e->off[i]);

	/* Calculate DRAM adjustment table sizes */
	for (i = 0; i < DRAM_TABLE_NUM; i++)
		adj_table_size[i] = (e->off[i + 1] - e->off[i]) /
				(sizeof(struct dram_cfg_param));

	debug("\nSizes table\n");
	for (i = 0; i < DRAM_TABLE_NUM; i++)
		debug("sizes[%d]=%d\n", i, adj_table_size[i]);

	/* Adjust DRAM controller configuration table */
	debug("\nAdjusting DDRC table: offset=%d, count=%d\n",
		e->off[0], adj_table_size[0]);
	adjust_dram_table(e->off[0], adj_table_size[0],
				d->ddrc_cfg, d->ddrc_cfg_num);

	/* Adjust DDR PHY configuration table */
	debug("\nAdjusting DDR PHY CFG table: offset=%d, count=%d\n",
		e->off[1], adj_table_size[1]);
	adjust_dram_table(e->off[1], adj_table_size[1],
				d->ddrphy_cfg, d->ddrphy_cfg_num);

	/* Adjust DDR PHY PIE table */
	debug("\nAdjusting DDR PHY PIE table: offset=%d, count=%d\n",
		e->off[2], adj_table_size[2]);
	adjust_dram_table(e->off[2], adj_table_size[2],
				d->ddrphy_pie, d->ddrphy_pie_num);

	/* Adjust FSP configuration tables
	 * i.MX8M B0 has 3 tables, i.MX8M B1 and i.MX8M Mini have 4 tables
	 */
	idx_map = (d->fsp_msg_num == 4) ? b1_idx_map : b0_idx_map;
	for (i = 0; i < d->fsp_msg_num; i++) {
		int j = idx_map[i];
		debug("\nAdjusting FSP table %d: offset=%d, count=%d\n",
			i, e->off[j], adj_table_size[j]);
		adjust_dram_table(e->off[j], adj_table_size[j],
			d->fsp_msg[i].fsp_cfg, d->fsp_msg[i].fsp_cfg_num);
	}
}
#endif
