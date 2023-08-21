#
# Copyright 2018-2019 Variscite Ltd
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += ../common/imx8m_eeprom.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing_b0.o lpddr4_timing.o lpddr4_timing_default.o lpddr4_timing_2g.o lpddr4_timing_4g.o
obj-$(CONFIG_POWER_PFUZE100) += ../../freescale/common/pfuze.o
else
obj-y += imx8m_var_dart.o ../common/mmc.o
endif
