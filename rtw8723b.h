/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */
/* Copyright(c) 2024-2026 Luka Gejak <luka.gejak@linux.dev>
 */

#ifndef __RTW8723B_H__
#define __RTW8723B_H__

#include "rtw8723x.h"

extern const struct rtw_chip_info rtw8723b_hw_spec;

/* phy status page0 */
#define GET_PHY_STAT_P0_PWDB(phy_stat)					\
	le32_get_bits(*((__le32 *)(phy_stat) + 0x00), GENMASK(15, 8))

/* phy status page1 */
#define GET_PHY_STAT_P1_PWDB_A(phy_stat)				\
	le32_get_bits(*((__le32 *)(phy_stat) + 0x00), GENMASK(15, 8))
#define GET_PHY_STAT_P1_PWDB_B(phy_stat)				\
	le32_get_bits(*((__le32 *)(phy_stat) + 0x00), GENMASK(23, 16))
#define GET_PHY_STAT_P1_RF_MODE(phy_stat)				\
	le32_get_bits(*((__le32 *)(phy_stat) + 0x03), GENMASK(29, 28))
#define GET_PHY_STAT_P1_L_RXSC(phy_stat)				\
	le32_get_bits(*((__le32 *)(phy_stat) + 0x01), GENMASK(11, 8))
#define GET_PHY_STAT_P1_HT_RXSC(phy_stat)				\
	le32_get_bits(*((__le32 *)(phy_stat) + 0x01), GENMASK(15, 12))
#define GET_PHY_STAT_P1_RXEVM_A(phy_stat)				\
	le32_get_bits(*((__le32 *)(phy_stat) + 0x04), GENMASK(7, 0))
#define GET_PHY_STAT_P1_CFO_TAIL_A(phy_stat)				\
	le32_get_bits(*((__le32 *)(phy_stat) + 0x05), GENMASK(7, 0))
#define GET_PHY_STAT_P1_RXSNR_A(phy_stat)				\
	le32_get_bits(*((__le32 *)(phy_stat) + 0x06), GENMASK(7, 0))

#define RTW8723B_DEF_OFDM_SWING_INDEX	28
#define RTW8723B_DEF_CCK_SWING_INDEX	28

#endif
