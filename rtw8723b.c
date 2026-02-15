// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/* Copyright(c) Michael Straube <straube.linux@gmail.com> */
/* Copyright(c) 2024-2026 Luka Gejak <luka.gejak@linux.dev> */

#include "main.h"
#include "coex.h"
#include "phy.h"
#include "mac.h"
#include "sdio.h"
#include "rtw8723b.h"
#include "tx.h"
#include "rtw8723b_table.h"
/* for struct phy_status_8703b */
#include "rtw8703b.h"

#define TRANS_SEQ_END			\
	0xFFFF,				\
	RTW_PWR_CUT_ALL_MSK,		\
	RTW_PWR_INTF_ALL_MSK,		\
	0,				\
	RTW_PWR_CMD_END, 0, 0

#define BIT_FEN_PPLL		BIT(7)
#define BIT_FEN_DIO_PCIE	BIT(5)

#define TBTT_PROHIBIT_SETUP_TIME		0x04
#define TBTT_PROHIBIT_HOLD_TIME_STOP_BCN	0x64
#define WLAN_BCN_DMA_TIME			0x02
#define WLAN_ANT_SEL				0x82
#define WLAN_BAR_VAL				0x0201ffff
#define WLAN_SLOT_TIME				0x09

#define ADDA_ON_VAL_8723B			0x01c00014

#define MASK_NETTYPE	0x30000
#define _NETTYPE(x)	(((x) & 0x3) << 16)
#define NT_LINK_AP	0x2

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
#define FIELD_PREP_CONST FIELD_PREP
#endif

#define WLAN_RX_FILTER0			0xFFFF
#define WLAN_RX_FILTER1			0x400
#define WLAN_RX_FILTER2			0xFFFF
#define WLAN_RCR_CFG			0x700060CE

#define REG_FPGA0_XA_RF_SW_CTRL		0x0870
#define REG_FPGA0_XA_RF_INT_OE		0x0860
#define REG_FPGA0_XA_HSSI_PARM2		0x0824

#define REG_FPGA0_XB_RF_SW_CTRL		0x0872
#define REG_FPGA0_XB_RF_INT_OE		0x0864
#define REG_FPGA0_XB_HSSI_PARM2		0x082c

#define RFSI_RFENV			0x10
#define HSSI_3WIRE_ADDR_LEN		0x400
#define HSSI_3WIRE_DATA_LEN		0x800

#define BIT_EN_PDN			BIT(4)

#define REG_CAM_CMD			0x0670
#define CAM_CMD_POLLING			BIT(31)

#define REG_PKT_VO_VI_LIFE_TIME		0x04C0
#define REG_PKT_BE_BK_LIFE_TIME		0x04C2

#define REG_BT_CONTROL_8723B		0x0764
#define REG_PWR_DATA			0x0038

#define RF_RCK_OS			0x30
#define RF_TXPA_G1			0x31
#define RF_TXPA_G2			0x32
#define IQK_DELAY_TIME_8723B		20

#define REG_B_RXIQI			0x0c1c

#define REG_NAV_UPPER			0x0652

/* TODO: check if these or equivalent are present in rtw88 */
#define BCNQ_PAGE_NUM_8723B     	0x08
#define BCNQ1_PAGE_NUM_8723B		0x00
#define WOWLAN_PAGE_NUM_8723B		0x00
#define TX_TOTAL_PAGE_NUMBER_8723B	(0xFF - BCNQ_PAGE_NUM_8723B - BCNQ1_PAGE_NUM_8723B - WOWLAN_PAGE_NUM_8723B) /* 0xf7 */

/* TODO: check if these or equivalent are present in rtw88 */
#define REG_TXPKTBUF_BCNQ_BDNY_8723B		0x0424
#define REG_TXPKTBUF_MGQ_BDNY_8723B		0x0425
#define REG_TXPKTBUF_WMAC_LBK_BF_HD_8723B	0x045D
#define REG_TRXFF_BNDY				0x0114
#define REG_TDECTRL				0x0208

/* rssi in percentage % (dbm = % - 100) */
/* These are used to select simple signal quality levels, might need
 * tweaking. Same for rf_para tables below.
 */
static const u8 wl_rssi_step_8723b[] = {60, 50, 44, 30};
static const u8 bt_rssi_step_8723b[] = {30, 30, 30, 30};
static const struct coex_5g_afh_map afh_5g_8723b[] = { {0, 0, 0} };

static const struct coex_rf_para rf_para_tx_8723b[] = {
	{0, 0, false, 7},  /* for normal */
	{0, 10, false, 7}, /* for WL-CPT */
	{1, 0, true, 4},
	{1, 2, true, 4},
	{1, 10, true, 4},
	{1, 15, true, 4}
};

static const struct coex_rf_para rf_para_rx_8723b[] = {
	{0, 0, false, 7},  /* for normal */
	{0, 10, false, 7}, /* for WL-CPT */
	{1, 0, true, 5},
	{1, 2, true, 5},
	{1, 10, true, 5},
	{1, 15, true, 5}
};

static_assert(ARRAY_SIZE(rf_para_tx_8723b) == ARRAY_SIZE(rf_para_rx_8723b));

/* taken from vendor file hal/phydm/halrf/halrf_powertracking_ce.c
 * ofdm_swing_table_new
 */
static const u32 rtw8723b_ofdm_swing_table[] = {
	0x0b40002d, /* 0, -15.0dB */
	0x0c000030, /* 1, -14.5dB */
	0x0cc00033, /* 2, -14.0dB */
	0x0d800036, /* 3, -13.5dB */
	0x0e400039, /* 4, -13.0dB */
	0x0f00003c, /* 5, -12.5dB */
	0x10000040, /* 6, -12.0dB */
	0x11000044, /* 7, -11.5dB */
	0x12000048, /* 8, -11.0dB */
	0x1300004c, /* 9, -10.5dB */
	0x14400051, /* 10, -10.0dB */
	0x15800056, /* 11, -9.5dB */
	0x16c0005b, /* 12, -9.0dB */
	0x18000060, /* 13, -8.5dB */
	0x19800066, /* 14, -8.0dB */
	0x1b00006c, /* 15, -7.5dB */
	0x1c800072, /* 16, -7.0dB */
	0x1e400079, /* 17, -6.5dB */
	0x20000080, /* 18, -6.0dB */
	0x22000088, /* 19, -5.5dB */
	0x24000090, /* 20, -5.0dB */
	0x26000098, /* 21, -4.5dB */
	0x288000a2, /* 22, -4.0dB */
	0x2ac000ab, /* 23, -3.5dB */
	0x2d4000b5, /* 24, -3.0dB */
	0x300000c0, /* 25, -2.5dB */
	0x32c000cb, /* 26, -2.0dB */
	0x35c000d7, /* 27, -1.5dB */
	0x390000e4, /* 28, -1.0dB */
	0x3c8000f2, /* 29, -0.5dB */
	0x40000100, /* 30, +0dB */
	0x43c0010f, /* 31, +0.5dB */
	0x47c0011f, /* 32, +1.0dB */
	0x4c000130, /* 33, +1.5dB */
	0x50800142, /* 34, +2.0dB */
	0x55400155, /* 35, +2.5dB */
	0x5a400169, /* 36, +3.0dB */
	0x5fc0017f, /* 37, +3.5dB */
	0x65400195, /* 38, +4.0dB */
	0x6b8001ae, /* 39, +4.5dB */
	0x71c001c7, /* 40, +5.0dB */
	0x788001e2, /* 41, +5.5dB */
	0x7f8001fe, /* 42, +6.0dB */
};

/* adapted from vendor file hal/phydm/halrf/rtl8723b/halrf_8723b_ce.c
 * function: set_cck_filter_coefficient
 */
static const u32 rtw8723b_cck_pwr_regs[] = {
	0x0a22, 0x0a23, 0x0a24, 0x0a25, 0x0a26, 0x0a27, 0x0a28, 0x0a29,
};

/* taken from vendor file hal/phydm/halrf/halrf_powertracking_ce.c
 * cck_swing_table_ch1_ch13_new
 */
static const u8 rtw8732b_cck_swing_table_ch1_ch13[][8] = {
	{0x09, 0x08, 0x07, 0x06, 0x04, 0x03, 0x01, 0x01},	/* 0, -16.0dB */
	{0x09, 0x09, 0x08, 0x06, 0x05, 0x03, 0x01, 0x01},	/* 1, -15.5dB */
	{0x0a, 0x09, 0x08, 0x07, 0x05, 0x03, 0x02, 0x01},	/* 2, -15.0dB */
	{0x0a, 0x0a, 0x09, 0x07, 0x05, 0x03, 0x02, 0x01},	/* 3, -14.5dB */
	{0x0b, 0x0a, 0x09, 0x08, 0x06, 0x04, 0x02, 0x01},	/* 4, -14.0dB */
	{0x0b, 0x0b, 0x0a, 0x08, 0x06, 0x04, 0x02, 0x01},	/* 5, -13.5dB */
	{0x0c, 0x0c, 0x0a, 0x09, 0x06, 0x04, 0x02, 0x01},	/* 6, -13.0dB */
	{0x0d, 0x0c, 0x0b, 0x09, 0x07, 0x04, 0x02, 0x01},	/* 7, -12.5dB */
	{0x0d, 0x0d, 0x0c, 0x0a, 0x07, 0x05, 0x02, 0x01},	/* 8, -12.0dB */
	{0x0e, 0x0e, 0x0c, 0x0a, 0x08, 0x05, 0x02, 0x01},	/* 9, -11.5dB */
	{0x0f, 0x0f, 0x0d, 0x0b, 0x08, 0x05, 0x03, 0x01},	/* 10, -11.0dB */
	{0x10, 0x10, 0x0e, 0x0b, 0x08, 0x05, 0x03, 0x01},	/* 11, -10.5dB */
	{0x11, 0x11, 0x0f, 0x0c, 0x09, 0x06, 0x03, 0x01},	/* 12, -10.0dB */
	{0x12, 0x12, 0x0f, 0x0c, 0x09, 0x06, 0x03, 0x01},	/* 13, -9.5dB */
	{0x13, 0x13, 0x10, 0x0d, 0x0a, 0x06, 0x03, 0x01},	/* 14, -9.0dB */
	{0x14, 0x14, 0x11, 0x0e, 0x0b, 0x07, 0x03, 0x02},	/* 15, -8.5dB */
	{0x16, 0x15, 0x12, 0x0f, 0x0b, 0x07, 0x04, 0x01},	/* 16, -8.0dB */
	{0x17, 0x16, 0x13, 0x10, 0x0c, 0x08, 0x04, 0x02},	/* 17, -7.5dB */
	{0x18, 0x17, 0x15, 0x11, 0x0c, 0x08, 0x04, 0x02},	/* 18, -7.0dB */
	{0x1a, 0x19, 0x16, 0x12, 0x0d, 0x09, 0x04, 0x02},	/* 19, -6.5dB */
	{0x1b, 0x1a, 0x17, 0x13, 0x0e, 0x09, 0x04, 0x02},	/* 20, -6.0dB */
	{0x1d, 0x1c, 0x18, 0x14, 0x0f, 0x0a, 0x05, 0x02},	/* 21, -5.5dB */
	{0x1f, 0x1e, 0x1a, 0x15, 0x10, 0x0a, 0x05, 0x02},	/* 22, -5.0dB */
	{0x20, 0x20, 0x1b, 0x16, 0x11, 0x08, 0x05, 0x02},	/* 23, -4.5dB */
	{0x22, 0x21, 0x1d, 0x18, 0x11, 0x0b, 0x06, 0x02},	/* 24, -4.0dB */
	{0x24, 0x23, 0x1f, 0x19, 0x13, 0x0c, 0x06, 0x03},	/* 25, -3.5dB */
	{0x26, 0x25, 0x21, 0x1b, 0x14, 0x0d, 0x06, 0x03},	/* 26, -3.0dB */
	{0x28, 0x28, 0x22, 0x1c, 0x15, 0x0d, 0x07, 0x03},	/* 27, -2.5dB */
	{0x2b, 0x2a, 0x25, 0x1e, 0x16, 0x0e, 0x07, 0x03},	/* 28, -2.0dB */
	{0x2d, 0x2d, 0x27, 0x1f, 0x18, 0x0f, 0x08, 0x03},	/* 29, -1.5dB */
	{0x30, 0x2f, 0x29, 0x21, 0x19, 0x10, 0x08, 0x03},	/* 30, -1.0dB */
	{0x33, 0x32, 0x2b, 0x23, 0x1a, 0x11, 0x08, 0x04},	/* 31, -0.5dB */
	{0x36, 0x35, 0x2e, 0x25, 0x1c, 0x12, 0x09, 0x04},	/* 32, +0dB */
};

/* taken from vendor file hal/phydm/halrf/halrf_powertracking_ce.c
 * cck_swing_table_ch14_new
 */
static const u8 rtw8732b_cck_swing_table_ch14[][8] = {
	{0x09, 0x08, 0x07, 0x04, 0x00, 0x00, 0x00, 0x00},	/* 0, -16.0dB */
	{0x09, 0x09, 0x08, 0x05, 0x00, 0x00, 0x00, 0x00},	/* 1, -15.5dB */
	{0x0a, 0x09, 0x08, 0x05, 0x00, 0x00, 0x00, 0x00},	/* 2, -15.0dB */
	{0x0a, 0x0a, 0x09, 0x05, 0x00, 0x00, 0x00, 0x00},	/* 3, -14.5dB */
	{0x0b, 0x0a, 0x09, 0x05, 0x00, 0x00, 0x00, 0x00},	/* 4, -14.0dB */
	{0x0b, 0x0b, 0x0a, 0x06, 0x00, 0x00, 0x00, 0x00},	/* 5, -13.5dB */
	{0x0c, 0x0c, 0x0a, 0x06, 0x00, 0x00, 0x00, 0x00},	/* 6, -13.0dB */
	{0x0d, 0x0c, 0x0b, 0x06, 0x00, 0x00, 0x00, 0x00},	/* 7, -12.5dB */
	{0x0d, 0x0d, 0x0c, 0x07, 0x00, 0x00, 0x00, 0x00},	/* 8, -12.0dB */
	{0x0e, 0x0e, 0x0c, 0x07, 0x00, 0x00, 0x00, 0x00},	/* 9, -11.5dB */
	{0x0f, 0x0f, 0x0d, 0x08, 0x00, 0x00, 0x00, 0x00},	/* 10, -11.0dB */
	{0x10, 0x10, 0x0e, 0x08, 0x00, 0x00, 0x00, 0x00},	/* 11, -10.5dB */
	{0x11, 0x11, 0x0f, 0x09, 0x00, 0x00, 0x00, 0x00},	/* 12, -10.0dB */
	{0x12, 0x12, 0x0f, 0x09, 0x00, 0x00, 0x00, 0x00},	/* 13, -9.5dB */
	{0x13, 0x13, 0x10, 0x0a, 0x00, 0x00, 0x00, 0x00},	/* 14, -9.0dB */
	{0x14, 0x14, 0x11, 0x0a, 0x00, 0x00, 0x00, 0x00},	/* 15, -8.5dB */
	{0x16, 0x15, 0x12, 0x0b, 0x00, 0x00, 0x00, 0x00},	/* 16, -8.0dB */
	{0x17, 0x16, 0x13, 0x0b, 0x00, 0x00, 0x00, 0x00},	/* 17, -7.5dB */
	{0x18, 0x17, 0x15, 0x0c, 0x00, 0x00, 0x00, 0x00},	/* 18, -7.0dB */
	{0x1a, 0x19, 0x16, 0x0d, 0x00, 0x00, 0x00, 0x00},	/* 19, -6.5dB */
	{0x1b, 0x1a, 0x17, 0x0e, 0x00, 0x00, 0x00, 0x00},	/* 20, -6.0dB */
	{0x1d, 0x1c, 0x18, 0x0e, 0x00, 0x00, 0x00, 0x00},	/* 21, -5.5dB */
	{0x1f, 0x1e, 0x1a, 0x0f, 0x00, 0x00, 0x00, 0x00},	/* 22, -5.0dB */
	{0x20, 0x20, 0x1b, 0x10, 0x00, 0x00, 0x00, 0x00},	/* 23, -4.5dB */
	{0x22, 0x21, 0x1d, 0x11, 0x00, 0x00, 0x00, 0x00},	/* 24, -4.0dB */
	{0x24, 0x23, 0x1f, 0x12, 0x00, 0x00, 0x00, 0x00},	/* 25, -3.5dB */
	{0x26, 0x25, 0x21, 0x13, 0x00, 0x00, 0x00, 0x00},	/* 26, -3.0dB */
	{0x28, 0x28, 0x24, 0x14, 0x00, 0x00, 0x00, 0x00},	/* 27, -2.5dB */
	{0x2b, 0x2a, 0x25, 0x15, 0x00, 0x00, 0x00, 0x00},	/* 28, -2.0dB */
	{0x2d, 0x2d, 0x17, 0x17, 0x00, 0x00, 0x00, 0x00},	/* 29, -1.5dB */
	{0x30, 0x2f, 0x29, 0x18, 0x00, 0x00, 0x00, 0x00},	/* 30, -1.0dB */
	{0x33, 0x32, 0x2b, 0x19, 0x00, 0x00, 0x00, 0x00},	/* 31, -0.5dB */
	{0x36, 0x35, 0x2e, 0x1b, 0x00, 0x00, 0x00, 0x00},	/* 32, +0dB */
};

static_assert(ARRAY_SIZE(rtw8732b_cck_swing_table_ch1_ch13) ==
	      ARRAY_SIZE(rtw8732b_cck_swing_table_ch14));

#define RTW_OFDM_SWING_TABLE_SIZE	ARRAY_SIZE(rtw8723b_ofdm_swing_table)
#define RTW_CCK_SWING_TABLE_SIZE	ARRAY_SIZE(rtw8732b_cck_swing_table_ch14)


/* see vendor functions _InitPowerOn_8723BS and CardEnable
 */
static const struct rtw_pwr_seq_cmd trans_pre_enable_8723b[] = {
#if 0
	/* change crystal to 26M */
	{REG_AFE_PLL_CTRL,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(2), BIT(2)},
	{REG_AFE_CTRL_4 + 1,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0) | BIT(1), BIT(1)},
	{REG_AFE_CTRL_4,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(7), 0},

	 /* set up external crystal (XTAL) */
	{REG_PAD_CTRL1 + 2,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(7), BIT(7)},
	/* set CLK_REQ to high active */
	{0x0069,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(5), BIT(5)},
#endif

	/* unlock ISO/CLK/power control register */
	{REG_RSV_CTRL,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, 0xff, 0},

	{TRANS_SEQ_END},
};

/* transitions adapted from vendor file include/Hal8723BPwrSeq.h
 */
static const struct rtw_pwr_seq_cmd trans_carddis_to_cardemu_8723b[] = {
	/*
	 * NOTE: these are ready!
	 *
	 * All values match the comments; verified with AI
	 */

	//{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT3 | BIT7, 0}, /*clear suspend enable and power down enable*/
	{0x0005,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(3) | BIT(7), 0},

	//{0x0086, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_SDIO, PWR_CMD_WRITE, BIT0, 0}, /*Set SDIO suspend local register*/
	{0x0086,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_SDIO,
	 RTW_PWR_CMD_WRITE, BIT(0), 0},

	//{0x0086, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_SDIO, PWR_CMD_POLLING, BIT1, BIT1}, /*wait power state to suspend*/
	{0x0086,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_SDIO,
	 RTW_PWR_CMD_POLLING, BIT(1), BIT(1)},

	//{0x004A, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT0, 0}, /*0x48[16] = 0 to disable GPIO9 as EXT WAKEUP*/
	{0x004A,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_USB_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0), 0},

	// {0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT3 | BIT4, 0}, /*0x04[12:11] = 2b'01enable WL suspend*/
	{0x0005,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(3) | BIT(4), 0},

	//{0x0023, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT4, 0}, /*0x23[4] = 1b'0 12H LDO enter normal mode*/
	{0x0023,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(4), 0},

	//{0x0301, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_PCI_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, 0xFF, 0},/*PCIe DMA start*/
	{0x0301,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_PCI_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, 0xFF, 0},

	{TRANS_SEQ_END},
};

static const struct rtw_pwr_seq_cmd trans_cardemu_to_act_8723b[] = {
	/*
	 * NOTE: these are ready!
	 *
	 * All values match the comments; verified with AI
	 */

	//{0x0020, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK | PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT0, BIT0}, /*0x20[0] = 1b'1 enable LDOA12 MACRO block for all interface*/
	{0x0020,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_USB_MSK | RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0), BIT(0)},

	//{0x0067, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK | PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT4, 0}, /*0x67[0] = 0 to disable BT_GPS_SEL pins*/
	{0x0067,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_USB_MSK | RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(4), 0},

	//{0x0001, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK | PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_DELAY, 1, PWRSEQ_DELAY_MS},/*Delay 1ms*/
	{0x0001,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_USB_MSK | RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_DELAY, 1, RTW_PWR_DELAY_MS},

	//{0x0000, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK | PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT5, 0}, /*0x00[5] = 1b'0 release analog Ips to digital ,1:isolation*/
	{0x0000,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_USB_MSK | RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(5), 0},

	//{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, (BIT4 | BIT3 | BIT2), 0},/* disable SW LPS 0x04[10]=0 and WLSUS_EN 0x04[11]=0*/
	{0x0005,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, (BIT(4) | BIT(3) | BIT(2)), 0},

	//{0x0075, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_PCI_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT0 , BIT0},/* Disable USB suspend */
	{0x0075,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_PCI_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0), BIT(0)},

	//{0x0006, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_POLLING, BIT1, BIT1},/* wait till 0x04[17] = 1    power ready*/
	{0x0006,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_POLLING, BIT(1), BIT(1)},

	//{0x0075, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_PCI_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT0 , 0},/* Enable USB suspend */
	{0x0075,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_PCI_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0), 0},

	//{0x0006, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT0, BIT0},/* release WLON reset  0x04[16]=1*/
	{0x0006,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0), BIT(0)},

	//{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT7, 0},/* disable HWPDN 0x04[15]=0*/
	{0x0005,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(7), 0},

	//{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, (BIT4 | BIT3), 0},/* disable WL suspend*/
	{0x0005,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(4) | BIT(3), 0},

	//{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT0, BIT0},/* polling until return 0*/
	{0x0005,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0), BIT(0)},

	//{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_POLLING, BIT0, 0},/**/
	{0x0005,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_POLLING, BIT(0), 0},

	//{0x0010, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT6, BIT6},/* Enable WL control XTAL setting*/
	{0x0010,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(6), BIT(6)},

	//{0x0049, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT1, BIT1},/*Enable falling edge triggering interrupt*/
	{0x0049,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(1), BIT(1)},

	//{0x0063, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT1, BIT1},/*Enable GPIO9 interrupt mode*/
	{0x0063,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(1), BIT(1)},

	//{0x0062, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT1, 0},/*Enable GPIO9 input mode*/
	{0x0062,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(1), 0},

	//{0x0058, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT0, BIT0},/*Enable HSISR GPIO[C:0] interrupt*/
	{0x0058,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0), BIT(0)},

	//{0x005A, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT1, BIT1},/*Enable HSISR GPIO9 interrupt*/
	{0x005A,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(1), BIT(1)},

	//{0x0068, PWR_CUT_TESTCHIP_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT3, BIT3},/*For GPIO9 internal pull high setting by test chip*/
	{0x0068,
	 RTW_PWR_CUT_TEST_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(3), BIT(3)},

	//{0x0069, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT6, BIT6},/*For GPIO9 internal pull high setting*/
	{0x0069,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(6), BIT(6)},

	 {TRANS_SEQ_END},
};

static const struct rtw_pwr_seq_cmd trans_act_to_lps_8723b[] = {
	/*
	 * NOTE: these are ready!
	 *
	 * All values match the comments; verified with AI
	 */

	//{0x0301, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_PCI_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, 0xFF, 0xFF},/*PCIe DMA stop*/
	{0x0301,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_PCI_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, 0xFF, 0xFF},

	//{0x0522, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, 0xFF, 0xFF},/*Tx Pause*/
	{0x0522,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, 0xFF, 0xFF},

	//{0x05F8, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_POLLING, 0xFF, 0},/*Should be zero if no packet is transmitting*/
	{0x05F8,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_POLLING, 0xFF, 0},

	//{0x05F9, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_POLLING, 0xFF, 0},/*Should be zero if no packet is transmitting*/
	{0x05F9,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_POLLING, 0xFF, 0},

	//{0x05FA, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_POLLING, 0xFF, 0},/*Should be zero if no packet is transmitting*/
	{0x05FA,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_POLLING, 0xFF, 0},

	//{0x05FB, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_POLLING, 0xFF, 0},/*Should be zero if no packet is transmitting*/
	{0x05FB,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_POLLING, 0xFF, 0},

	//{0x0002, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT0, 0},/*CCK and OFDM are disabled, and clock are gated*/
	{0x0002,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0), 0},

	//{0x0002, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_DELAY, 0, PWRSEQ_DELAY_US},/*Delay 1us*/
	{0x0002,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_DELAY, 0, RTW_PWR_DELAY_US},

	//{0x0002, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT1, 0},/*Whole BB is reset*/
	{0x0002,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(1), 0},

	//{0x0100, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, 0xFF, 0x03},/*Reset MAC TRX*/
	{0x0100,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, 0xFF, 0x03},

	//{0x0101, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT1, 0},/*check if removed later*/
	{0x0101,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(1), 0},

	//{0x0093, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, 0xFF, 0x00},/*When driver enter Sus/ Disable, enable LOP for BT*/
	{0x0093,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, 0xFF, 0x00},

	//{0x0553, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT5, BIT5},/*Respond TxOK to scheduler*/
	{0x0553,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(5), BIT(5)},

	{TRANS_SEQ_END},
};

/* adapted from vendor function CardDisableRTL8723BSdio
 */
static const struct rtw_pwr_seq_cmd trans_act_to_reset_mcu_8723b[] = {
	{REG_SYS_FUNC_EN + 1,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT_FEN_CPUEN, 0},
	/* reset MCU ready */
	{REG_MCUFW_CTRL,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, 0xff, 0},
	/* reset MCU IO wrapper */
	{REG_RSV_CTRL + 1,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0), 0},
	{REG_RSV_CTRL + 1,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0), 1},
	{TRANS_SEQ_END},
};

static const struct rtw_pwr_seq_cmd trans_act_to_cardemu_8723b[] = {
	/*
	 * NOTE: these are ready!
	 *
	 * All values match the comments; verified with AI
	 */

	//{0x001F, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, 0xFF, 0},/*0x1F[7:0] = 0 turn off RF*/
	{0x001F,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, 0xFF, 0},

	//{0x0049, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT1, 0},/*Enable rising edge triggering interrupt*/
	{0x0049,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(1), 0},

	//{0x0006, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT0, BIT0},/* release WLON reset  0x04[16]=1*/
	{0x0006,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0), BIT(0)},

	//{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT1, BIT1}, /*0x04[9] = 1 turn off MAC by HW state machine*/
	{0x0005,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(1), BIT(1)},

	//{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_POLLING, BIT1, 0}, /*wait till 0x04[9] = 0 polling until return 0 to disable*/
	{0x0005,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_POLLING, BIT(1), 0},

	//{0x0010, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_ALL_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT6, 0},/* Enable BT control XTAL setting*/
	{0x0010,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(6), 0},

	//{0x0000, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK | PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT5, BIT5}, /*0x00[5] = 1b'1 analog Ips to digital ,1:isolation*/
	{0x0000,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_USB_MSK | RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(5), BIT(5)},

	//{0x0020, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK | PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT0, 0}, /*0x20[0] = 1b'0 disable LDOA12 MACRO block*/
	{0x0020,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_USB_MSK | RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0), 0},

	{TRANS_SEQ_END},
};

static const struct rtw_pwr_seq_cmd trans_cardemu_to_carddis_8723b[] = {
	/*
	 * NOTE: these are ready!
	 *
	 * All values match the comments; verified with AI
	 */

	//{0x0007, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, 0xFF, 0x20}, /*0x07 = 0x20 , SOP option to disable BG/MB*/
	{0x0007,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, 0xFF, 0x20},

	//{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK | PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT3 | BIT4, BIT3}, /*0x04[12:11] = 2b'01 enable WL suspend*/
	{0x0005,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_USB_MSK | RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(3) | BIT(4), BIT(3)},

	//{0x0005, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_PCI_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT2, BIT2}, /*0x04[10] = 1, enable SW LPS*/
	{0x0005,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_PCI_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(2), BIT(2)},

	//{0x004A, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT0, 1}, /*0x48[16] = 1 to enable GPIO9 as EXT WAKEUP*/
	{0x004A,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_USB_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0), 1},

	//{0x0023, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_MAC, PWR_CMD_WRITE, BIT4, BIT4}, /*0x23[4] = 1b'1 12H LDO enter sleep mode*/
	{0x0023,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(4), BIT(4)},

	//{0x0086, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_SDIO, PWR_CMD_WRITE, BIT0, BIT0}, /*Set SDIO suspend local register*/
	{0x0086,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_SDIO,
	 RTW_PWR_CMD_WRITE, BIT(0), BIT(0)},

	//{0x0086, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK, PWR_BASEADDR_SDIO, PWR_CMD_POLLING, BIT1, 0}, /*wait power state to suspend*/
	{0x0086,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_SDIO,
	 RTW_PWR_CMD_POLLING, BIT(1), 0},

	{TRANS_SEQ_END},
};

/* adapted from vendor file hal/rtl8723b/Hal8723BPwrSeq.c
 */
static const struct rtw_pwr_seq_cmd * const card_enable_flow_8723b[] = {
	trans_pre_enable_8723b,
	trans_carddis_to_cardemu_8723b,
	trans_cardemu_to_act_8723b,
	NULL
};

/* see vendor function CardDisableRTL8723BSdio
 */
static const struct rtw_pwr_seq_cmd * const card_disable_flow_8723b[] = {
	trans_act_to_lps_8723b,
	trans_act_to_reset_mcu_8723b,
	trans_act_to_cardemu_8723b,
	trans_cardemu_to_carddis_8723b,
	NULL
};

static const struct rtw_page_table page_table_8723b[] = {
	/*
	 * NOTE: this is ready!
	 *
	 * given we set: rsvd_drv_pg_num = 8
	 *
	 * verified by comparing mac.c:__priority_queue_cfg_legacy
	 * with vendor _InitQueueReservedPage
	 */
	{12, 2, 2, 0, 1}, /* SDIO */
	{12, 2, 2, 0, 1},
	{12, 2, 2, 0, 1},
	{12, 2, 2, 0, 1},
	{12, 2, 2, 0, 1},
};

/* TODO: check this table */
static const struct rtw_rqpn rqpn_table_8723b[] = {
	/* SDIO */
	{RTW_DMA_MAPPING_NORMAL, RTW_DMA_MAPPING_NORMAL,
	 RTW_DMA_MAPPING_LOW, RTW_DMA_MAPPING_LOW,
	 RTW_DMA_MAPPING_HIGH, RTW_DMA_MAPPING_HIGH},
	/* PCIE */
	{RTW_DMA_MAPPING_NORMAL, RTW_DMA_MAPPING_NORMAL,
	 RTW_DMA_MAPPING_LOW, RTW_DMA_MAPPING_LOW,
	 RTW_DMA_MAPPING_HIGH, RTW_DMA_MAPPING_HIGH},
	/* USB bulkout 2 */
	{RTW_DMA_MAPPING_NORMAL, RTW_DMA_MAPPING_NORMAL,
	 RTW_DMA_MAPPING_NORMAL, RTW_DMA_MAPPING_HIGH,
	 RTW_DMA_MAPPING_HIGH, RTW_DMA_MAPPING_HIGH},
	/* USB bulkout 3 */
	{RTW_DMA_MAPPING_NORMAL, RTW_DMA_MAPPING_NORMAL,
	 RTW_DMA_MAPPING_LOW, RTW_DMA_MAPPING_LOW,
	 RTW_DMA_MAPPING_HIGH, RTW_DMA_MAPPING_HIGH},
	/* USB bulkout 4 */
	{RTW_DMA_MAPPING_NORMAL, RTW_DMA_MAPPING_NORMAL,
	 RTW_DMA_MAPPING_LOW, RTW_DMA_MAPPING_LOW,
	 RTW_DMA_MAPPING_HIGH, RTW_DMA_MAPPING_HIGH},
};

/* taken from vendor file hal/phydm/rtl8723b/halhwimg8723b_rf.c
 * txpowertrack_sdio.TXT section
 * NOTE: tables for pcie and usb slightly differ in the vendor driver
 */
static const u8 rtw8723b_pwrtrk_2gb_n[] = {
	0, 0, 1, 2, 2, 2, 3, 3, 3, 4, 5, 5, 6, 6, 6, 6,
	7, 7, 7, 8, 8, 9, 9, 10, 10, 11, 12, 13, 14, 15
};

static const u8 rtw8723b_pwrtrk_2gb_p[] = {
	0, 0, 1, 2, 2, 3, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8,
	9, 9, 10, 10, 10, 11, 11, 12, 12, 13, 13, 14, 15, 15
};

static const u8 rtw8723b_pwrtrk_2ga_n[] = {
	0, 0, 1, 2, 2, 2, 3, 3, 3, 4, 5, 5, 6, 6, 6, 6,
	7, 7, 7, 8, 8, 9, 9, 10, 10, 11, 12, 13, 14, 15
};

static const u8 rtw8723b_pwrtrk_2ga_p[] = {
	0, 0, 1, 2, 2, 3, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8,
	9, 9, 10, 10, 10, 11, 11, 12, 12, 13, 13, 14, 15, 15
};

static const u8 rtw8723b_pwrtrk_2g_cck_b_n[] = {
	0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 6, 6, 7, 7, 7, 8,
	8, 8, 9, 9, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15
};

static const u8 rtw8723b_pwrtrk_2g_cck_b_p[] = {
	0, 0, 1, 2, 2, 2, 3, 3, 3, 4, 5, 5, 6, 6, 7, 7,
	8, 8, 9, 9, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15
};

static const u8 rtw8723b_pwrtrk_2g_cck_a_n[] = {
	0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 6, 6, 7, 7, 7, 8,
	8, 8, 9, 9, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15
};

static const u8 rtw8723b_pwrtrk_2g_cck_a_p[] = {
	0, 0, 1, 2, 2, 2, 3, 3, 3, 4, 5, 5, 6, 6, 7, 7,
	8, 8, 9, 9, 9, 10, 10, 11, 11, 12, 12, 13, 14, 15
};
/* -----------------------------------------------------------------------*/

// static const s8 rtw8723b_pwrtrk_xtal_n[] = {
// 	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
// 	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
// };
//
// static const s8 rtw8723b_pwrtrk_xtal_p[] = {
// 	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
// 	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
// };

static const struct rtw_pwr_track_tbl rtw8723b_rtw_pwr_track_tbl = {
	.pwrtrk_2gb_n = rtw8723b_pwrtrk_2gb_n,
	.pwrtrk_2gb_p = rtw8723b_pwrtrk_2gb_p,
	.pwrtrk_2ga_n = rtw8723b_pwrtrk_2ga_n,
	.pwrtrk_2ga_p = rtw8723b_pwrtrk_2ga_p,
	.pwrtrk_2g_cckb_n = rtw8723b_pwrtrk_2g_cck_b_n,
	.pwrtrk_2g_cckb_p = rtw8723b_pwrtrk_2g_cck_b_p,
	.pwrtrk_2g_ccka_n = rtw8723b_pwrtrk_2g_cck_a_n,
	.pwrtrk_2g_ccka_p = rtw8723b_pwrtrk_2g_cck_a_p,
	// .pwrtrk_xtal_n = rtw8723b_pwrtrk_xtal_n,
	// .pwrtrk_xtal_p = rtw8723b_pwrtrk_xtal_p,
	/* used in rtw8723x_pwrtrack_set_xtal which is not done in 8723b vendor driver*/
	.pwrtrk_xtal_n = NULL,
	.pwrtrk_xtal_p = NULL,
};

static const struct rtw_rfe_def rtw8723b_rfe_defs[] = {
	[0] = { .phy_pg_tbl	= &rtw8723b_bb_pg_tbl,
		.txpwr_lmt_tbl	= &rtw8723b_txpwr_lmt_tbl,
		.pwr_track_tbl	= &rtw8723b_rtw_pwr_track_tbl, },
};

/* Shared-Antenna Coex Table */
static const struct coex_table_para table_sant_8723b[] = {
	{0xffffffff, 0xffffffff}, /* case-0 */
	{0x55555555, 0x55555555},
	{0x66555555, 0x66555555},
	{0xaaaaaaaa, 0xaaaaaaaa},
	{0x5a5a5a5a, 0x5a5a5a5a},
	{0xfafafafa, 0xfafafafa}, /* case-5 */
	{0x6a5a5555, 0xaaaaaaaa},
	{0x6a5a56aa, 0x6a5a56aa},
	{0x6a5a5a5a, 0x6a5a5a5a},
	{0x66555555, 0x5a5a5a5a},
	{0x66555555, 0x6a5a5a5a}, /* case-10 */
	{0x66555555, 0x6a5a5aaa},
	{0x66555555, 0x5a5a5aaa},
	{0x66555555, 0x6aaa5aaa},
	{0x66555555, 0xaaaa5aaa},
	{0x66555555, 0xaaaaaaaa}, /* case-15 */
	{0xffff55ff, 0xfafafafa},
	{0xffff55ff, 0x6afa5afa},
	{0xaaffffaa, 0xfafafafa},
	{0xaa5555aa, 0x5a5a5a5a},
	{0xaa5555aa, 0x6a5a5a5a}, /* case-20 */
	{0xaa5555aa, 0xaaaaaaaa},
	{0xffffffff, 0x5a5a5a5a},
	{0xffffffff, 0x5a5a5a5a},
	{0xffffffff, 0x55555555},
	{0xffffffff, 0x5a5a5aaa}, /* case-25 */
	{0x55555555, 0x5a5a5a5a},
	{0x55555555, 0xaaaaaaaa},
	{0x55555555, 0x6a5a6a5a},
	{0x66556655, 0x66556655},
	{0x66556aaa, 0x6a5a6aaa}, /* case-30 */
	{0xffffffff, 0x5aaa5aaa},
	{0x56555555, 0x5a5a5aaa},
};

/* Non-Shared-Antenna Coex Table */
static const struct coex_table_para table_nsant_8723b[] = {
	{0xffffffff, 0xffffffff}, /* case-100 */
	{0x55555555, 0x55555555},
	{0x66555555, 0x66555555},
	{0xaaaaaaaa, 0xaaaaaaaa},
	{0x5a5a5a5a, 0x5a5a5a5a},
	{0xfafafafa, 0xfafafafa}, /* case-105 */
	{0x5afa5afa, 0x5afa5afa},
	{0x55555555, 0xfafafafa},
	{0x66555555, 0xfafafafa},
	{0x66555555, 0x5a5a5a5a},
	{0x66555555, 0x6a5a5a5a}, /* case-110 */
	{0x66555555, 0xaaaaaaaa},
	{0xffff55ff, 0xfafafafa},
	{0xffff55ff, 0x5afa5afa},
	{0xffff55ff, 0xaaaaaaaa},
	{0xffff55ff, 0xffff55ff}, /* case-115 */
	{0xaaffffaa, 0x5afa5afa},
	{0xaaffffaa, 0xaaaaaaaa},
	{0xffffffff, 0xfafafafa},
	{0xffffffff, 0x5afa5afa},
	{0xffffffff, 0xaaaaaaaa}, /* case-120 */
	{0x55ff55ff, 0x5afa5afa},
	{0x55ff55ff, 0xaaaaaaaa},
	{0x55ff55ff, 0x55ff55ff}
};

/* Shared-Antenna TDMA */
static const struct coex_tdma_para tdma_sant_8723b[] = {
	{ {0x00, 0x00, 0x00, 0x00, 0x00} }, /* case-0 */
	{ {0x61, 0x45, 0x03, 0x11, 0x11} }, /* case-1 */
	{ {0x61, 0x3a, 0x03, 0x11, 0x11} },
	{ {0x61, 0x30, 0x03, 0x11, 0x11} },
	{ {0x61, 0x20, 0x03, 0x11, 0x11} },
	{ {0x61, 0x10, 0x03, 0x11, 0x11} }, /* case-5 */
	{ {0x61, 0x45, 0x03, 0x11, 0x10} },
	{ {0x61, 0x3a, 0x03, 0x11, 0x10} },
	{ {0x61, 0x30, 0x03, 0x11, 0x10} },
	{ {0x61, 0x20, 0x03, 0x11, 0x10} },
	{ {0x61, 0x10, 0x03, 0x11, 0x10} }, /* case-10 */
	{ {0x61, 0x08, 0x03, 0x11, 0x14} },
	{ {0x61, 0x08, 0x03, 0x10, 0x14} },
	{ {0x51, 0x08, 0x03, 0x10, 0x54} },
	{ {0x51, 0x08, 0x03, 0x10, 0x55} },
	{ {0x51, 0x08, 0x07, 0x10, 0x54} }, /* case-15 */
	{ {0x51, 0x45, 0x03, 0x10, 0x50} },
	{ {0x51, 0x3a, 0x03, 0x10, 0x50} },
	{ {0x51, 0x30, 0x03, 0x10, 0x50} },
	{ {0x51, 0x20, 0x03, 0x10, 0x50} },
	{ {0x51, 0x10, 0x03, 0x10, 0x50} }, /* case-20 */
	{ {0x51, 0x4a, 0x03, 0x10, 0x50} },
	{ {0x51, 0x0c, 0x03, 0x10, 0x54} },
	{ {0x55, 0x08, 0x03, 0x10, 0x54} },
	{ {0x65, 0x10, 0x03, 0x11, 0x10} },
	{ {0x51, 0x10, 0x03, 0x10, 0x51} }, /* case-25 */
	{ {0x51, 0x08, 0x03, 0x10, 0x50} },
	{ {0x61, 0x08, 0x03, 0x11, 0x11} }
};

/* Non-Shared-Antenna TDMA */
static const struct coex_tdma_para tdma_nsant_8723b[] = {
	{ {0x00, 0x00, 0x00, 0x00, 0x01} }, /* case-100 */
	{ {0x61, 0x45, 0x03, 0x11, 0x11} }, /* case-101 */
	{ {0x61, 0x3a, 0x03, 0x11, 0x11} },
	{ {0x61, 0x30, 0x03, 0x11, 0x11} },
	{ {0x61, 0x20, 0x03, 0x11, 0x11} },
	{ {0x61, 0x10, 0x03, 0x11, 0x11} }, /* case-105 */
	{ {0x61, 0x45, 0x03, 0x11, 0x10} },
	{ {0x61, 0x3a, 0x03, 0x11, 0x10} },
	{ {0x61, 0x30, 0x03, 0x11, 0x10} },
	{ {0x61, 0x20, 0x03, 0x11, 0x10} },
	{ {0x61, 0x10, 0x03, 0x11, 0x10} }, /* case-110 */
	{ {0x61, 0x08, 0x03, 0x11, 0x14} },
	{ {0x61, 0x08, 0x03, 0x10, 0x14} },
	{ {0x51, 0x08, 0x03, 0x10, 0x54} },
	{ {0x51, 0x08, 0x03, 0x10, 0x55} },
	{ {0x51, 0x08, 0x07, 0x10, 0x54} }, /* case-115 */
	{ {0x51, 0x45, 0x03, 0x10, 0x50} },
	{ {0x51, 0x3a, 0x03, 0x10, 0x50} },
	{ {0x51, 0x30, 0x03, 0x10, 0x50} },
	{ {0x51, 0x20, 0x03, 0x10, 0x50} },
	{ {0x51, 0x10, 0x03, 0x10, 0x50} }, /* case-120 */
	{ {0x51, 0x08, 0x03, 0x10, 0x50} }
};

/* vendor: hal/rtl8723b/rtl8723b_hal_init.c
 * function: Hal_EfusePowerSwitch
 */
static void rtw8723b_efuse_grant(struct rtw_dev *rtwdev, bool on)
{
	printk("%s begin: on=%s", __func__, on == true ? "true" : "false");

	/* TODO: if we do not need the 'enable BT ...' lines,
	 * then we can use __rtw8723x_efuse_grant
	 */

	if (on) {
		/* enable BT power cut 0x6A[14] = 1*/
		rtw_write8_set(rtwdev, 0x6b, BIT(6)); /* TODO: do we need this? */

		rtw_write8(rtwdev, REG_EFUSE_ACCESS, EFUSE_ACCESS_ON);

		rtw_write16_set(rtwdev, REG_SYS_FUNC_EN, BIT_FEN_ELDR);
		rtw_write16_set(rtwdev, REG_SYS_CLKR, BIT_LOADER_CLK_EN | BIT_ANA8M);
	} else {
		/*enable BT output isolation 0x6A[15] = 1 */
		rtw_write8_set(rtwdev, 0x6b, BIT(7)); /* TODO: do we need this? */

		rtw_write8(rtwdev, REG_EFUSE_ACCESS, EFUSE_ACCESS_OFF);
	}
	printk("%s end", __func__);
}

/* adapted from vendor: halrf_powertracking_ce.c
 * function: get_swing_index
 */
static u8 rtw8723b_default_ofdm_index(struct rtw_dev *rtwdev)
{
	printk("%s begin", __func__);

	u8 i;
	u32 val32;
	u32 swing;

	swing = rtw_read32_mask(rtwdev, REG_OFDM_0_XA_TX_IQ_IMBALANCE, 0xffc00000);

	for (i = 0; i < RTW_OFDM_SWING_TABLE_SIZE; i++) {
		val32 = rtw8723b_ofdm_swing_table[i];

		if (val32 >= 0x100000)
			val32 >>= 22;

		if (val32 == swing)
			break;
	}

	if (i >= RTW_OFDM_SWING_TABLE_SIZE)
		i = 30;

	return i;
}

/* adapted from vendor: halrf_powertracking_ce.c
 * function: get_cck_swing_index
 */
static u8 rtw8723b_default_cck_index(struct rtw_dev *rtwdev)
{
	printk("%s begin", __func__);

	u8 i;
	u8 swing;

	swing = rtw_read8(rtwdev, rtw8723b_cck_pwr_regs[0]);

	for (i = 0; i < RTW_CCK_SWING_TABLE_SIZE; i++) {
		if (rtw8732b_cck_swing_table_ch1_ch13[i][0] == swing)
			break;
	}

	if (i >= RTW_CCK_SWING_TABLE_SIZE)
		i = 20;

	return i;
}

/* vendor: halrf_powertracking_ce.c
 * function: odm_txpowertracking_thermal_meter_init
 */
static void rtw8723b_pwrtrack_init(struct rtw_dev *rtwdev)
{
	printk("%s begin", __func__);

	struct rtw_dm_info *dm_info = &rtwdev->dm_info;
	u8 path;

	dm_info->default_ofdm_index = rtw8723b_default_ofdm_index(rtwdev);
	dm_info->default_cck_index = rtw8723b_default_cck_index(rtwdev);
	// dm_info->default_ofdm_index = 30;
	// dm_info->default_cck_index = 20;

	for (path = RF_PATH_A; path < rtwdev->hal.rf_path_num; path++) {
		ewma_thermal_init(&dm_info->avg_thermal[path]); /* TODO: check this */
		dm_info->delta_power_index[path] = 0;
	}
	dm_info->pwr_trk_triggered = false;
	dm_info->pwr_trk_init_trigger = true;
	dm_info->thermal_meter_k = rtwdev->efuse.thermal_meter_k; /* TODO: check this */
	dm_info->txagc_remnant_cck = 0;
	dm_info->txagc_remnant_ofdm[RF_PATH_A] = 0;
}

#if 0
/* adapted from vendor function: phy_SpurCalibration_8723B
 */
static bool rtw8723b_check_spur_ov_thres(struct rtw_dev *rtwdev,
					 u32 freq, u32 thres)
{
	printk("%s begin", __func__);

	bool ret = false;

	rtw_write32(rtwdev, REG_ANALOG_P4, DIS_3WIRE);
	rtw_write32(rtwdev, REG_PSDFN, freq);
	rtw_write32(rtwdev, REG_PSDFN, START_PSD | freq);

	msleep(30);
	if (rtw_read32(rtwdev, REG_PSDRPT) >= thres)
		ret = true;

	rtw_write32(rtwdev, REG_PSDFN, freq);
	rtw_write32(rtwdev, REG_ANALOG_P4, EN_3WIRE);

	return ret;
}

/* adapted from vendor function: phy_SpurCalibration_8723B
 */
static void rtw8723b_cfg_notch(struct rtw_dev *rtwdev, u8 channel, bool notch)
{
	printk("%s begin", __func__);

	if (!notch) {
		/* TODO: The code in this block has been blindly copied from
		 * rtw8703b.c for now.
		 */
		rtw_write32_mask(rtwdev, REG_OFDM0_RXDSP, BIT_MASK_RXDSP, 0x1f);
		rtw_write32_mask(rtwdev, REG_OFDM0_RXDSP, BIT_EN_RXDSP, 0x0);
		rtw_write32(rtwdev, REG_OFDM1_CSI1, 0x00000000);
		rtw_write32(rtwdev, REG_OFDM1_CSI2, 0x00000000);
		rtw_write32(rtwdev, REG_OFDM1_CSI3, 0x00000000);
		rtw_write32(rtwdev, REG_OFDM1_CSI4, 0x00000000);
		rtw_write32_mask(rtwdev, REG_OFDM1_CFOTRK, BIT_EN_CFOTRK, 0x0);
		return;
	}

	switch (channel) {
	case 5:
		fallthrough;
	case 13:
		rtw_write32_mask(rtwdev, REG_OFDM0_RXDSP, BIT_MASK_RXDSP, 0xb);
		rtw_write32_mask(rtwdev, REG_OFDM0_RXDSP, BIT_EN_RXDSP, 0x1);
		rtw_write32(rtwdev, REG_OFDM1_CSI1, 0x06000000);
		rtw_write32(rtwdev, REG_OFDM1_CSI2, 0x00000000);
		rtw_write32(rtwdev, REG_OFDM1_CSI3, 0x00000000);
		rtw_write32(rtwdev, REG_OFDM1_CSI4, 0x00000000);
		rtw_write32_mask(rtwdev, REG_OFDM1_CFOTRK, BIT_EN_CFOTRK, 0x1);
		break;
	case 6:
		rtw_write32_mask(rtwdev, REG_OFDM0_RXDSP, BIT_MASK_RXDSP, 0x4);
		rtw_write32_mask(rtwdev, REG_OFDM0_RXDSP, BIT_EN_RXDSP, 0x1);
		rtw_write32(rtwdev, REG_OFDM1_CSI1, 0x00000600);
		rtw_write32(rtwdev, REG_OFDM1_CSI2, 0x00000000);
		rtw_write32(rtwdev, REG_OFDM1_CSI3, 0x00000000);
		rtw_write32(rtwdev, REG_OFDM1_CSI4, 0x00000000);
		rtw_write32_mask(rtwdev, REG_OFDM1_CFOTRK, BIT_EN_CFOTRK, 0x1);
		break;
	case 7:
		rtw_write32_mask(rtwdev, REG_OFDM0_RXDSP, BIT_MASK_RXDSP, 0x3);
		rtw_write32_mask(rtwdev, REG_OFDM0_RXDSP, BIT_EN_RXDSP, 0x1);
		rtw_write32(rtwdev, REG_OFDM1_CSI1, 0x00000000);
		rtw_write32(rtwdev, REG_OFDM1_CSI2, 0x00000000);
		rtw_write32(rtwdev, REG_OFDM1_CSI3, 0x00000000);
		rtw_write32(rtwdev, REG_OFDM1_CSI4, 0x06000000);
		rtw_write32_mask(rtwdev, REG_OFDM1_CFOTRK, BIT_EN_CFOTRK, 0x1);
		break;
	case 8:
		rtw_write32_mask(rtwdev, REG_OFDM0_RXDSP, BIT_MASK_RXDSP, 0xa);
		rtw_write32_mask(rtwdev, REG_OFDM0_RXDSP, BIT_EN_RXDSP, 0x1);
		rtw_write32(rtwdev, REG_OFDM1_CSI1, 0x00000000);
		rtw_write32(rtwdev, REG_OFDM1_CSI2, 0x00000000);
		rtw_write32(rtwdev, REG_OFDM1_CSI3, 0x00000000);
		rtw_write32(rtwdev, REG_OFDM1_CSI4, 0x00000380);
		rtw_write32_mask(rtwdev, REG_OFDM1_CFOTRK, BIT_EN_CFOTRK, 0x1);
		break;
	case 14:
		rtw_write32_mask(rtwdev, REG_OFDM0_RXDSP, BIT_MASK_RXDSP, 0x5);
		rtw_write32_mask(rtwdev, REG_OFDM0_RXDSP, BIT_EN_RXDSP, 0x1);
		rtw_write32(rtwdev, REG_OFDM1_CSI1, 0x00000000);
		rtw_write32(rtwdev, REG_OFDM1_CSI2, 0x00000000);
		rtw_write32(rtwdev, REG_OFDM1_CSI3, 0x00000000);
		rtw_write32(rtwdev, REG_OFDM1_CSI4, 0x00180000);
		rtw_write32_mask(rtwdev, REG_OFDM1_CFOTRK, BIT_EN_CFOTRK, 0x1);
		break;
	default:
		rtw_warn(rtwdev,
			 "Bug: Notch filter enable called for channel %u!",
			 channel);
		rtw_write32_mask(rtwdev, REG_OFDM0_RXDSP, BIT_EN_RXDSP, 0x0);
		rtw_write32_mask(rtwdev, REG_OFDM1_CFOTRK, BIT_EN_CFOTRK, 0x0);
		break;
	}
}

/* adapted from vendor function: phy_SpurCalibration_8723B
 */
static void rtw8723b_spur_cal(struct rtw_dev *rtwdev, u8 channel)
{
	printk("%s begin", __func__);

	bool notch;
	u32 freq;

	if (channel == 5) {
		freq = FREQ_CH5;
	} else if (channel == 6) {
		freq = FREQ_CH6;
	} else if (channel == 7) {
		freq = FREQ_CH7;
	} else if (channel == 8) {
		freq = FREQ_CH8;
	} else if (channel == 13) {
		freq = FREQ_CH13;
	} else if (channel == 14) {
		freq = FREQ_CH14;
	} else {
		rtw8723b_cfg_notch(rtwdev, channel, false);
		return;
	}

	notch = rtw8723b_check_spur_ov_thres(rtwdev, freq, SPUR_THRES);
	rtw8723b_cfg_notch(rtwdev, channel, notch);
}
#endif


#define RXDMA_AGG_MODE_EN BIT(1)
static void rtw8723b_rx_aggregation_switch(struct rtw_dev * rtwdev, bool enable)
{
	/* NOTE: enable/disable RX aggregation
	 *
	 * for testing purposes
	 */

	u8 dma;
	u8 rx_agg_ctrl;

	dma = rtw_read8(rtwdev, 0x010c);
	rx_agg_ctrl = rtw_read8(rtwdev,  0x0290);

	if (enable) {
		dma |= BIT_RXDMA_AGG_EN;
		rx_agg_ctrl |= RXDMA_AGG_MODE_EN;
	} else {
		dma &= ~BIT_RXDMA_AGG_EN;
		rx_agg_ctrl &= ~RXDMA_AGG_MODE_EN;
	}

	rtw_write8(rtwdev, 0x010c, dma);
	rtw_write8(rtwdev, 0x0290, rx_agg_ctrl);
}

/* adapted from: _InitPowerOn_8723BS (steps after calling cardEnable)
 */
static void rtw8723b_post_enable_flow(struct rtw_dev *rtwdev)
{
	/* these two are also done in card_enable_flow
	 * we cab probably remove them
	 */
	rtw_write8_set(rtwdev, 0x0049, BIT(1));
	rtw_write8_set(rtwdev, 0x0063, BIT(1));

	rtw_write16_set(rtwdev, REG_APS_FSMCO, BIT_EN_PDN);

	rtw_write8(rtwdev, REG_CR, 0x00);

	/* Enable MAC DMA/WMAC/SCHEDULE/SEC block */
	rtw_write16_set(rtwdev, REG_CR, MAC_TRX_ENABLE | BIT_MAC_SEC_EN |
	BIT_32K_CAL_TMR_EN);
}

/* vendor: hal/rtl8723b/rtl8723b_phycfg.c
 * function: PHY_BBConfig8723B
 */
static void rtw8723b_phy_bb_config(struct rtw_dev *rtwdev)
{
	/* NOTE: this func is ready! */

	u8 xtal_cap;

	/* Enable BB and RF */
	rtw_write16_set(rtwdev, REG_SYS_FUNC_EN,
			BIT_FEN_EN_25_1 | BIT_FEN_BB_GLB_RST | BIT_FEN_BB_RSTB);

	if (rtw_hci_type(rtwdev) == RTW_HCI_TYPE_USB)
		rtw_write32(rtwdev, REG_BB_SEL_BTG, 0x0);
	else
		rtw_write32(rtwdev, REG_BB_SEL_BTG, 0x280);

	rtw_write8_set(rtwdev, REG_RF_CTRL,
		       BIT_RF_EN | BIT_RF_RSTB | BIT_RF_SDM_RSTB);
	usleep_range(10, 11);
	rtw_write_rf(rtwdev, RF_PATH_A, RF_WLINT, RFREG_MASK, 0x0780);
	rtw_write8(rtwdev, REG_SYS_FUNC_EN,
		   BIT_FEN_PPLL | BIT_FEN_PCIEA | BIT_FEN_DIO_PCIE |
		   BIT_FEN_BB_GLB_RST | BIT_FEN_BB_RSTB); /* 0xe3 */
	rtw_write8(rtwdev, REG_AFE_CTRL1 + 1, 0x80);

	// rtw_load_table(rtwdev, rtwdev->chip->bb_tbl);
	// rtw_load_table(rtwdev, rtwdev->chip->agc_tbl);

	xtal_cap = rtwdev->efuse.crystal_cap & 0x3f;
	rtw_write32_mask(rtwdev,  REG_AFE_CTRL3, BIT_MASK_XTAL,
			 xtal_cap | (xtal_cap << 6));
}

/* vendor: hal/rtl8723b/rtl8723b_rf6052.c
 * function: PHY_RF6052_Config8723B
 */
static void rtw8723b_phy_rf6052_config(struct rtw_dev *rtwdev)
{
	/* NOTE: this func is ready! */

	struct rtw_hal *hal = &rtwdev->hal;
	printk("%s: hal->rf_path_num=%d\n", __func__, hal->rf_path_num);

	u8 path;
	u32 val32, mask;
	u32 intf_s, intf_oe, hssi_2;

	for (path = RF_PATH_A; path < hal->rf_path_num; path++) {
		printk("%s(): path=%d\n", __func__, path);
		switch (path) {
		case RF_PATH_A:
			intf_s = REG_FPGA0_XA_RF_SW_CTRL;
			intf_oe = REG_FPGA0_XA_RF_INT_OE;
			hssi_2 = REG_FPGA0_XA_HSSI_PARM2;
			mask = RFSI_RFENV;
			break;
		case RF_PATH_B:
			intf_s = REG_FPGA0_XB_RF_SW_CTRL;
			intf_oe = REG_FPGA0_XB_RF_INT_OE;
			hssi_2 = REG_FPGA0_XB_HSSI_PARM2;
			mask = RFSI_RFENV << 16;
			break;
		default:
			rtw_err(rtwdev, "invalid rf path %c\n", path + 'A');
			return;
		}

		val32 = rtw_read32_mask(rtwdev, intf_s, mask);

		rtw_write32_mask(rtwdev, intf_oe, RFSI_RFENV << 16, 0x1);
		udelay(1);

		rtw_write32_mask(rtwdev, intf_oe, RFSI_RFENV, 0x1);
		udelay(1);

		rtw_write32_mask(rtwdev, hssi_2, HSSI_3WIRE_ADDR_LEN, 0x0);
		udelay(1);

		rtw_write32_mask(rtwdev, hssi_2, HSSI_3WIRE_DATA_LEN, 0x0);
		udelay(1);

		/* NOTE: path A only, there is no table for path B
		 * vendor driver also uses radio_a table for both paths
		 */
		//rtw_load_table(rtwdev, rtwdev->chip->rf_tbl[RF_PATH_A]);

		rtw_write32_mask(rtwdev, intf_s, mask, val32);
	}

	/* 3 Configuration of Tx Power Tracking */
	/* NOTE: reads only pwr track tables into memory in the vendor driver,
	 * we define them directly in rtw8723b_rtw_pwr_track_tbl
	 */
}

/* vendor: hal/rtl8723b/rtl8723b_phycfg.c
 * function: PHY_RFConfig8723B
 */
static void rtw8723b_phy_lck(struct rtw_dev *rtwdev)
{
	/* NOTE: this func is ready! */

	rtw_write_rf(rtwdev, RF_PATH_A, 0xb0, RFREG_MASK, 0xdfbe0);
	rtw_write_rf(rtwdev, RF_PATH_A, RF_CFGCH, RFREG_MASK, 0x8c01);
	mdelay(200); /* rtl8xxxu uses msleep(200) */
	rtw_write_rf(rtwdev, RF_PATH_A, 0xb0, RFREG_MASK, 0xdffe0);
}

/* vendor: hal/rtl8723b/rtl8723b_phycfg.c
 * function: PHY_RFConfig8723B
 */
static void rtw8723b_phy_rf_config(struct rtw_dev *rtwdev)
{
	/* NOTE: this func is ready! */

	rtw8723b_phy_rf6052_config(rtwdev);

	rtw8723b_phy_lck(rtwdev); /* TODO: do wee need this? */
}

static void rtw8723b_init_available_page_threshold(struct rtw_dev *rtwdev)
{
	const struct rtw_chip_info *chip = rtwdev->chip;
	struct rtw_fifo_conf *fifo = &rtwdev->fifo;
	const struct rtw_page_table *pg_tbl = NULL;
	u16 hq_threshold, nq_threshold, lq_threshold;
	u16 pubq_num;
	/* Only initialize these page thresholds for SDIO devices.
	 * PCIe and USB handle TX FIFO/thresholds differently (DMA/host
	 * scheduling) and writing these registers on those buses can be
	 * unnecessary or counter-productive.
	 */
	if (rtw_hci_type(rtwdev) != RTW_HCI_TYPE_SDIO)
		return;

	pg_tbl = &chip->page_table[0]; /* SDIO */

	/* fifo must be initialized before this is called */
	if (fifo->acq_pg_num == 0)
		return;

	/* ensure we don't underflow if tables are misconfigured */
	if (fifo->acq_pg_num <= (pg_tbl->hq_num + pg_tbl->lq_num +
				   pg_tbl->nq_num + pg_tbl->exq_num +
				   pg_tbl->gapq_num))
		return;

	pubq_num = fifo->acq_pg_num - pg_tbl->hq_num - pg_tbl->lq_num -
		   pg_tbl->nq_num - pg_tbl->exq_num - pg_tbl->gapq_num;

	hq_threshold = (pubq_num + pg_tbl->hq_num + 1) >> 1;
	hq_threshold |= (hq_threshold << 8);

	nq_threshold = (pubq_num + pg_tbl->nq_num + 1) >> 1;
	nq_threshold |= (nq_threshold << 8);

	lq_threshold = (pubq_num + pg_tbl->lq_num + 1) >> 1;
	lq_threshold |= (lq_threshold << 8);

	rtw_write16(rtwdev, 0x218, hq_threshold);
	rtw_write16(rtwdev, 0x21a, nq_threshold);
	rtw_write16(rtwdev, 0x21c, lq_threshold);
}

static void rtw8723b_init_queue_reserved_page(struct rtw_dev *rtwdev)
{
	/* handled by mac.c:__priority_queue_cfg_legacy */

	/* vendor code not handled by  mac.c:__priority_queue_cfg_legacy
	 * TODO: probably this is not needed or handled somewhere else in rtw88?
	 * also, there is no trace of this in rtl8xxxu driver
	 */
	rtw8723b_init_available_page_threshold(rtwdev);
}

static void rtw8723b_init_tx_buffer_boundary(struct rtw_dev *rtwdev)
{
	/* NOTE: done also in mac.c:__priority_queue_cfg_legacy ? */

	u8 val8 = TX_TOTAL_PAGE_NUMBER_8723B + 1; /* 0xf7 */

	rtw_write8(rtwdev, REG_TXPKTBUF_BCNQ_BDNY_8723B, val8);
	rtw_write8(rtwdev, REG_TXPKTBUF_MGQ_BDNY_8723B, val8);
	rtw_write8(rtwdev, REG_TXPKTBUF_WMAC_LBK_BF_HD_8723B, val8);
	rtw_write8(rtwdev, REG_TRXFF_BNDY, val8);
	rtw_write8(rtwdev, REG_TDECTRL + 1, val8);
}

static void rtw8723b_init_llt_table(struct rtw_dev *rtwdev)
{
	/* vendor functionality handled in
	 *
	 * mac.c:__priority_queue_cfg_legacy
	 *
	 */
}

static void rtw8723b_init_page_boundary(struct rtw_dev *rtwdev)
{
	 /* NOTE: this is also done in __priority_queue_cfg_legacy,
	  * maybe we can remove it
	  */
	rtw_write16(rtwdev, REG_TRXFF_BNDY + 2, 0x4000 - REPORT_BUF - 1);
}

static void rtw8723b_init_transfer_page_size(struct rtw_dev *rtwdev)
{
	rtw_write8(rtwdev, REG_PBP, 0x11);
}

static void rtw8723b_init_driver_info_size(struct rtw_dev *rtwdev)
{
	/* NOTE: also is done in rtw_drv_info_cfg */
	rtw_write8(rtwdev, REG_RX_DRVINFO_SZ, PHY_STATUS_SIZE);
}

static void rtw8723b_init_network_type(struct rtw_dev *rtwdev)
{
	u32 val32;

	val32 = rtw_read32(rtwdev, REG_CR);
	val32 = (val32 & ~MASK_NETTYPE) | _NETTYPE(NT_LINK_AP);
	rtw_write32(rtwdev, REG_CR, val32);
}

static void rtw8723b_init_wmac_setting(struct rtw_dev *rtwdev)
{
	rtw_write32(rtwdev, REG_RCR, WLAN_RCR_CFG);

	rtw_write32(rtwdev, REG_MAR, 0xffffffff);
	rtw_write32(rtwdev, REG_MAR + 4, 0xffffffff);

	rtw_write16(rtwdev, REG_RXFLTMAP2, WLAN_RX_FILTER2);
	rtw_write16(rtwdev, REG_RXFLTMAP1, WLAN_RX_FILTER1);
	rtw_write16(rtwdev, REG_RXFLTMAP0, WLAN_RX_FILTER0);
}

static void rtw8723b_init_adaptive_ctrl(struct rtw_dev *rtwdev)
{
	rtw_write32_mask(rtwdev, REG_RRSR, 0xfffff, 0xffff1);
	rtw_write16(rtwdev, REG_RETRY_LIMIT, 0x3030);
}

static void rtw8723b_init_edca(struct rtw_dev *rtwdev)
{
	rtw_write16(rtwdev, REG_SPEC_SIFS, 0x100a);
	rtw_write16(rtwdev, REG_MAC_SPEC_SIFS, 0x100a);
	rtw_write16(rtwdev, REG_SIFS, 0x100a);
	rtw_write16(rtwdev, REG_SIFS + 2, 0x100a);

	/* TXOP */
	rtw_write32(rtwdev, REG_EDCA_BE_PARAM, 0x005EA42B);
	rtw_write32(rtwdev, REG_EDCA_BK_PARAM, 0x0000A44F);
	rtw_write32(rtwdev, REG_EDCA_VI_PARAM, 0x005EA324);
	rtw_write32(rtwdev, REG_EDCA_VO_PARAM, 0x002FA226);
}

static void rtw8723b_init_retry_function(struct rtw_dev *rtwdev)
{
	rtw_write8_set(rtwdev, REG_FWHW_TXQ_CTRL, BIT(7));
	rtw_write8(rtwdev, REG_ACKTO, 0x40);
}

static void rtw8723b_init_beacon_parameters(struct rtw_dev *rtwdev)
{
	rtw_write16(rtwdev, REG_BCN_CTRL,
		    BIT_DIS_TSF_UDT | (BIT_DIS_TSF_UDT << 8) | BIT_EN_BCN_FUNCTION);
	rtw_write8(rtwdev, REG_TBTT_PROHIBIT, TBTT_PROHIBIT_SETUP_TIME);
	rtw_write8(rtwdev, REG_TBTT_PROHIBIT + 1,
		  TBTT_PROHIBIT_HOLD_TIME_STOP_BCN & 0xff);
	rtw_write8(rtwdev, REG_TBTT_PROHIBIT + 2,
		   (rtw_read8(rtwdev, REG_TBTT_PROHIBIT + 2) & 0xf0) |
		   (TBTT_PROHIBIT_HOLD_TIME_STOP_BCN >> 8));

	rtw_write8(rtwdev, REG_BCNDMATIM, WLAN_BCN_DMA_TIME);
	/* Suggested by designer timchen. Change beacon AIFS to the largest number */
	/* beacause test chip does not contension before sending beacon. by tynli. 2009.11.03 */
	rtw_write16(rtwdev, REG_BCNTCFG, 0x660F);
}

static void rtw8723b_init_burst_pkt_len(struct rtw_dev *rtwdev)
{
	rtw_write8_set(rtwdev, REG_SINGLE_AMPDU_CTRL, BIT_EN_SINGLE_APMDU);
	rtw_write8(rtwdev, REG_RX_PKT_LIMIT, 0x18);
	rtw_write8(rtwdev, REG_MAX_AGGR_NUM, 0x1F);
	rtw_write8(rtwdev, REG_PIFS, 0x00);
	rtw_write8_clr(rtwdev, REG_FWHW_TXQ_CTRL, BIT(7));
	rtw_write8(rtwdev, REG_AMPDU_MAX_TIME, 0x70);
}

static void rtw8723b_init_antenna_selection(struct rtw_dev *rtwdev)
{
	/* Let 8051 take control antenna settting */
	rtw_write8(rtwdev, REG_LEDCFG2, WLAN_ANT_SEL);
}

static void rtl8xxxu_init_antenna_selection(struct rtw_dev *rtwdev)
{
	/* NOTE: adapted from rtl8xxu driver function
	 * rtl8723bu_phy_init_antenna_selection
	 *
	 * for testing purposes
	 */

	u32 val32;

	val32 = rtw_read32(rtwdev, REG_PAD_CTRL1);
	val32 &= ~(BIT(20) | BIT(24));
	rtw_write32(rtwdev, REG_PAD_CTRL1, val32);

	val32 = rtw_read32(rtwdev, REG_GPIO_MUXCFG);
	val32 &= ~BIT(4);
	rtw_write32(rtwdev, REG_GPIO_MUXCFG, val32);

	val32 = rtw_read32(rtwdev, REG_GPIO_MUXCFG);
	val32 |= BIT(3);
	rtw_write32(rtwdev, REG_GPIO_MUXCFG, val32);

	val32 = rtw_read32(rtwdev, REG_LED_CFG);
	val32 |= BIT(24);
	rtw_write32(rtwdev, REG_LED_CFG, val32);

	val32 = rtw_read32(rtwdev, REG_LED_CFG);
	val32 &= ~BIT(23);
	rtw_write32(rtwdev, REG_LED_CFG, val32);

	val32 = rtw_read32(rtwdev, 0x0944);
	val32 |= (BIT(0) | BIT(1));
	rtw_write32(rtwdev, 0x0944, val32);

	val32 = rtw_read32(rtwdev, 0x0930);
	val32 &= 0xffffff00;
	val32 |= 0x77;
	rtw_write32(rtwdev, 0x0930, val32);

	val32 = rtw_read32(rtwdev, 0x0038);
	val32 |= BIT(11);
	rtw_write32(rtwdev, 0x0038, val32);
}

#define RF_AC	0x00

/* vendor function: _phy_lc_calibrate_8723b
 * (the vendor function is always called with is2T == false)
 */
static void rtw8723b_lck(struct rtw_dev *rtwdev)
{
	u8 val_ctx;
	u32 rf_mode = 0, lc_cal;
	u8 rf_val;
	int ret;

	val_ctx = rtw_read8(rtwdev, REG_CTX);

	if ((val_ctx & BIT_MASK_CTX_TYPE) != 0)
		rtw_write8(rtwdev, REG_CTX, val_ctx & ~BIT_MASK_CTX_TYPE);
	else
		rtw_write8(rtwdev, REG_TXPAUSE, 0xff);

	// This is probably not needed, but keep it for now
	if ((val_ctx & BIT_MASK_CTX_TYPE) != 0) {
		/* 1. Read original RF mode */
		rf_mode = rtw_read_rf(rtwdev, RF_PATH_A, RF_AC, MASK12BITS);
		/* 2. Set RF mode = standby mode */
		rtw_write_rf(rtwdev, RF_PATH_A, RF_AC, MASK12BITS, (rf_mode & 0x8ffff) | 0x10000);
	}

	/* 3. Read RF reg18 */
	lc_cal = rtw_read_rf(rtwdev, RF_PATH_A, RF_CFGCH, MASK12BITS);

	/* 4. Set LC calibration begin	bit15 */
	rtw_write_rf(rtwdev, RF_PATH_A, 0xb0, RFREG_MASK, 0xdfbe0); /* LDO ON */
	rtw_write_rf(rtwdev, RF_PATH_A, RF_CFGCH, MASK12BITS, lc_cal | BIT_LCK);

	// NOTE: vendor driver just uses mdelay(100)
	ret = read_poll_timeout(rtw_read_rf, rf_val, rf_val != 0x1,
				10000, 1000000, false,
			 rtwdev, RF_PATH_A, RF_CFGCH, BIT_LCK);
	if (ret)
		rtw_warn(rtwdev, "failed to poll LCK status bit\n");

	rtw_write_rf(rtwdev, RF_PATH_A, 0xb0, RFREG_MASK, 0xdffe0); /* LDO OFF */

	// /* channel 10 LC calibration issue for 8723bs with 26M xtal */
	// if (p_dm->support_interface == ODM_ITRF_SDIO && p_dm->package_type >= 0x2)
	// 	rtw_write_rf(rtwdev, RF_PATH_A, RF_CFGCH, MASK12BITS, lc_cal);

	/* Restore original situation */
	if ((val_ctx & BIT_MASK_CTX_TYPE) != 0) {
		rtw_write8(rtwdev, REG_CTX, val_ctx);

		// this is probably not needed, but keep it for now
		rtw_write_rf(rtwdev, RF_PATH_A, RF_AC, MASK12BITS, rf_mode);
	} else {
		rtw_write8(rtwdev, REG_TXPAUSE, 0x00);
	}
}

static int rtw8723b_mac_init(struct rtw_dev *rtwdev)
{
	/* TODO: needed? there is no trace of this in the vendor
	 * drivers rtl8723bs/cs, but rtw8703b uses rtw8723x_mac_init
	 * which contais this
	 */
	//rtw_write32(rtwdev, REG_TCR, BIT_TCR_CFG); /* BIT_TCR_CFG = 0x3200 */

	rtw8723b_init_wmac_setting(rtwdev);

	/* TODO: needed?
	 * there is no trace of the following two in the vendor
	 * drivers 8723bs/8723cs, but rtw8703b uses rtw8723x_mac_init
	 * which contains these. Also, rtlwifi/8723be sets them as well
	 * however, the rtlwifi driver for 8723be uses it.
	 */
	rtw_write32(rtwdev, REG_INT_MIG, 0);
	rtw_write32(rtwdev, REG_MCUTST_1, 0x0);

	rtw_write8(rtwdev, REG_MISC_CTRL, 0x3); /* CCA */
	rtw_write8(rtwdev, REG_2ND_CCA_CTRL, 0x0); /* hpfan_todo: 2nd CCA related */

	return 0;
}

/* based on
 * vendor: hal/rtl8723b/sdio/sdio_halinit.c
 * function: rtl8723bs_hal_init
 *
 * NOTE: We should also check if anything is already done in sdio.c or elsewhere.
 */
static void rtw8723b_phy_set_param(struct rtw_dev *rtwdev)
{
	printk("%s begin", __func__);

	u32 val32;

	rtw8723b_post_enable_flow(rtwdev);

	// rtw_load_table(rtwdev, rtwdev->chip->mac_tbl);

	rtw8723b_phy_bb_config(rtwdev);
	rtw8723b_phy_rf_config(rtwdev);

	/* NOTE: if we uncomment this, do not forget to comment
	 * the individual load_table() calls */
	rtw_phy_load_tables(rtwdev);

	/* enable CCK and OFDM block */
	rtw_write32_set(rtwdev, REG_FPGA0_RFMOD, BIT_CCKEN | BIT_OFDMEN);

	rtw8723b_init_queue_reserved_page(rtwdev);
	rtw8723b_init_tx_buffer_boundary(rtwdev);
	rtw8723b_init_llt_table(rtwdev);

	// _InitQueuePriority(padapter);

	rtw8723b_init_page_boundary(rtwdev);
	rtw8723b_init_transfer_page_size(rtwdev);
	rtw8723b_init_driver_info_size(rtwdev);
	rtw8723b_init_network_type(rtwdev);

	rtw8723b_init_wmac_setting(rtwdev);

	rtw8723b_init_adaptive_ctrl(rtwdev);
	rtw8723b_init_edca(rtwdev);
	rtw8723b_init_retry_function(rtwdev);

	/* Set up RX aggregation. sdio.c also sets DMA mode, but not
	 * the burst parameters.
	 */
	rtw_write8(rtwdev, REG_RXDMA_MODE,
		   BIT_DMA_MODE |
		   FIELD_PREP_CONST(BIT_MASK_AGG_BURST_NUM, AGG_BURST_NUM) |
		   FIELD_PREP_CONST(BIT_MASK_AGG_BURST_SIZE, AGG_BURST_SIZE));

	rtw8723b_init_beacon_parameters(rtwdev);
	rtw8723b_init_burst_pkt_len(rtwdev);

	// done in rtw8723b_mac_init
	// rtw_write8(rtwdev, REG_MISC_CTRL, 0x3); /* CCA */
	// rtw_write8(rtwdev, REG_2ND_CCA_CTRL, 0x0); /* hpfan_todo: 2nd CCA related */

	rtw_write8(rtwdev, REG_SLOT, WLAN_SLOT_TIME);

	/* disable BAR */
	rtw_write32(rtwdev, REG_BAR_MODE_CTRL, WLAN_BAR_VAL);

	/* set 0x0 to 0xFF by tynli. Default enable HW SEQ NUM. */
	rtw_write8(rtwdev, REG_HWSEQ_CTRL, 0xff);

	/*
	 * Configure SDIO TxRx Control to enable Rx DMA timer masking.
	 * 2010.02.24.
	 * Only clear necessary bits 0x0[2:0] and 0x2[15:0] and keep 0x0[15:3]
	 * 2015.03.19.
	 */
	// NOTE: also done by the mac poer on rtw88 routines, we can remove it probably here
	val32 = rtw_read32(rtwdev, REG_SDIO_TX_CTRL);
	val32 &= 0x0000fff8;
	rtw_write32(rtwdev, REG_SDIO_TX_CTRL, val32);

	rtw_write16(rtwdev, REG_ATIMWND, 0x2);

	rtw8723b_init_antenna_selection(rtwdev);

	/* NOTE: the following is also done in rtw8723b_post_enable_flow */
	/* Enable MACTXEN/MACRXEN block */
	rtw_write8_set(rtwdev, REG_CR, BIT_MACTXEN | BIT_MACRXEN);

	rtw_write8(rtwdev, REG_NAV_UPPER, 0xeb); /* ((30000 + 128 - 1) / 128) */

	/* ack for xmit mgmt frames */
	rtw_write32_set(rtwdev, REG_FWHW_TXQ_CTRL, BIT(12));

	rtw_phy_init(rtwdev);

// 	/* TODO: If we set cck_pd_set in chip_ops in the future we might need
// 	 * the following two lines (taken from rtw8723d.c). If we uncomment these
// 	 * we need to check if the registers and constants match rtl8723b.
// 	 */
// 	// rtwdev->dm_info.cck_pd_default = rtw_read8(rtwdev, REG_CSRATIO) & 0x1f;
// 	// rtw_write16_set(rtwdev, REG_TXDMA_OFFSET_CHK, BIT_DROP_DATA_EN);

	rtw8723x_lck(rtwdev); /* TODO: ? */
	//rtw8723b_lck(rtwdev);

	rtw_write32_mask(rtwdev, REG_OFDM0_XAAGC1, MASKBYTE0, 0x50);
	rtw_write32_mask(rtwdev, REG_OFDM0_XAAGC1, MASKBYTE0, 0x20);

	rtw8723b_pwrtrack_init(rtwdev);
}

/* based on vendor functions
 * hal/rtl8723b/rtl8723b_phycfg.c: phy_SwChnl8723B
 * hal/rtl8723b/rtl8723b_phycfg.c: phy_PostSetBwMode8723B
 * hal/rtl8723b/rtl8723b_rf6052.c: PHY_RF6052SetBandwidth8723B
 */
static void rtw8723b_set_channel_rf(struct rtw_dev *rtwdev, u8 channel, u8 bw)
{
	printk("%s begin", __func__);

	/* NOTE: this func is ready! */

	u32 rf_cfgch_a;
	u32 rf_cfgch_b;

	rf_cfgch_a = rtw_read_rf(rtwdev, RF_PATH_A, RF_CFGCH, RFREG_MASK);
	rf_cfgch_b = rtw_read_rf(rtwdev, RF_PATH_B, RF_CFGCH, RFREG_MASK);

	// DEBUG
	printk("channel=0x%x, bw=0x%x\n", channel, bw);
	printk("initial reg value: rf_cfgch_a= 0x%x\n", rf_cfgch_a);
	printk("initial reg value: rf_cfgch_b= 0x%x\n", rf_cfgch_b);


	rf_cfgch_a &= ~RFCFGCH_CHANNEL_MASK;
	rf_cfgch_b &= ~RFCFGCH_CHANNEL_MASK;
	rf_cfgch_a |= (channel & RFCFGCH_CHANNEL_MASK);
	rf_cfgch_b |= (channel & RFCFGCH_CHANNEL_MASK);

	rf_cfgch_a &= ~RFCFGCH_BW_MASK;

	switch (bw) {
	case RTW_CHANNEL_WIDTH_20:
		rf_cfgch_a |= RFCFGCH_BW_20M;
		break;
	case RTW_CHANNEL_WIDTH_40:
		rf_cfgch_a |= RFCFGCH_BW_40M;
		break;
	default:
		break;
	}

	/* the vendor driver writes A value also to B */
	rf_cfgch_b = rf_cfgch_a;

	printk("before writing reg: rf_cfgch_a= 0x%x\n", rf_cfgch_a);
	printk("before writing reg : rf_cfgch_b= 0x%x\n", rf_cfgch_b);

	rtw_write_rf(rtwdev, RF_PATH_A, RF_CFGCH, RFREG_MASK, rf_cfgch_a);
	rtw_write_rf(rtwdev, RF_PATH_B, RF_CFGCH, RFREG_MASK, rf_cfgch_b);

	// DEBUG
	mdelay(100);
	rf_cfgch_a = rtw_read_rf(rtwdev, RF_PATH_A, RF_CFGCH, RFREG_MASK);
	rf_cfgch_b = rtw_read_rf(rtwdev, RF_PATH_B, RF_CFGCH, RFREG_MASK);

	printk("reg after readback: rf_cfgch_a= 0x%x\n", rf_cfgch_a);
	printk("reg after readback : rf_cfgch_b= 0x%x\n", rf_cfgch_b);

	/* NOTE: not called in vendor driver */
	// rtw8723b_spur_cal(rtwdev, channel);

	printk("%s end", __func__);

}

/* based on vendor functions
 * hal/rtl8723b/rtl8723b_phycfg.c: phy_SwChnl8723B
 * hal/rtl8723b/rtl8723b_phycfg.c: phy_PostSetBwMode8723B
 * hal/rtl8723b/rtl8723b_rf6052.c: PHY_RF6052SetBandwidth8723B
 */
static void rtw8723b_set_channel_bb(struct rtw_dev *rtwdev, u8 bw,
				    u8 primary_ch_idx)
{
	/* NOTE: this func is ready! */

	printk("%s begin", __func__);

	switch (bw) {
	case RTW_CHANNEL_WIDTH_20:
		rtw_write32_mask(rtwdev, REG_FPGA0_RFMOD, BIT_MASK_RFMOD, 0x0);
		rtw_write32_mask(rtwdev, REG_FPGA1_RFMOD, BIT_MASK_RFMOD, 0x0);
		rtw_write32_mask(rtwdev, REG_OFDM0_TX_PSD_NOISE,
				 GENMASK(31, 30), 0x0);
		break;
	case RTW_CHANNEL_WIDTH_40:
		rtw_write32_mask(rtwdev, REG_FPGA0_RFMOD, BIT_MASK_RFMOD, 0x1);
		rtw_write32_mask(rtwdev, REG_FPGA1_RFMOD, BIT_MASK_RFMOD, 0x1);
		rtw_write32_mask(rtwdev, REG_CCK0_SYS, BIT_CCK_SIDE_BAND,
				 primary_ch_idx == RTW_SC_20_UPPER ? 1 : 0);
		rtw_write32_mask(rtwdev, REG_OFDM_FA_RSTD_11N, 0xc00,
				 primary_ch_idx == RTW_SC_20_UPPER ? 2 : 1);
		rtw_write32_mask(rtwdev, REG_BB_PWR_SAV5_11N, GENMASK(27, 26),
				 primary_ch_idx == RTW_SC_20_UPPER ? 1 : 2);
		break;
	default:
		break;
	}
}

/* for testing purposes */
static void rtw8723b_dump_rf_reg(struct rtw_dev *rtwdev)
{
	u32 val32, offset;

	printk("%s() ====>\n", __func__);

	for (offset = 0x00; offset <= 0x30; offset++) {
		val32 = rtw_read_rf(rtwdev, RF_PATH_A, offset, 0xffffffff);
		printk(" 0x%02x = 0x%08x\n", offset, val32);
	}

	printk("<==== %s()\n", __func__);

}

static void rtw8723b_set_channel(struct rtw_dev *rtwdev, u8 channel,
				 u8 bw, u8 primary_chan_idx)
{
	/* NOTE: this func is ready! */

	printk("%s begin", __func__);

	// printk("RF reg before\n");
	// rtw8723b_dump_rf_reg(rtwdev);

	rtw8723b_set_channel_rf(rtwdev, channel, bw);
	rtw_set_channel_mac(rtwdev, channel, bw, primary_chan_idx);
	rtw8723b_set_channel_bb(rtwdev, bw, primary_chan_idx);

	// printk("RF reg after\n");
	// rtw8723b_dump_rf_reg(rtwdev);

	printk("%s end", __func__);
}

/* adapted from vendor file hal/phydm/rtl8723b/phydm_rtl8723b.c
 * function: odm_CCKRSSI_8723B
 */
static s8 rtw8723b_cck_rx_power(u8 lna_idx, u8 vga_idx)
{
	/* NOTE: this func is ready! */

	printk("%s begin", __func__);

	s8 rx_power = 0;

	switch (lna_idx) {
	case 6:
		rx_power = -40 - (2 * vga_idx);
		break;
	case 4:
		rx_power = -20 - (2 * vga_idx);
		break;
	case 1:
		rx_power = 0 - (2 * vga_idx);
		break;
	case 0:
		rx_power = 10 - (2 * vga_idx);
		break;
	default:
		break;
	}

	return rx_power;
}

/* vendor: hal/phydm/phydm_phystatus.c
 * function: phydm_rx_phy_status92c_series_parsing (is_cck_rate arm)
 */
static void rtw8723b_query_phy_status_cck(struct rtw_dev *rtwdev, u8 *phy_raw,
					  struct rtw_rx_pkt_stat *pkt_stat)
{
	printk("%s begin", __func__);

	struct phy_status_8703b *phy_status = (struct phy_status_8703b *)phy_raw;
	u8 lna_idx = (phy_status->cck_agc_rpt_ofdm_cfosho_a & 0xE0) >> 5;
	u8 vga_idx = (phy_status->cck_agc_rpt_ofdm_cfosho_a & 0x1F);
	s8 rx_power = rtw8723b_cck_rx_power(lna_idx, vga_idx);
	s8 min_rx_power = -120;

	pkt_stat->bw = RTW_CHANNEL_WIDTH_20;

	pkt_stat->rx_power[RF_PATH_A] = rx_power;
	pkt_stat->rssi = rtw_phy_rf_power_2_rssi(pkt_stat->rx_power, 1);
	pkt_stat->signal_power = max(pkt_stat->rx_power[RF_PATH_A],
				     min_rx_power);
	rtwdev->dm_info.rssi[RF_PATH_A] = pkt_stat->rssi;
}

/* vendor: hal/phydm/phydm_phystatus.c
 * function: phydm_rx_phy_status92c_series_parsing (!is_cck_rate arm)
 */
static void rtw8723b_query_phy_status_ofdm(struct rtw_dev *rtwdev, u8 *phy_raw,
					   struct rtw_rx_pkt_stat *pkt_stat)
{
	printk("%s begin", __func__);

	struct phy_status_8703b *phy_status = (struct phy_status_8703b *)phy_raw;
	struct rtw_dm_info *dm_info = &rtwdev->dm_info;
	s8 val_s8;

	/* TODO: pkt_stat->bw = */

	val_s8 = phy_status->path_agc[RF_PATH_A].gain & 0x3F;
	pkt_stat->rx_power[RF_PATH_A] = (val_s8 * 2) - 110;

	pkt_stat->rssi = rtw_phy_rf_power_2_rssi(pkt_stat->rx_power, 1);
	pkt_stat->rx_snr[RF_PATH_A] = (s8)(phy_status->path_rxsnr[RF_PATH_A] / 2);

	/* signal power reported by HW */
	val_s8 = phy_status->cck_sig_qual_ofdm_pwdb_all >> 1;
	pkt_stat->signal_power = (val_s8 & 0x7f) - 110;

	pkt_stat->rx_evm[RF_PATH_A] = phy_status->stream_rxevm[RF_PATH_A];
	pkt_stat->cfo_tail[RF_PATH_A] = phy_status->path_cfotail[RF_PATH_A];

	dm_info->curr_rx_rate = pkt_stat->rate;
	dm_info->rssi[RF_PATH_A] = pkt_stat->rssi;
	dm_info->rx_snr[RF_PATH_A] = pkt_stat->rx_snr[RF_PATH_A] >> 1;
	dm_info->cfo_tail[RF_PATH_A] = (pkt_stat->cfo_tail[RF_PATH_A] * 5) >> 1;

	val_s8 = (s8)pkt_stat->rx_evm[RF_PATH_A];
	val_s8 = clamp_t(s8, -val_s8 >> 1, 0, 64);
	val_s8 &= 0x3F; /* 64->0: second path of 1SS rate is 64 */
	dm_info->rx_evm_dbm[RF_PATH_A] = val_s8;
}

/* vendor: hal/phydm/phydm_phystatus.c
 * function: phydm_rx_phy_status92c_series_parsing
 */
static void rtw8723b_query_phy_status(struct rtw_dev *rtwdev, u8 *phy_status,
				      struct rtw_rx_pkt_stat *pkt_stat)
{
	printk("%s begin", __func__);

	if (pkt_stat->rate <= DESC_RATE11M)
		rtw8723b_query_phy_status_cck(rtwdev, phy_status, pkt_stat);
	else
		rtw8723b_query_phy_status_ofdm(rtwdev, phy_status, pkt_stat);
}

/* vendor: hal/phydm/halrf/rtl8723b/halrf_8723b_ce.c
 * function: set_iqk_matrix_8723b
 */
static void rtw8723b_set_iqk_matrix_by_result(struct rtw_dev *rtwdev,
					      u32 ofdm_swing, u8 path)
{
	/* NOTE: this func should be ready! */

	printk("%s begin", __func__);

	struct rtw_dm_info *dm_info = &rtwdev->dm_info;
	s32 ele_A, ele_D, ele_C, ele_A_ext;
	s32 iqk_result_x;
	s32 iqk_result_y;
	s32 value32;

	switch (path) {
	default:
	case RF_PATH_A:
		iqk_result_x = dm_info->iqk.result.s1_x;
		iqk_result_y = dm_info->iqk.result.s1_y;
		break;
	case RF_PATH_B:
		iqk_result_x = dm_info->iqk.result.s0_x;
		iqk_result_y = dm_info->iqk.result.s0_y;
		break;
	}

	/* new element D */
	ele_D = OFDM_SWING_D(ofdm_swing);

	/* new element A */
	iqk_result_x = iqkxy_to_s32(iqk_result_x);
	ele_A = iqk_mult(iqk_result_x, ele_D, &ele_A_ext);

	/* new element C */
	iqk_result_y = iqkxy_to_s32(iqk_result_y);
	ele_C = iqk_mult(iqk_result_y, ele_D, NULL);

	switch (path) {
	case RF_PATH_A:
	default:
		/* wirte new elements A, C, D, element B is always 0 */
		value32 = BIT_SET_TXIQ_ELM_ACD(ele_A, ele_C, ele_D);
		rtw_write32(rtwdev, REG_OFDM_0_XA_TX_IQ_IMBALANCE, value32);
		value32 = BIT_SET_TXIQ_ELM_C1(ele_C);
		rtw_write32_mask(rtwdev, REG_TXIQK_MATRIXA_LSB2_11N, MASKH4BITS,
				 value32);
		rtw_write32_mask(rtwdev, REG_OFDM_0_ECCA_THRESHOLD, BIT(24),
				 ele_A_ext);
		break;

	case RF_PATH_B:
		/* wirte new elements A, C, D, element B is always 0 */
		value32 = BIT_SET_TXIQ_ELM_ACD(ele_A, ele_C, ele_D);
		rtw_write32(rtwdev, REG_OFDM_0_XB_TX_IQ_IMBALANCE, value32);
		value32 = BIT_SET_TXIQ_ELM_C1(ele_C);
		rtw_write32_mask(rtwdev, REG_TXIQK_MATRIXB_LSB2_11N, MASKH4BITS,
				 value32);
		rtw_write32_mask(rtwdev, REG_OFDM_0_ECCA_THRESHOLD, BIT(28),
				 ele_A_ext);
		break;
	}
}

/* vendor: hal/phydm/halrf/rtl8723b/halrf_8723b_ce.c
 * function: set_iqk_matrix_8723b
 */
static void rtw8723b_set_iqk_matrix(struct rtw_dev *rtwdev, s8 ofdm_index,
				    u8 path)
{
	/* NOTE: this func should be ready! */

	printk("%s begin", __func__);

	struct rtw_dm_info *dm_info = &rtwdev->dm_info;
	u32 ofdm_swing;

	ofdm_index = clamp_t(s8, ofdm_index, 0, RTW_OFDM_SWING_TABLE_SIZE - 1);

	ofdm_swing = rtw8723b_ofdm_swing_table[ofdm_index];

	if (dm_info->iqk.done) {
		rtw8723b_set_iqk_matrix_by_result(rtwdev, ofdm_swing, path);
		return;
	}

	switch (path) {
	case RF_PATH_A:
	default:
		rtw_write32(rtwdev, REG_OFDM_0_XA_TX_IQ_IMBALANCE, ofdm_swing);
		rtw_write32_mask(rtwdev, REG_TXIQK_MATRIXA_LSB2_11N, MASKH4BITS,
				 0x00);
		rtw_write32_mask(rtwdev, REG_OFDM_0_ECCA_THRESHOLD, BIT(24),
				 0x00);
		break;

	case RF_PATH_B:
		rtw_write32(rtwdev, REG_OFDM_0_XB_TX_IQ_IMBALANCE, ofdm_swing);
		rtw_write32_mask(rtwdev, REG_TXIQK_MATRIXB_LSB2_11N, MASKH4BITS,
				 0x00);
		rtw_write32_mask(rtwdev, REG_OFDM_0_ECCA_THRESHOLD, BIT(28),
				 0x00);
		break;
	}
}

static u8 rtw8723b_iqk_check_tx_failed(struct rtw_dev *rtwdev)
{
	printk("%s begin", __func__);

	/* NOTE: this func is ready! */

	s32 tx_x, tx_y;
	u32 tx_fail;

	rtw_dbg(rtwdev, RTW_DBG_RFK, "[IQK] 0xeac = 0x%x\n",
		rtw_read32(rtwdev, REG_IQK_RES_RY));
	rtw_dbg(rtwdev, RTW_DBG_RFK, "[IQK] 0xe94 = 0x%x, 0xe9c = 0x%x\n",
		rtw_read32(rtwdev, REG_IQK_RES_TX),
		rtw_read32(rtwdev, REG_IQK_RES_TY));
	rtw_dbg(rtwdev, RTW_DBG_RFK,
		"[IQK] 0xe90(before IQK) = 0x%x, 0xe98(after IQK) = 0x%x\n",
		rtw_read32(rtwdev, 0xe90),
		rtw_read32(rtwdev, 0xe98));

	tx_fail = rtw_read32_mask(rtwdev, REG_IQK_RES_RY, BIT_IQK_TX_FAIL);
	tx_x = rtw_read32_mask(rtwdev, REG_IQK_RES_TX, BIT_MASK_RES_TX);
	tx_y = rtw_read32_mask(rtwdev, REG_IQK_RES_TY, BIT_MASK_RES_TY);

	if (!tx_fail && tx_x != IQK_TX_X_ERR && tx_y != IQK_TX_Y_ERR)
		return IQK_TX_OK; // BIT(0)

	rtw_dbg(rtwdev, RTW_DBG_RFK, "[IQK] A TX IQK failed\n");

	return 0;
}

static u8 rtw8723b_iqk_check_rx_failed(struct rtw_dev *rtwdev)
{
	printk("%s begin", __func__);

	/* NOTE: this func is ready! */

	s32 rx_x, rx_y;
	u32 rx_fail;

	rtw_dbg(rtwdev, RTW_DBG_RFK, "[IQK] 0xea4 = 0x%x, 0xeac = 0x%x\n",
		rtw_read32(rtwdev, REG_IQK_RES_RX),
		rtw_read32(rtwdev, REG_IQK_RES_RY));
	rtw_dbg(rtwdev, RTW_DBG_RFK,
		"[IQK] 0xea0(before IQK) = 0x%x, 0xea8(after IQK) = 0x%x\n",
		rtw_read32(rtwdev, 0xea0),
		rtw_read32(rtwdev, 0xea8));

	rx_fail = rtw_read32_mask(rtwdev, REG_IQK_RES_RY, BIT_IQK_RX_FAIL);
	rx_x = rtw_read32_mask(rtwdev, REG_IQK_RES_RX, BIT_MASK_RES_RX);
	rx_y = rtw_read32_mask(rtwdev, REG_IQK_RES_RY, BIT_MASK_RES_RY);
	rx_y = abs(iqkxy_to_s32(rx_y));

	if (!rx_fail && rx_x != IQK_RX_X_ERR && rx_y != IQK_RX_Y_ERR &&
	     rx_x < IQK_RX_X_UPPER && rx_x > IQK_RX_X_LOWER &&
	     rx_y < IQK_RX_Y_LMT)
		return IQK_RX_OK; // BIT(1)

	rtw_dbg(rtwdev, RTW_DBG_RFK, "[IQK] A RX IQK failed\n");

	return 0;
}

/* vendor: hal/phydm/halrf/rtl8723b/halrf_8723b_ce.c
 * function: phy_path_a_iqk_8723b
 */
static u8 rtw8723b_iqk_tx_path_a(struct rtw_dev *rtwdev)
{
	printk("%s begin", __func__);

	/* NOTE: this func is ready! */

	u8 status;
	u32 path_sel;

	rtw_dbg(rtwdev, RTW_DBG_RFK, "[IQK] path A TX IQK!\n");

	/* Save RF path */
	path_sel = rtw_read32(rtwdev, REG_BB_SEL_BTG);

	/* leave IQK mode */
	rtw_write32_mask(rtwdev, REG_FPGA0_IQK_11N, MASKH3BYTES, 0x000000);

	/* enable path A PA in TX IQK mode */
	rtw_write_rf(rtwdev, RF_PATH_A, RF_LUTWE, 0x80000, 0x1);
	rtw_write_rf(rtwdev, RF_PATH_A, RF_RCK_OS, RFREG_MASK, 0x20000);
	rtw_write_rf(rtwdev, RF_PATH_A, RF_TXPA_G1, RFREG_MASK, 0x0003f);
	rtw_write_rf(rtwdev, RF_PATH_A, RF_TXPA_G2, RFREG_MASK, 0xc7f87);

	/* Tx IQK setting */
	rtw_write32(rtwdev, REG_TXIQK_11N, 0x01007c00);
	rtw_write32(rtwdev, REG_RXIQK_11N, 0x01004800);

	/* path-A IQK setting */
	rtw_write32(rtwdev, REG_TXIQK_TONE_A_11N, 0x18008c1c);
	rtw_write32(rtwdev, REG_RXIQK_TONE_A_11N, 0x38008c1c);
	rtw_write32(rtwdev, REG_TX_IQK_TONE_B, 0x38008c1c);
	rtw_write32(rtwdev, REG_RX_IQK_TONE_B, 0x38008c1c);

	rtw_write32(rtwdev, REG_TXIQK_PI_A_11N, 0x821403ea);
	rtw_write32(rtwdev, REG_RXIQK_PI_A_11N, 0x28110000);
	rtw_write32(rtwdev, REG_TXIQK_PI_B, 0x82110000);
	rtw_write32(rtwdev, REG_RXIQK_PI_B, 0x28110000);

	/* LO calibration setting */
	rtw_write32(rtwdev, REG_IQK_AGC_RSP_11N, 0x00462911);

	/* enter IQK mode */
	rtw_write32_mask(rtwdev, REG_FPGA0_IQK_11N, MASKH3BYTES, 0x808000);

	/* ant switch */
	if (rtw_hci_type(rtwdev) == RTW_HCI_TYPE_USB)
		rtw_write32(rtwdev, REG_BB_SEL_BTG, 0x280);
	else
		rtw_write32(rtwdev, REG_BB_SEL_BTG, 0x0);

	/* GNT_BT = 0 */
	rtw_write32(rtwdev, REG_BT_CONTROL_8723B, 0x00000800);

	/* One shot, path A LOK & IQK */
	rtw_write32(rtwdev, REG_IQK_AGC_PTS_11N, 0xf9000000);
	rtw_write32(rtwdev, REG_IQK_AGC_PTS_11N, 0xf8000000);

	mdelay(IQK_DELAY_TIME_8723B); /* NOTE: rtl8xxxu uses mdelay(1) */

	/* restore ant path */
	rtw_write32(rtwdev, REG_BB_SEL_BTG, path_sel);

	/* GNT_BT = 1 */
	rtw_write32(rtwdev, REG_BT_CONTROL_8723B, 0x00001800); /* NOTE: rtl8xxxu does this only ifdef RTL8723BU_BT */

	/* leave IQK mode */
	rtw_write32_mask(rtwdev, REG_FPGA0_IQK_11N, MASKH3BYTES, 0x000000);

	/* Check failed */
	status = rtw8723b_iqk_check_tx_failed(rtwdev);

	return status;
}

/* vendor: hal/phydm/halrf/rtl8723b/halrf_8723b_ce.c
 * function: phy_path_a_rx_iqk_8723b
 */
static u8 rtw8723b_iqk_rx_path_a(struct rtw_dev *rtwdev)
{
	printk("%s begin", __func__);

	/* NOTE: this func is ready! */

	u32 reg_e94, reg_e9c, val32, path_sel;
	u8 status;

	rtw_dbg(rtwdev, RTW_DBG_RFK, "[IQK] path A RX IQK step1!\n");

	/* Save RF path */
	path_sel = rtw_read32(rtwdev, REG_BB_SEL_BTG);

	/* leave IQK mode */
	rtw_write32_mask(rtwdev, REG_FPGA0_IQK_11N, MASKH3BYTES, 0x000000);

	rtw_write_rf(rtwdev, RF_PATH_A, RF_LUTWE, 0x80000, 0x1);
	rtw_write_rf(rtwdev, RF_PATH_A, RF_RCK_OS, RFREG_MASK, 0x30000);
	rtw_write_rf(rtwdev, RF_PATH_A, RF_TXPA_G1, RFREG_MASK, 0x0001f);
	rtw_write_rf(rtwdev, RF_PATH_A, RF_TXPA_G2, RFREG_MASK, 0xf7fb7);

	/* IQK setting */
	rtw_write32(rtwdev, REG_TXIQK_11N, 0x01007c00);
	rtw_write32(rtwdev, REG_RXIQK_11N, 0x01004800);

	/* path-A IQK setting */
	rtw_write32(rtwdev, REG_TXIQK_TONE_A_11N, 0x18008c1c);
	rtw_write32(rtwdev, REG_RXIQK_TONE_A_11N, 0x38008c1c);
	rtw_write32(rtwdev, REG_TX_IQK_TONE_B, 0x38008c1c);
	rtw_write32(rtwdev, REG_RX_IQK_TONE_B, 0x38008c1c);

	rtw_write32(rtwdev, REG_TXIQK_PI_A_11N, 0x82160ff0);
	rtw_write32(rtwdev, REG_RXIQK_PI_A_11N, 0x28110000);
	rtw_write32(rtwdev, REG_TXIQK_PI_B, 0x82110000);
	rtw_write32(rtwdev, REG_RXIQK_PI_B, 0x28110000);

	/* LO calibration setting */
	rtw_write32(rtwdev, REG_IQK_AGC_RSP_11N, 0x0046a911);

	/* enter IQK mode */
	rtw_write32_mask(rtwdev, REG_FPGA0_IQK_11N, MASKH3BYTES, 0x808000);

	/* ant switch */
	if (rtw_hci_type(rtwdev) == RTW_HCI_TYPE_USB)
		rtw_write32(rtwdev, REG_BB_SEL_BTG, 0x280);
	else
		rtw_write32(rtwdev, REG_BB_SEL_BTG, 0x0);

	/* GNT_BT = 0 (disable BT) */
	rtw_write32(rtwdev, REG_BT_CONTROL_8723B, 0x00000800);

	/* One shot, path A LOK & IQK */
	rtw_write32(rtwdev, REG_IQK_AGC_PTS_11N, 0xf9000000);
	rtw_write32(rtwdev, REG_IQK_AGC_PTS_11N, 0xf8000000);

	mdelay(IQK_DELAY_TIME_8723B); // NOTE: rtl8xxxu uses mdelay(1)

	/* restore ant path */
	rtw_write32(rtwdev, REG_BB_SEL_BTG, path_sel);

	/* GNT_BT = 1 */
	rtw_write32(rtwdev, REG_BT_CONTROL_8723B, 0x00001800); /* NOTE: rtl8xxxu does this only ifdef RTL8723BU_BT */

	/* leave IQK mode */
	rtw_write32_mask(rtwdev, REG_FPGA0_IQK_11N, MASKH3BYTES, 0x000000);

	/* Check failed */
	status = rtw8723b_iqk_check_tx_failed(rtwdev);

	/* if Tx not OK, ignore Rx */
	if (!status)
		return status;

	reg_e94 = rtw_read32(rtwdev, REG_IQK_RES_TX);
	reg_e9c = rtw_read32(rtwdev, REG_IQK_RES_TY);
	val32 = 0x80007c00 | (reg_e94 & 0x3ff0000) |
	((reg_e9c & 0x3ff0000) >> 16);
	rtw_write32(rtwdev, REG_TXIQK_11N, val32);

	rtw_dbg(rtwdev, RTW_DBG_RFK, "[IQK] path A RX IQK step2!");

	/* modify RX IQK mode */
	rtw_write32_mask(rtwdev, REG_FPGA0_IQK_11N, MASKH3BYTES, 0x000000);
	rtw_write_rf(rtwdev, RF_PATH_A, RF_LUTWE, 0x80000, 0x1);
	rtw_write_rf(rtwdev, RF_PATH_A, RF_RCK_OS, RFREG_MASK, 0x30000);
	rtw_write_rf(rtwdev, RF_PATH_A, RF_TXPA_G1, RFREG_MASK, 0x0001f);
	rtw_write_rf(rtwdev, RF_PATH_A, RF_TXPA_G2, RFREG_MASK, 0xf7d77);

	/* PA, PAD setting */
	rtw_write_rf(rtwdev, RF_PATH_A, 0xdf, RFREG_MASK, 0xf80);
	rtw_write_rf(rtwdev, RF_PATH_A, 0x55, RFREG_MASK, 0x4021f);

	/* IQK setting */
	rtw_write32(rtwdev, REG_RXIQK_11N, 0x01004800);

	/* path-A IQK setting */
	rtw_write32(rtwdev, REG_TXIQK_TONE_A_11N, 0x38008c1c);
	rtw_write32(rtwdev, REG_RXIQK_TONE_A_11N, 0x18008c1c);
	rtw_write32(rtwdev, REG_TX_IQK_TONE_B, 0x38008c1c);
	rtw_write32(rtwdev, REG_RX_IQK_TONE_B, 0x38008c1c);

	rtw_write32(rtwdev, REG_TXIQK_PI_A_11N, 0x82110000);
	rtw_write32(rtwdev, REG_RXIQK_PI_A_11N, 0x2816001f);
	rtw_write32(rtwdev, REG_TXIQK_PI_B, 0x82110000);
	rtw_write32(rtwdev, REG_RXIQK_PI_B, 0x28110000);

	/* LO calibration setting */
	rtw_write32(rtwdev, REG_IQK_AGC_RSP_11N, 0x0046a8d1);

	/* enter IQK mode */
	rtw_write32_mask(rtwdev, REG_FPGA0_IQK_11N, MASKH3BYTES, 0x808000);

	/* ant switch */
	if (rtw_hci_type(rtwdev) == RTW_HCI_TYPE_USB)
		rtw_write32(rtwdev, REG_BB_SEL_BTG, 0x280);
	else
		rtw_write32(rtwdev, REG_BB_SEL_BTG, 0x0);

	/* GNT_BT = 0 */
	rtw_write32(rtwdev, REG_BT_CONTROL_8723B, 0x00000800);

	/* One shot, path A LOK & IQK */
	rtw_write32(rtwdev, REG_IQK_AGC_PTS_11N, 0xf9000000);
	rtw_write32(rtwdev, REG_IQK_AGC_PTS_11N, 0xf8000000);

	mdelay(IQK_DELAY_TIME_8723B); // NOTE: rtl8xxxu uses mdelay(1)

	/* restore ant path */
	rtw_write32(rtwdev, REG_BB_SEL_BTG, path_sel);

	/* GNT_BT = 1 */
	rtw_write32(rtwdev, REG_BT_CONTROL_8723B, 0x00001800); /* NOTE: rtl8xxxu does this only ifdef RTL8723BU_BT */

	/* leave IQK mode */
	rtw_write32_mask(rtwdev, REG_FPGA0_IQK_11N, MASKH3BYTES, 0x000000);

	/* Check failed */

	/* NOTE: in vendor driver this is called after reading reg_eac
	 * and reg_ea4, we read them in rtw8723b_iqk_check_rx_failed
	 */
	rtw_write_rf(rtwdev, RF_PATH_A, 0xdf, RFREG_MASK, 0x780);

	status |= rtw8723b_iqk_check_rx_failed(rtwdev);

	return status;
}

/* vendor: hal/phydm/halrf/rtl8723b/halrf_8723b_ce.c
 * function: _phy_path_a_fill_iqk_matrix8723b
 */
static
void rtw8723b_iqk_fill_a_matrix(struct rtw_dev *rtwdev, const s32 result[])
{
	/* NOTE: this func is ready! */

	printk("%s begin", __func__);

	s32 tx1_a, tx1_a_ext;
	s32 tx1_c, tx1_c_ext;
	s32 oldval_1;
	s32 x, y;

	if (result[IQK_S1_TX_X] == 0)
		return;

	oldval_1 = rtw_read32_mask(rtwdev, REG_OFDM_0_XA_TX_IQ_IMBALANCE,
				   BIT_MASK_TXIQ_ELM_D);

	x = iqkxy_to_s32(result[IQK_S1_TX_X]);
	tx1_a = iqk_mult(x, oldval_1, &tx1_a_ext);
	rtw_write32_mask(rtwdev, REG_OFDM_0_XA_TX_IQ_IMBALANCE,
			 BIT_MASK_TXIQ_ELM_A, tx1_a);
	rtw_write32_mask(rtwdev, REG_OFDM_0_ECCA_THRESHOLD,
			 BIT_MASK_OFDM0_EXT_A, tx1_a_ext);

	y = iqkxy_to_s32(result[IQK_S1_TX_Y]);
	tx1_c = iqk_mult(y, oldval_1, &tx1_c_ext);
	rtw_write32_mask(rtwdev, REG_TXIQK_MATRIXA_LSB2_11N, MASKH4BITS,
			 BIT_SET_TXIQ_ELM_C1(tx1_c));
	rtw_write32_mask(rtwdev, REG_OFDM_0_XA_TX_IQ_IMBALANCE,
			 BIT_MASK_TXIQ_ELM_C, BIT_SET_TXIQ_ELM_C2(tx1_c));
	rtw_write32_mask(rtwdev, REG_OFDM_0_ECCA_THRESHOLD,
			 BIT_MASK_OFDM0_EXT_C, tx1_c_ext);

	rtw_dbg(rtwdev, RTW_DBG_RFK,
		"[IQK] X = 0x%x, TX1_A = 0x%x, oldval_1 0x%x\n",
		x, tx1_a, oldval_1);
	rtw_dbg(rtwdev, RTW_DBG_RFK,
		"[IQK] Y = 0x%x, TX1_C = 0x%x\n", y, tx1_c);

	if (result[IQK_S1_RX_X] == 0)
		return;

	rtw_write32_mask(rtwdev, REG_A_RXIQI, BIT_MASK_RXIQ_S1_X,
			 result[IQK_S1_RX_X]);
	rtw_write32_mask(rtwdev, REG_A_RXIQI, BIT_MASK_RXIQ_S1_Y1,
			 BIT_SET_RXIQ_S1_Y1(result[IQK_S1_RX_Y]));
	rtw_write32_mask(rtwdev, REG_RXIQK_MATRIX_LSB_11N, BIT_MASK_RXIQ_S1_Y2,
			 BIT_SET_RXIQ_S1_Y2(result[IQK_S1_RX_Y]));
}

/* vendor: hal/phydm/halrf/rtl8723b/halrf_8723b_ce.c
 * function: _phy_path_b_fill_iqk_matrix8723b
 */
static
void rtw8723b_iqk_fill_b_matrix(struct rtw_dev *rtwdev, const s32 result[])
{
	/* NOTE: this func is ready! */

	printk("%s begin", __func__);

	s32 tx0_a, tx0_a_ext;
	s32 tx0_c, tx0_c_ext;
	s32 oldval_0;
	s32 x, y;

	if (result[IQK_S0_TX_X] == 0)
		return;

	oldval_0 = rtw_read32_mask(rtwdev, REG_OFDM_0_XB_TX_IQ_IMBALANCE,
				   BIT_MASK_TXIQ_ELM_D);

	x = iqkxy_to_s32(result[IQK_S0_TX_X]);
	tx0_a = iqk_mult(x, oldval_0, &tx0_a_ext);

	rtw_write32_mask(rtwdev, REG_OFDM_0_XB_TX_IQ_IMBALANCE,
			 BIT_MASK_TXIQ_ELM_A, tx0_a);
	rtw_write32_mask(rtwdev, REG_OFDM_0_ECCA_THRESHOLD, BIT(27),
			 tx0_a_ext);

	y = iqkxy_to_s32(result[IQK_S0_TX_Y]);
	tx0_c = iqk_mult(y, oldval_0, &tx0_c_ext);

	rtw_write32_mask(rtwdev, REG_TXIQK_MATRIXB_LSB2_11N, MASKH4BITS,
			 BIT_SET_TXIQ_ELM_C1(tx0_c));
	rtw_write32_mask(rtwdev, REG_OFDM_0_XB_TX_IQ_IMBALANCE,
			 BIT_MASK_TXIQ_ELM_C, BIT_SET_TXIQ_ELM_C2(tx0_c));
	rtw_write32_mask(rtwdev, REG_OFDM_0_ECCA_THRESHOLD, BIT(25),
			 tx0_c_ext);

	if (result[IQK_S0_RX_X] == 0)
		return;

	rtw_write32_mask(rtwdev, REG_B_RXIQI, BIT_MASK_RXIQ_X_S0,
			 result[IQK_S0_RX_X]);
	rtw_write32_mask(rtwdev, REG_B_RXIQI, BIT_MASK_RXIQ_S1_Y1,
			 BIT_SET_RXIQ_S1_Y1(result[IQK_S0_RX_Y]));
}

/* vendor hal/phydm/halrf/rtl8723b/halrf_8723b_ce.c
 * function: _phy_mac_setting_calibration8723b
 */
static
void rtw8723b_iqk_config_mac(struct rtw_dev *rtwdev,
			     const struct rtw8723x_iqk_backup_regs *backup)
{
	printk("%s begin", __func__);

	/* NOTE: this func is ready! */

	int i;

	rtw_write8(rtwdev, rtw8723x_common.iqk_mac8_regs[0], 0x3f);

	for (i = 1; i < RTW8723X_IQK_MAC8_REG_NUM; i++)
		rtw_write8(rtwdev, rtw8723x_common.iqk_mac8_regs[i],
			   backup->mac8[i] & (~BIT(3)));

	/* vendor driver writes 1 byte, is that intentional? */
	rtw_write8(rtwdev, rtw8723x_common.iqk_mac32_regs[0],
		   backup->mac32[0] & (~BIT(5)));
}

/* vendor file hal/phydm/halrf/rtl8723b/halrf_8723b_ce.c
 * function: _phy_iq_calibrate_8723b / phy_iq_calibrate_8723b
 */
static
void rtw8723b_iqk_one_round(struct rtw_dev *rtwdev, s32 result[][IQK_NR], u8 t,
			    const struct rtw8723x_iqk_backup_regs *backup)
{
	/* NOTE: this func is ready! */

	printk("%s begin", __func__);

	u32 i;
	u8 a_ok;
	/* u8 b_ok; */

	rtw_dbg(rtwdev, RTW_DBG_RFK,
		"[IQK] IQ Calibration for 1T1R_S0/S1 for %d times\n", t);

	rtw8723x_iqk_path_adda_on(rtwdev, ADDA_ON_VAL_8723B);
	rtw8723b_iqk_config_mac(rtwdev, backup);

	rtw_write32_mask(rtwdev, REG_CCK_ANT_SEL_11N, 0x0f000000, 0xf);
	rtw_write32(rtwdev, REG_BB_RX_PATH_11N, 0x03a05600);
	rtw_write32(rtwdev, REG_TRMUX_11N, 0x000800e4);
	rtw_write32(rtwdev, REG_BB_PWR_SAV1_11N, 0x22204000);

	/* RX IQ calibration setting for 8723B D cut large current issue
	 * when leaving IPS
	 */
	rtw_write32_mask(rtwdev, REG_FPGA0_IQK_11N, MASKH3BYTES, 0x000000);
	rtw_write_rf(rtwdev, RF_PATH_A, RF_LUTWE, 0x80000, 0x1);
	rtw_write_rf(rtwdev, RF_PATH_A, RF_RCK_OS, RFREG_MASK, 0x30000);
	rtw_write_rf(rtwdev, RF_PATH_A, RF_TXPA_G1, RFREG_MASK, 0x0001f);
	rtw_write_rf(rtwdev, RF_PATH_A, RF_TXPA_G2, RFREG_MASK, 0xf7fb7);
	rtw_write_rf(rtwdev, RF_PATH_A, 0xed, 0x20, 0x1);
	rtw_write_rf(rtwdev, RF_PATH_A, 0x43, RFREG_MASK, 0x60fbd);

	for (i = 0; i < PATH_IQK_RETRY; i++) {
		a_ok = rtw8723b_iqk_tx_path_a(rtwdev);
		if (a_ok == IQK_TX_OK) {
			rtw_dbg(rtwdev, RTW_DBG_RFK,
				"[IQK] path A TX IQK success!\n");

			rtw_write32_mask(rtwdev, REG_FPGA0_IQK_11N,
					 MASKH3BYTES, 0x000000);

			result[t][IQK_S1_TX_X] =
				rtw_read32_mask(rtwdev, REG_IQK_RES_TX,
						BIT_MASK_RES_TX);
			result[t][IQK_S1_TX_Y] =
				rtw_read32_mask(rtwdev, REG_IQK_RES_TY,
						BIT_MASK_RES_TY);
			break;
		}

		rtw_dbg(rtwdev, RTW_DBG_RFK, "[IQK] path A TX IQK fail!\n");
		result[t][IQK_S1_TX_X] = 0x100;
		result[t][IQK_S1_TX_Y] = 0x0;
	}

	for (i = 0; i < PATH_IQK_RETRY; i++) {
		a_ok = rtw8723b_iqk_rx_path_a(rtwdev);
		if (a_ok == (IQK_TX_OK | IQK_RX_OK)) {
			rtw_dbg(rtwdev, RTW_DBG_RFK,
				"[IQK] path A RX IQK success!\n");
			result[t][IQK_S1_RX_X] =
				rtw_read32_mask(rtwdev, REG_IQK_RES_RX,
						BIT_MASK_RES_RX);
			result[t][IQK_S1_RX_Y] =
				rtw_read32_mask(rtwdev, REG_IQK_RES_RY,
						BIT_MASK_RES_RY);
			break;
		}

		rtw_dbg(rtwdev, RTW_DBG_RFK, "[IQK] path A RX IQK fail!\n");
		result[t][IQK_S1_RX_X] = 0x100;
		result[t][IQK_S1_RX_Y] = 0x0;
	}

	if (a_ok == 0x0)
		rtw_dbg(rtwdev, RTW_DBG_RFK, "[IQK] path A IQK fail!\n");

	/* NOTE: the vendor driver does path B only for 2T, but rtl8723b is 1T1R */

	rtw_write32_mask(rtwdev, REG_FPGA0_IQK_11N, MASKH3BYTES, 0x000000);
}

/* vendor hal/phydm/halrf/rtl8723b/halrf_8723b_ce.c
 * function: phy_iq_calibrate_8723b / _phy_iq_calibrate_8723b
 */
static void rtw8723b_phy_calibration(struct rtw_dev *rtwdev)
{
	printk("%s begin", __func__);

	struct rtw_dm_info *dm_info = &rtwdev->dm_info;
	struct rtw8723x_iqk_backup_regs backup;
	s32 result[IQK_ROUND_SIZE][IQK_NR];
	u8 final_candidate = IQK_ROUND_INVALID;
	u32 bt_control;
	bool good;
	u8 i, j;

	rtw_dbg(rtwdev, RTW_DBG_RFK, "[IQK] Start!\n");

	memset(result, 0, sizeof(result));

	// TODO
	rtw8723x_iqk_backup_path_ctrl(rtwdev, &backup); // needed? reg 0x67 valid?
	//rtw8723x_iqk_backup_lte_path_gnt(rtwdev, &backup); // not done in vendor driver for rtl8723bs
	rtw8723x_iqk_backup_regs(rtwdev, &backup); // includes 'Safe RF path'

	/* save default GNT_BT */
	bt_control = rtw_read32(rtwdev, REG_BT_CONTROL_8723B);

	for (i = IQK_ROUND_0; i <= IQK_ROUND_2; i++) {
		// TODO
		rtw8723x_iqk_config_path_ctrl(rtwdev); // no trace of this in vendor driver
		//rtw8723x_iqk_config_lte_path_gnt(rtwdev, IQK_LTE_WRITE_VAL_8703B);

		rtw8723b_iqk_one_round(rtwdev, result, i, &backup);

		rtw_dbg(rtwdev, RTW_DBG_RFK,
			"[IQK] back to BB mode, load original value!\n");

		if (i > IQK_ROUND_0) {
			rtw8723x_iqk_restore_regs(rtwdev, &backup);

			/* Restore RX initial gain */
			rtw_write32_mask(rtwdev, REG_OFDM0_XAAGC1, MASKBYTE0, 0x50);
			rtw_write32_mask(rtwdev, REG_OFDM0_XAAGC1, MASKBYTE0, backup.igia);

			/* path B; only for 2T */
			//rtw_write32_mask(rtwdev, REG_OFDM0_XBAGC1, MASKBYTE0, 0x50);
			//rtw_write32_mask(rtwdev, REG_OFDM0_XBAGC1, MASKBYTE0, backup.igib);

			/* load 0xe30 IQC default value */
			rtw_write32(rtwdev, REG_TXIQK_TONE_A_11N, 0x01008c00);
			rtw_write32(rtwdev, REG_RXIQK_TONE_A_11N, 0x01008c00);
		}

		// TODO
		//rtw8723x_iqk_restore_lte_path_gnt(rtwdev, &backup);
		rtw8723x_iqk_restore_path_ctrl(rtwdev, &backup);

		for (j = IQK_ROUND_0; j < i; j++) {
			good = rtw8723x_iqk_similarity_cmp(rtwdev, result, j, i);
			if (good) {
				final_candidate = j;
				rtw_dbg(rtwdev, RTW_DBG_RFK,
					"[IQK] cmp %d:%d final_candidate is %x\n",
					j, i, final_candidate);
				goto iqk_done;
			}
		}
	}

	if (final_candidate == IQK_ROUND_INVALID) {
		s32 reg_tmp = 0;

		for (i = 0; i < IQK_NR; i++)
			reg_tmp += result[IQK_ROUND_HYBRID][i];

		if (reg_tmp != 0) {
			final_candidate = IQK_ROUND_HYBRID;
		} else {
			WARN(1, "IQK failed\n");
			goto out;
		}
	}

iqk_done:
	rtw8723b_iqk_fill_a_matrix(rtwdev, result[final_candidate]);
	rtw8723b_iqk_fill_b_matrix(rtwdev, result[final_candidate]);

	dm_info->iqk.result.s1_x = result[final_candidate][IQK_S1_TX_X];
	dm_info->iqk.result.s1_y = result[final_candidate][IQK_S1_TX_Y];
	dm_info->iqk.result.s0_x = result[final_candidate][IQK_S0_TX_X];
	dm_info->iqk.result.s0_y = result[final_candidate][IQK_S0_TX_Y];
	dm_info->iqk.done = true;

out:
	/* restore RF path */
	rtw_write32(rtwdev, REG_BB_SEL_BTG, backup.bb_sel_btg);

	// TODO: vendo code: do we need it? */
	// _phy_save_adda_registers8723b(p_dm, IQK_BB_REG_92C, p_dm->rf_calibrate_info.IQK_BB_backup_recover, IQK_BB_REG_NUM);

	/* restore GNT_BT */
	rtw_write32(rtwdev, REG_BT_CONTROL_8723B, bt_control);

	/* Resotre RX mode table parameter */
	rtw_write_rf(rtwdev, RF_PATH_A, RF_LUTWE, 0x80000, 0x1);
	rtw_write_rf(rtwdev, RF_PATH_A, RF_RCK_OS, RFREG_MASK, 0x18000);
	rtw_write_rf(rtwdev, RF_PATH_A, RF_TXPA_G1, RFREG_MASK, 0x0001f);
	rtw_write_rf(rtwdev, RF_PATH_A, RF_TXPA_G2, RFREG_MASK, 0xe6177);
	rtw_write_rf(rtwdev, RF_PATH_A, 0xed, 0x20, 0x1);
	rtw_write_rf(rtwdev, RF_PATH_A, 0x43, RFREG_MASK, 0x300bd);


	// TODO: vendor code:  do we need it?
	// 	if(*(p_dm->p_mp_mode)) {
	// 		if ((path_sel_bb == 0x0) || (path_sel_bb == 0x200))  /* S1 */
	// 			odm_set_iqc_by_rfpath(p_dm, 0);
	// 		else
	// 			odm_set_iqc_by_rfpath(p_dm, 1);
	// 	} else {
	// 		if (*p_dm->p_rf_default_path == 0x0)  /* S1 */
	// 			odm_set_iqc_by_rfpath(p_dm, 0);
	// 		else
	// 			odm_set_iqc_by_rfpath(p_dm, 1);
	// 	}


	rtw_dbg(rtwdev, RTW_DBG_RFK, "[IQK] final_candidate is %x\n",
		final_candidate);

	for (i = IQK_ROUND_0; i < IQK_ROUND_SIZE; i++)
		rtw_dbg(rtwdev, RTW_DBG_RFK,
			"[IQK] Result %u: rege94_s1=%x rege9c_s1=%x regea4_s1=%x regeac_s1=%x rege94_s0=%x rege9c_s0=%x regea4_s0=%x regeac_s0=%x %s\n",
			i,
			result[i][0], result[i][1], result[i][2], result[i][3],
			result[i][4], result[i][5], result[i][6], result[i][7],
			final_candidate == i ? "(final candidate)" : "");

	rtw_dbg(rtwdev, RTW_DBG_RFK,
		"[IQK]0xc80 = 0x%x 0xc94 = 0x%x 0xc14 = 0x%x 0xca0 = 0x%x\n",
	rtw_read32(rtwdev, REG_OFDM_0_XA_TX_IQ_IMBALANCE),
		rtw_read32(rtwdev, REG_TXIQK_MATRIXA_LSB2_11N),
		rtw_read32(rtwdev, REG_A_RXIQI),
		rtw_read32(rtwdev, REG_RXIQK_MATRIX_LSB_11N));
	rtw_dbg(rtwdev, RTW_DBG_RFK,
		"[IQK]0xcd0 = 0x%x 0xcd4 = 0x%x 0xcd8 = 0x%x\n",
	rtw_read32(rtwdev, REG_TXIQ_AB_S0),
		rtw_read32(rtwdev, REG_TXIQ_CD_S0),
		rtw_read32(rtwdev, REG_RXIQ_AB_S0));

	rtw_dbg(rtwdev, RTW_DBG_RFK, "[IQK] finished\n");
}

static void rtw8723b_pwrtrack_set_ofdm_pwr(struct rtw_dev *rtwdev, s8 swing_idx,
					   s8 txagc_idx)
{
	printk("%s begin", __func__);

	struct rtw_dm_info *dm_info = &rtwdev->dm_info;

	dm_info->txagc_remnant_ofdm[RF_PATH_A] = txagc_idx;

	rtw8723b_set_iqk_matrix(rtwdev, swing_idx, RF_PATH_A);
	// rtw8723b_set_iqk_matrix(rtwdev, swing_idx, RF_PATH_B);
}

/* adapted from vendor file hal/phydm/halrf/rtl8723b/halrf_8723b_ce.c
 * function: set_cck_filter_coefficient
 */
static void rtw8723b_pwrtrack_set_cck_pwr(struct rtw_dev *rtwdev, s8 swing_idx,
					  s8 txagc_idx)
{
	printk("%s begin", __func__);

	struct rtw_dm_info *dm_info = &rtwdev->dm_info;

	dm_info->txagc_remnant_cck = txagc_idx;

	swing_idx = clamp_t(s8, swing_idx, 0, RTW_CCK_SWING_TABLE_SIZE - 1);

	BUILD_BUG_ON(ARRAY_SIZE(rtw8723b_cck_pwr_regs) !=
		     ARRAY_SIZE(rtw8732b_cck_swing_table_ch1_ch13[0]));

	// if (!p_dm->rf_calibrate_info.is_cck_in_ch14) {
	// 	odm_write_1byte(p_dm, 0xa22, cck_swing_table_ch1_ch13_new[cck_swing_index][0]);
	// 	odm_write_1byte(p_dm, 0xa23, cck_swing_table_ch1_ch13_new[cck_swing_index][1]);
	// 	odm_write_1byte(p_dm, 0xa24, cck_swing_table_ch1_ch13_new[cck_swing_index][2]);
	// 	odm_write_1byte(p_dm, 0xa25, cck_swing_table_ch1_ch13_new[cck_swing_index][3]);
	// 	odm_write_1byte(p_dm, 0xa26, cck_swing_table_ch1_ch13_new[cck_swing_index][4]);
	// 	odm_write_1byte(p_dm, 0xa27, cck_swing_table_ch1_ch13_new[cck_swing_index][5]);
	// 	odm_write_1byte(p_dm, 0xa28, cck_swing_table_ch1_ch13_new[cck_swing_index][6]);
	// 	odm_write_1byte(p_dm, 0xa29, cck_swing_table_ch1_ch13_new[cck_swing_index][7]);
	// } else {
	// 	odm_write_1byte(p_dm, 0xa22, cck_swing_table_ch14_new[cck_swing_index][0]);
	// 	odm_write_1byte(p_dm, 0xa23, cck_swing_table_ch14_new[cck_swing_index][1]);
	// 	odm_write_1byte(p_dm, 0xa24, cck_swing_table_ch14_new[cck_swing_index][2]);
	// 	odm_write_1byte(p_dm, 0xa25, cck_swing_table_ch14_new[cck_swing_index][3]);
	// 	odm_write_1byte(p_dm, 0xa26, cck_swing_table_ch14_new[cck_swing_index][4]);
	// 	odm_write_1byte(p_dm, 0xa27, cck_swing_table_ch14_new[cck_swing_index][5]);
	// 	odm_write_1byte(p_dm, 0xa28, cck_swing_table_ch14_new[cck_swing_index][6]);
	// 	odm_write_1byte(p_dm, 0xa29, cck_swing_table_ch14_new[cck_swing_index][7]);
	// }

	/* TODO: different table for ch 14 */
	for (int i = 0; i < ARRAY_SIZE(rtw8723b_cck_pwr_regs); i++)
		rtw_write8(rtwdev, rtw8723b_cck_pwr_regs[i],
			   rtw8732b_cck_swing_table_ch1_ch13[swing_idx][i]);
}

/* vendor driver hal/phydm/halrf/rtl8723b/halrf_8723b_ce.c
 * function: odm_tx_pwr_track_set_pwr_8723b
 */
static void rtw8723b_pwrtrack_set(struct rtw_dev *rtwdev, u8 path)
{
	printk("%s begin", __func__);

	struct rtw_dm_info *dm_info = &rtwdev->dm_info;
	struct rtw_hal *hal = &rtwdev->hal;
	u8 limit_ofdm;
	/* 8703b and 8723d seem to use RTW_CCK_SWING_TABLE_SIZE */
	u8 limit_cck = 28; /* -2dB */
	s8 final_ofdm_swing_index;
	s8 final_cck_swing_index;

	limit_ofdm = rtw8723x_pwrtrack_get_limit_ofdm(rtwdev);

	final_ofdm_swing_index = dm_info->default_ofdm_index +
				 dm_info->delta_power_index[path];
	final_cck_swing_index = dm_info->default_cck_index +
				dm_info->delta_power_index[path];

	if (final_ofdm_swing_index > limit_ofdm)
		rtw8723b_pwrtrack_set_ofdm_pwr(rtwdev, limit_ofdm,
					       final_ofdm_swing_index - limit_ofdm);
	else if (final_ofdm_swing_index < 0)
		rtw8723b_pwrtrack_set_ofdm_pwr(rtwdev, 0,
					       final_ofdm_swing_index);
	else
		rtw8723b_pwrtrack_set_ofdm_pwr(rtwdev, final_ofdm_swing_index, 0);

	if (final_cck_swing_index > limit_cck)
		rtw8723b_pwrtrack_set_cck_pwr(rtwdev, limit_cck,
					      final_cck_swing_index - limit_cck);
	else if (final_cck_swing_index < 0)
		rtw8723b_pwrtrack_set_cck_pwr(rtwdev, 0,
					      final_cck_swing_index);
	else
		rtw8723b_pwrtrack_set_cck_pwr(rtwdev, final_cck_swing_index, 0);

	// NOTE: not 100% sure but seems correct
	rtw_phy_set_tx_power_level(rtwdev, hal->current_channel);
}

/* vendor driver hal/phydm/halrf/halphyrf_ce.c
 * function: odm_txpowertracking_callback_thermal_meter
 */
static void rtw8723b_phy_pwrtrack(struct rtw_dev *rtwdev)
{
	printk("%s begin", __func__);

	struct rtw_dm_info *dm_info = &rtwdev->dm_info;
	struct rtw_swing_table swing_table;
	u8 thermal_value, delta, path;
	bool do_iqk = false;

	rtw_phy_config_swing_table(rtwdev, &swing_table);

	if (rtwdev->efuse.thermal_meter[0] == 0xff)
		return;

	thermal_value = rtw_read_rf(rtwdev, RF_PATH_A, RF_T_METER, 0xfc00);

	// NOTE: clamping not needed, keep the comments for now
	// to avoid checking it again in the future
	// vendor
	// thermal_value_temp = thermal_value + phydm_get_thermal_offset(p_dm);
	// if (thermal_value_temp > 63)
	// 	thermal_value = 63;
	// else if (thermal_value_temp < 0)
	// 	thermal_value = 0;
	// else
	// 	thermal_value = thermal_value_temp;

	/*4 4. Calculate average thermal meter*/
	rtw_phy_pwrtrack_avg(rtwdev, thermal_value, RF_PATH_A);

	do_iqk = rtw_phy_pwrtrack_need_iqk(rtwdev);

	/* TODO: nod 100% sure below here */

	if (do_iqk)
		rtw8723x_lck(rtwdev);
		// rtw8723b_lck(rtwdev);

	if (dm_info->pwr_trk_init_trigger)
		dm_info->pwr_trk_init_trigger = false;
	else if (!rtw_phy_pwrtrack_thermal_changed(rtwdev, thermal_value,
						   RF_PATH_A))
		goto iqk;

	delta = rtw_phy_pwrtrack_get_delta(rtwdev, RF_PATH_A);

	/* NOTE: also done in rtw_phy_pwrtrack_get_delta */
	delta = min_t(u8, delta, RTW_PWR_TRK_TBL_SZ - 1);

	for (path = 0; path < rtwdev->hal.rf_path_num; path++) {
		s8 delta_cur, delta_last;

		delta_last = dm_info->delta_power_index[path];
		delta_cur = rtw_phy_pwrtrack_get_pwridx(rtwdev, &swing_table,
							path, RF_PATH_A, delta);
		if (delta_last == delta_cur)
			continue;

		dm_info->delta_power_index[path] = delta_cur;
		rtw8723b_pwrtrack_set(rtwdev, path);
	}

	/* not done in the 8723bs vendor driver */
	//rtw8723x_pwrtrack_set_xtal(rtwdev, RF_PATH_A, delta);

iqk:
	if (do_iqk)
		rtw8723b_phy_calibration(rtwdev);
}

static void rtw8723b_pwr_track(struct rtw_dev *rtwdev)
{
	printk("%s begin", __func__);

	struct rtw_efuse *efuse = &rtwdev->efuse;
	struct rtw_dm_info *dm_info = &rtwdev->dm_info;

	if (efuse->power_track_type != 0) {
		rtw_warn(rtwdev, "unsupported power track type");
		return;
	}

	if (!dm_info->pwr_trk_triggered) {
		rtw_write_rf(rtwdev, RF_PATH_A, RF_T_METER,
			     GENMASK(17, 16), 0x03);
		dm_info->pwr_trk_triggered = true;
		return;
	}

	rtw8723b_phy_pwrtrack(rtwdev);
	dm_info->pwr_trk_triggered = false;
}

/* vendor file hal/btc/halbtc8723b1ant.c
 * function: halbtc8723b1ant_init_hw_config
 */
static void rtw8723b_coex_cfg_init(struct rtw_dev *rtwdev)
{
	printk("%s begin", __func__);

	/* enable TBTT nterrupt */
	rtw_write8_mask(rtwdev, 0x550, 0x8, 0x1);

	/* 0x790[5:0]= 0x5 */
	rtw_write8(rtwdev, 0x790, 0x5);

	/* enable counter statistics */
	rtw_write8(rtwdev, 0x778, 0x1);
	rtw_write8_mask(rtwdev, 0x40, 0x20, 0x1);
}

static void rtw8723b_coex_set_gnt_fix(struct rtw_dev *rtwdev)
{
	// printk("%s begin", __func__);

	/* TODO: ... */
}

static void rtw8723b_coex_set_gnt_debug(struct rtw_dev *rtwdev)
{
	// printk("%s begin", __func__);

	/* TODO: ... */
}

/* vendor file: hal/btc/halbtc8723b1ant.c
 * function: ex_halbtc8723b1ant_power_on_setting
 * NOTE: do wee need also 2ant?
 */
static void rtw8723b_coex_set_rfe_type(struct rtw_dev *rtwdev)
{
	/* NOTE: this func should be ready! */

	struct rtw_efuse *efuse = &rtwdev->efuse;
	struct rtw_coex *coex = &rtwdev->coex;
	struct rtw_coex_rfe *coex_rfe = &coex->rfe;
	bool aux = efuse->bt_setting & BIT(6); /* 0xc3[6] */
	u32 reg;

	/* MAYBE TODO: are these correct? */
	coex_rfe->rfe_module_type = rtwdev->efuse.rfe_option;
	coex_rfe->ant_switch_polarity = 0;
	coex_rfe->ant_switch_exist = false;
	coex_rfe->ant_switch_with_bt = false;
	coex_rfe->ant_switch_diversity = false;
	coex_rfe->wlg_at_btg = true;

	rtw_write8(rtwdev, 0x67, 0x20);

	/* set GRAN_BT = 1 */
	rtw_write8(rtwdev, 0x765, 0x18);

	/* set WLAN_ACT = 0 */
	rtw_write8(rtwdev, 0x76e, 0x4);

	switch (rtwdev->hci.type) {
	case RTW_HCI_TYPE_USB:
		rtw_write32(rtwdev, REG_BB_SEL_BTG, 0x0);
		rtw_write8(rtwdev, 0xfe08, 0x1); // antenna inverse
		break;
	case RTW_HCI_TYPE_PCIE:
		/* fallthrough */
	case RTW_HCI_TYPE_SDIO:
		reg = rtwdev->hci.type == RTW_HCI_TYPE_PCIE ? 0x384 : 0x60;
		/* efuse 0xc3[6] == 0, S1(Main), RF_PATH_A
		 * efuse 0xc3[6] == 1, S0(Aux), RF_PATH_B
		 */
		if (aux) {
			rtw_write32(rtwdev, REG_BB_SEL_BTG, 0x0);
			rtw_write8(rtwdev, reg, 0x1);
		} else {
			rtw_write32(rtwdev, REG_BB_SEL_BTG, 0x280);
			rtw_write8(rtwdev, reg, 0x0);
		}
		break;
	default:
		break;
	}
}

static void rtw8723b_coex_set_wl_tx_power(struct rtw_dev *rtwdev, u8 wl_pwr)
{
	// printk("%s begin", __func__);

	/* TODO: ... */
}

static void rtw8723b_coex_set_wl_rx_gain(struct rtw_dev *rtwdev, bool low_gain)
{
	// printk("%s begin", __func__);

	/* TODO: ... */
}

static void rtw8723b_cfg_ldo25(struct rtw_dev *rtwdev, bool enable)
{
	// printk("%s begin", __func__);

	/* TODO: ... */
}

static void rtw8723b_fill_txdesc_checksum(struct rtw_dev *rtwdev,
										 struct rtw_tx_pkt_info *pkt_info,
										 struct rtw_tx_desc *txdesc)
{
	/* USB (8723BU) expects the raw XOR checksum (no bitwise inversion).
	 * SDIO/PCIe variants (8723BS/BE) require inverted checksum as in
	 * __rtw8723x_fill_txdesc_checksum. Use bus-aware behavior to avoid
	 * dropping TX packets on USB.
	 */
	if (rtw_hci_type(rtwdev) == RTW_HCI_TYPE_USB) {
		/* compute XOR of first 32 bytes (16 words) without inversion */
		fill_txdesc_checksum_common(txdesc, 32 / 2);
	} else {
		/* call common routine which applies vendor-specific inversion */
		rtw8723x_fill_txdesc_checksum(rtwdev, pkt_info, txdesc);
	}
}

static const struct rtw_chip_ops rtw8723b_ops = {
	.power_on		= rtw_power_on,
	.power_off		= rtw_power_off,

	.mac_init		= rtw8723b_mac_init,
	//.mac_postinit		= NULL,
	.mac_postinit		= rtw8723x_mac_postinit,

	.dump_fw_crash		= NULL,
	/* 8723d sets REG_HCI_OPT_CTRL to BIT_USB_SUS_DIS in
	 * its shutdown fubction, not needed for SDIO devices.
	 */
	.shutdown		= NULL,
	.read_efuse		= rtw8723x_read_efuse,
	.phy_set_param		= rtw8723b_phy_set_param,

	.set_channel		= rtw8723b_set_channel,

	.query_phy_status	= rtw8723b_query_phy_status,
	.read_rf		= rtw_phy_read_rf_sipi,		// ??  looks like vendor phy_RFSerialRead_8723B ?
	.write_rf		= rtw_phy_write_rf_reg_sipi,	// ??
	.set_tx_power_index	= rtw8723x_set_tx_power_index, /* checked against vendor and it matches */
	.rsvd_page_dump		= NULL,
	.set_antenna		= NULL,
	.cfg_ldo25		= rtw8723b_cfg_ldo25,
	.efuse_grant		= rtw8723b_efuse_grant,
	// .efuse_grant		= rtw8723x_efuse_grant,
	.set_ampdu_factor	= NULL,
	.false_alarm_statistics	= rtw8723x_false_alarm_statistics, /* checked against vendor driver and it matches */
	.phy_calibration	= rtw8723b_phy_calibration,
	.dpk_track		= NULL,
	/* from 8703b
	 * 8723d uses REG_CSRATIO to set dm_info.cck_pd_default, which
	 * is used in its cck_pd_set function. According to comments
	 * in the vendor driver code it doesn't exist in this chip
	 * generation, only 0xa0a ("ODM_CCK_PD_THRESH", which is only
	 * *written* to).
	 */
	/* TODO: does above apply to 8723b? */
	.cck_pd_set		= NULL,
	.pwr_track		= rtw8723b_pwr_track,
	.config_bfee		= NULL,
	.set_gid_table		= NULL,
	.cfg_csi_rate		= NULL,
	.adaptivity_init	= NULL,
	.adaptivity		= NULL,
	.cfo_init		= NULL,
	.cfo_track		= NULL,
	.config_tx_path		= NULL,
	.config_txrx_mode	= NULL,
	.led_set		= NULL,
	.fill_txdesc_checksum	= rtw8723b_fill_txdesc_checksum, /* USB needs non-inverted checksum */

	.coex_set_init		= rtw8723b_coex_cfg_init,
	.coex_set_ant_switch	= NULL,
	.coex_set_gnt_fix	= rtw8723b_coex_set_gnt_fix,
	.coex_set_gnt_debug	= rtw8723b_coex_set_gnt_debug,
	.coex_set_rfe_type	= rtw8723b_coex_set_rfe_type,
	.coex_set_wl_tx_power	= rtw8723b_coex_set_wl_tx_power,
	.coex_set_wl_rx_gain	= rtw8723b_coex_set_wl_rx_gain,
};

const struct rtw_chip_info rtw8723b_hw_spec = {
	.ops = &rtw8723b_ops,
	.id = RTW_CHIP_TYPE_8723B,
	.fw_name = "rtw88/rtw8723b_fw.bin",
	.wlan_cpu = RTW_WCPU_8051,
	.tx_pkt_desc_sz = 40,
	.tx_buf_desc_sz = 16,
	.rx_pkt_desc_sz = 24,
	.rx_buf_desc_sz = 8,
	.phy_efuse_size = 512, // ? vendor drv: 8723b and 8723d use EFUSE_MAX_HW_SIZE / 2 */
	.log_efuse_size = 512, // ?
	.ptct_efuse_size = 15,	/* vendor: EFUSE_OOB_PROTECT_BYTES */

	// NOTE: these three are for sure correct
	.txff_size = 32768,	/* vendor: TX_DMA_SIZE_8723B 0x8000 */
	.rxff_size = 16384,	/* vendor: RX_DMA_SIZE_8723B 0x4000 */
	.rsvd_drv_pg_num = 8,   /* NOTE: used together with page_table in mac.c:__priority_queue_cfg_legacy */

	.txgi_factor = 1,
	.is_pwr_by_rate_dec = true,
	.rx_ldpc = false,
	.tx_stbc = false,

	.max_power_index = 0x3f,

	// NOTE: these three are for sure correct
	.csi_buf_pg_num = 0,
	.band = RTW_BAND_2G,
	.page_size = TX_PAGE_SIZE,

	.dig_min = 0x20,
	.usb_tx_agg_desc_num = 1,

	/* vendor function hal_read_mac_hidden_rpt is not called in the
	 * rtl8723bs driver, also when true main.c:rtw_dump_hw_feature fails,
	 * because firmware reports id=0xfd instead of C2H_HW_FEATURE_REPORT,
	 * so seems not supported
	 */
	.hw_feature_report = false,

	.c2h_ra_report_size = 7,	// ?
	.old_datarate_fb_limit = true,	/* likely true; see main-line commit c7706b1 */

	.path_div_supported = false,
	.ht_supported = true,
	.vht_supported = false,
	.lps_deep_mode_supported = 0,

	.sys_func_en = 0xfd,
	.pwr_on_seq = card_enable_flow_8723b,
	.pwr_off_seq = card_disable_flow_8723b,
	.page_table = page_table_8723b, /* this table is correct for sure */

	.rqpn_table = rqpn_table_8723b,
	.prioq_addrs = &rtw8723x_common.prioq_addrs,	// TODO: check this

	/* used only in pci.c, not needed for SDIO devices */
	.intf_table = NULL,

	.dig = rtw8723x_common.dig,
	.dig_cck = rtw8723x_common.dig_cck,

	.rf_sipi_addr = {0x840, 0x844},
	.rf_sipi_read_addr = rtw8723x_common.rf_sipi_addr,

	.fix_rf_phy_num = 2,

	/* there are no traces of lte coex registers in the vendor driver */
	.ltecoex_addr = NULL,

	/* these are correct for sure */
	.mac_tbl = &rtw8723b_mac_tbl,
	.agc_tbl = &rtw8723b_agc_tbl,
	.bb_tbl = &rtw8723b_bb_tbl,
	.rf_tbl = {&rtw8723b_rf_a_tbl},

	.rfe_defs = rtw8723b_rfe_defs,
	.rfe_defs_size = ARRAY_SIZE(rtw8723b_rfe_defs),
	.iqk_threshold = 8, /* this is correct for sure */
	.ampdu_density = IEEE80211_HT_MPDU_DENSITY_16,
	.max_scan_ie_len = IEEE80211_MAX_DATA_LEN,

	/* WOWLAN firmware exists, but not implemented yet */
	.wow_fw_name = "rtw88/rtw8723b_wow_fw.bin",
	.wowlan_stub = NULL,

	.coex_para_ver = 20180201,	/* glcoex_ver_date_8723b_1ant */
	.bt_desired_ver = 0x6f,		/* but for 2 ant it's 0x52 */
	.scbd_support = true,		// ?
	.new_scbd10_def = true,		// ?
	.ble_hid_profile_support = false,	// ?
	.wl_mimo_ps_support = false,		// ?
	.pstdma_type = COEX_PSTDMA_FORCE_LPSOFF,
	.bt_rssi_type = COEX_BTRSSI_RATIO,
	.ant_isolation = 15,
	.rssi_tolerance = 2,
	.wl_rssi_step = wl_rssi_step_8723b,
	.bt_rssi_step = bt_rssi_step_8723b,
	.table_sant_num = ARRAY_SIZE(table_sant_8723b),
	.table_sant = table_sant_8723b,
	.table_nsant_num = ARRAY_SIZE(table_nsant_8723b),
	.table_nsant = table_nsant_8723b,
	.tdma_sant_num = ARRAY_SIZE(tdma_sant_8723b),
	.tdma_sant = tdma_sant_8723b,
	.tdma_nsant_num = ARRAY_SIZE(tdma_nsant_8723b),
	.tdma_nsant = tdma_nsant_8723b,
	.wl_rf_para_num = ARRAY_SIZE(rf_para_tx_8723b),
	.wl_rf_para_tx = rf_para_tx_8723b,
	.wl_rf_para_rx = rf_para_rx_8723b,
	.bt_afh_span_bw20 = 0x20,
	.bt_afh_span_bw40 = 0x30,
	.afh_5g_num =  ARRAY_SIZE(afh_5g_8723b),
	.afh_5g = afh_5g_8723b,
	/* REG_BTG_SEL doesn't seem to have a counterpart in the
	 * vendor driver. Mathematically it's REG_PAD_CTRL1 + 3.
	 *
	 * It is used in the cardemu_to_act power sequence by though
	 * (by address, 0x0067), comment: "0x67[0] = 0 to disable
	 * BT_GPS_SEL pins" That seems to fit.
	 */
	.btg_reg = NULL,

	/* These registers are used to read (and print) from if
	 * CONFIG_RTW88_DEBUGFS is enabled.
	 */
	.coex_info_hw_regs_num = 0,
	.coex_info_hw_regs = NULL,
};
EXPORT_SYMBOL(rtw8723b_hw_spec);

MODULE_FIRMWARE("rtw88/rtw8723b_fw.bin");
MODULE_FIRMWARE("rtw88/rtw8723b_wow_fw.bin");

MODULE_AUTHOR("Luka Gejak <luka.gejak@linux.dev>");
MODULE_AUTHOR("Michael Straube <straube.linux@gmail.com>");
MODULE_DESCRIPTION("Realtek 802.11n wireless 8723b driver");
MODULE_LICENSE("Dual BSD/GPL");
