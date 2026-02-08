// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/* Copyright(c) 2024-2026 Luka Gejak <luka.gejak@linux.dev>
 *
 * RTL8723BS SDIO bus binding.
 */

#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/module.h>
#include "main.h"
#include "rtw8723b.h"
#include "sdio.h"

static const struct sdio_device_id rtw8723bs_id_table[] = {
	{
		SDIO_DEVICE(SDIO_VENDOR_ID_REALTEK,
			    SDIO_DEVICE_ID_REALTEK_RTW8723BS),
		.driver_data = (kernel_ulong_t)&rtw8723b_hw_spec,
	},
	{}
};
MODULE_DEVICE_TABLE(sdio, rtw8723bs_id_table);

static struct sdio_driver rtw8723bs_driver = {
	.name = "rtw_8723bs",
	.probe = rtw_sdio_probe,
	.remove = rtw_sdio_remove,
	.id_table = rtw8723bs_id_table,
	.drv = {
		.pm = &rtw_sdio_pm_ops,
		.shutdown = rtw_sdio_shutdown,
	},
};
module_sdio_driver(rtw8723bs_driver);

MODULE_AUTHOR("Luka Gejak <luka.gejak@linux.dev>");
MODULE_DESCRIPTION("Realtek 802.11n wireless 8723bs SDIO driver");
MODULE_LICENSE("Dual BSD/GPL");
