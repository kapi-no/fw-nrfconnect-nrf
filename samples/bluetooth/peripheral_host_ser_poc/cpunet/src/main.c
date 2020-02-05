/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */

#include <zephyr.h>

#include "bt_ser.h"
#include "rpmsg.h"

void main(void)
{
	int err = 0;

	printk("Starting BLE Host serialization PoC [NET CORE]\n");

	ipc_register_rx_callback(bt_rx_parse);

	err = ipc_init();
	if (err) {
		printk("RPMSG serialization init error %d\n", err);
		return;
	}
}
