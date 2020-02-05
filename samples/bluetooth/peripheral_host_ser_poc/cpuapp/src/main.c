/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */

#include <drivers/uart.h>
#include <zephyr/types.h>
#include <zephyr.h>

#include <device.h>
#include <soc.h>

#include <dk_buttons_and_leds.h>

#include <stdio.h>
#include <string.h>
#include <init.h>

#include "bt_ser.h"
#include "rpmsg.h"

#include <bluetooth/bluetooth.h>

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x0d, 0x18, 0x0f, 0x18, 0x0a, 0x18),
};

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true) {
		/* Spin for ever */
		k_sleep(1000);
	}
}

static void configure_gpio(void)
{
	int err;

	err = dk_leds_init();
	if (err) {
		printk("Cannot init LEDs (err: %d)\n", err);
	}
}

void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth initialization failed: %d\n", err);
	}

	printk("Bluetooth initialized: %d\n", err);

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

void main(void)
{
	int blink_status = 0;
	int err = 0;

	ipc_register_rx_callback(bt_rx_parse);
	err = ipc_init();
	if (err) {
		printk("Rpmsg init error %d\n", err);
	}

	configure_gpio();

	err = bt_enable(bt_ready);
	if (err < 0) {
		printk("Bt enable failed: %d\n", err);
		error();
	}

	printk("Starting BLE Host serialization PoC [APP CORE]\n");

	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(RUN_LED_BLINK_INTERVAL);
	}
}