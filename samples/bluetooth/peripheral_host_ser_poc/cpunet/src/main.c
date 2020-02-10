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
#include <bluetooth/conn.h>

static void connected(struct bt_conn *conn, u8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
	}

	u8_t buf[CONFIG_NETWORK_TO_APP_MAX_PACKET_SIZE];
	u8_t *it = buf;

	memcpy(it, &conn, sizeof(conn));
	it += sizeof(conn);

	memcpy(it, &err, sizeof(err));
	it += sizeof(err);

	bt_evt_send(BT_EVENT_CONNECTED, buf, (it - buf));
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);

	u8_t buf[CONFIG_NETWORK_TO_APP_MAX_PACKET_SIZE];
	u8_t *it = buf;

	memcpy(it, &conn, sizeof(conn));
	it += sizeof(conn);

	memcpy(it, &reason, sizeof(reason));
	it += sizeof(reason);

	bt_evt_send(BT_EVENT_DISCONNECTED, buf, (it - buf));
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

void main(void)
{
	int err = 0;

	printk("Starting BLE Host serialization PoC [NET CORE]\n");

	bt_conn_cb_register(&conn_callbacks);

	ipc_register_rx_callback(bt_rx_parse);

	err = ipc_init();
	if (err) {
		printk("RPMSG serialization init error %d\n", err);
		return;
	}
}
