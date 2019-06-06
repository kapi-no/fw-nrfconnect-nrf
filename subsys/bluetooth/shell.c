/** @file
 * @brief Bluetooth shell module
 *
 * Provide some Bluetooth shell commands that can be useful to applications.
 */

/*
 * Copyright (c) 2017 Intel Corporation
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <misc/printk.h>
#include <misc/byteorder.h>
#include <zephyr.h>

#include <settings/settings.h>

#include <bluetooth/hci.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/rfcomm.h>
#include <bluetooth/sdp.h>

#include <shell/shell.h>

static const struct shell *ctx_shell;

static void connected(struct bt_conn *conn, u8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		shell_error(ctx_shell, "Failed to connect to %s (%u)", addr,
			     err);
		return;
	}

	shell_print(ctx_shell, "Connected: %s", addr);
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	shell_print(ctx_shell, "Disconnected: %s (reason %u)", addr, reason);
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static int cmd_init(const struct shell *shell,
			    size_t argc, char *argv[])
{
	if (argc != 1) {
		shell_print(shell, "Incorrect number of argumentsl");
	}
	ctx_shell = shell;

        bt_conn_cb_register(&conn_callbacks);
	shell_print(shell, "Initialized");

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(bt_cmds,
	SHELL_CMD_ARG(init, NULL, NULL, cmd_init, 1, 0),
	SHELL_SUBCMD_SET_END
);

static int cmd_bt(const struct shell *shell, size_t argc, char **argv)
{
	if (argc == 1) {
		shell_help(shell);
		return SHELL_CMD_HELP_PRINTED;
	}

	shell_error(shell, "%s unknown parameter: %s", argv[0], argv[1]);

	return -EINVAL;
}

SHELL_CMD_REGISTER(nrf_bt, &bt_cmds, "Bluetooth shell commands", cmd_bt);
