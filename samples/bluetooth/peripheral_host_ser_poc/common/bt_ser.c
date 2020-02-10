/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cbor.h>
#include <errno.h>
#include <kernel.h>
#include <zephyr/types.h>

#include <bluetooth/addr.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>

#include "bt_ser.h"
#include "rpmsg.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(bt_ser);

#define BT_EVT_SEND_ARRAY_SIZE 0x03
#define BT_RSP_RESULT_ARRAY_SIZE 0x02
#define BT_CMD_BT_ENABLE_ARRAY_SIZE 0x03
#define BT_CMD_BT_LE_ADV_START_ARRAY_SIZE 0x07

#define BT_CMD_BT_ENABLE 0x01
#define BT_CMD_BT_LE_ADV_START 0x02

#define BT_TYPE_CMD 0x01
#define BT_TYPE_EVENT 0x02
#define BT_TYPE_RSP 0x03

#define APP_TO_NETWORK_MAX_PKT_SIZE CONFIG_APP_TO_NETWORK_MAX_PACKET_SIZE
#define NETWORK_TO_APP_MAX_PKT_SIZE CONFIG_NETWORK_TO_APP_MAX_PACKET_SIZE

#define APP_WAIT_FOR_RSP_TIME CONFIG_APP_WAIT_FOR_RESPONSE_TIME

K_SEM_DEFINE(rsp_sem, 0, 1);

typedef int (*cmd_rsp_handler_t)(CborValue *it);
typedef int (*cmd_handler_t)(CborValue *it, CborEncoder *encoder);

struct cmd_list {
	u8_t cmd;
	cmd_handler_t func;
};

static cmd_rsp_handler_t rsp_handler;

static int response_code_encode(CborEncoder *encoder, int rsp)
{
	int err;
	CborEncoder array;

	err = cbor_encoder_create_array(encoder, &array,
					BT_RSP_RESULT_ARRAY_SIZE);

	err = cbor_encode_simple_value(&array, BT_TYPE_RSP);
	if (err) {
		return -EFAULT;
	}

	if (cbor_encode_int(&array, rsp)) {
		return -EFAULT;
	}

	err = cbor_encoder_close_container(encoder, &array);
	if (err) {
		return -EFAULT;
	}

	return 0;
}

#if defined(NRF5340_XXAA_NETWORK)
int bt_evt_send(u8_t evt, const u8_t *evt_data, size_t length)
{
	int err;
	CborEncoder encoder;
	CborEncoder array;
	u8_t buf[NETWORK_TO_APP_MAX_PKT_SIZE];
	size_t buf_len = sizeof(buf);

	cbor_encoder_init(&encoder, buf, buf_len, 0);
	err = cbor_encoder_create_array(&encoder, &array,
					BT_EVT_SEND_ARRAY_SIZE);
	if (err) {
		goto error;
	}

	err = cbor_encode_simple_value(&array, BT_TYPE_EVENT);
	if (err) {
		goto error;
	}

	err = cbor_encode_simple_value(&array, evt);
	if (err) {
		goto error;
	}

	err = cbor_encode_byte_string(&array, evt_data, length);
	if (err) {
		goto error;
	}

	err = cbor_encoder_close_container(&encoder, &array);
	if (err) {
		goto error;
	}

	buf_len = cbor_encoder_get_buffer_size(&encoder, buf);

	LOG_DBG("Sending 0x%02x event", evt);

	return ipc_transmit(buf, buf_len);

error:
	LOG_ERR("Send event 0x%02x error %d", evt, err);
	return err;
}


enum {
	APP_CORE_BT_READY_CB,

	/* Total number of callbacks */
	APP_CORE_CBS_NUM,
};
static u32_t application_core_cbs[APP_CORE_CBS_NUM];

void bt_ready(int err)
{
	u8_t buf[APP_TO_NETWORK_MAX_PKT_SIZE];
	u8_t *it = buf;

	/* Encode callback. */
	memcpy(it, &application_core_cbs[APP_CORE_BT_READY_CB], sizeof(application_core_cbs[0]));
	it += sizeof(application_core_cbs[0]);

	/* Encode callback arguments. */
	memcpy(it, &err, sizeof(err));
	it += sizeof(err);

	bt_evt_send(BT_EVENT_READY, buf, (it - buf));
}

static int bt_data_collect(u8_t *buf, size_t buf_len, struct bt_data *ad_data, size_t ad_len)
{
	for (int i = 0, buf_pos = 0; i < ad_len; i++, ad_data++) {
		if ((buf_pos + 2) > buf_len) {
			return -EINVAL;
		}

		ad_data->type = buf[buf_pos++];
		ad_data->data_len = buf[buf_pos++];

		if ((buf_pos + ad_data->data_len) > buf_len) {
			return -EINVAL;	
		}

		ad_data->data = &buf[buf_pos];
		buf_pos += ad_data->data_len;
	}

	return 0;
}

static int bt_cmd_bt_le_adv_start(CborValue *it, CborEncoder *encoder)
{
	struct bt_le_adv_param adv_param;
	size_t ad_len;
	size_t sd_len;
	u8_t ad_buf[APP_TO_NETWORK_MAX_PKT_SIZE];
	u8_t sd_buf[APP_TO_NETWORK_MAX_PKT_SIZE];
	struct bt_data ad_data[10];
	struct bt_data sd_data[10];
	size_t len;

	len = sizeof(adv_param);
	if (!cbor_value_is_byte_string(it) ||
	    cbor_value_copy_byte_string(it, (char *) &adv_param, &len, it) != CborNoError) {
		return -EINVAL;
	}

	len = sizeof(ad_len);
	if (!cbor_value_is_byte_string(it) ||
	    cbor_value_copy_byte_string(it, (char *) &ad_len, &len, it) != CborNoError) {
		return -EINVAL;
	}

	len = sizeof(sd_len);
	if (!cbor_value_is_byte_string(it) ||
	    cbor_value_copy_byte_string(it, (char *) &sd_len, &len, it) != CborNoError) {
		return -EINVAL;
	}

	if (ad_len != 0) {
		len = sizeof(ad_buf);
		if (!cbor_value_is_byte_string(it) ||
		cbor_value_copy_byte_string(it, (char *) &ad_buf, &len, it) != CborNoError) {
			return -EINVAL;
		}

		if (ad_len > ARRAY_SIZE(ad_data)) {
			return -ENOMEM;
		}

		if (bt_data_collect(ad_buf, len, ad_data, ad_len)) {
			return -ENOMEM;
		}
	}

	if (sd_len != 0) {
		len = sizeof(sd_buf);
		if (!cbor_value_is_byte_string(it) ||
		cbor_value_copy_byte_string(it, (char *) &sd_buf, &len, it) != CborNoError) {
			return -EINVAL;
		}

		if (sd_len > ARRAY_SIZE(ad_data)) {
			return -ENOMEM;
		}

		if (bt_data_collect(sd_buf, len, sd_data, sd_len)) {
			return -ENOMEM;
		}
	}

	return bt_le_adv_start(&adv_param, (ad_len > 0) ? ad_data : NULL, ad_len,
		(sd_len > 0) ? sd_data : NULL, sd_len);
}

static int bt_cmd_bt_enable(CborValue *it, CborEncoder *encoder)
{
	int err;
	bt_ready_cb_t cb;
	u32_t bt_ready_addr;
	size_t len = sizeof(bt_ready_addr);

	if (!cbor_value_is_byte_string(it) ||
	    cbor_value_copy_byte_string(it, (char *) &bt_ready_addr, &len, it) != CborNoError) {
		return -EINVAL;
	}

	application_core_cbs[APP_CORE_BT_READY_CB] = bt_ready_addr;
	cb = (bt_ready_addr > 0) ? bt_ready : NULL;
	err = bt_enable(cb);
	if (err) {
		LOG_ERR("Failed to enable Bluetooth: %d", err);
	}

	return err;
}

static const struct cmd_list ser_cmd_list[] = {
	{BT_CMD_BT_ENABLE, bt_cmd_bt_enable},
	{BT_CMD_BT_LE_ADV_START, bt_cmd_bt_le_adv_start},
};
#else
static const struct cmd_list ser_cmd_list[0];
#endif /* defined(NRF5340_XXAA_NETWORK) */

static cmd_handler_t cmd_handler_get(u8_t cmd)
{
	cmd_handler_t cmd_handler = NULL;

	for (size_t i = 0; i < ARRAY_SIZE(ser_cmd_list); i++) {
		if (cmd == ser_cmd_list[i].cmd) {
			cmd_handler = ser_cmd_list[i].func;
			break;
		}
	}

	return cmd_handler;
}

static int cmd_execute(u8_t cmd, CborValue *it, CborEncoder *encoder)
{
	int err;
	cmd_handler_t cmd_handler;

	cmd_handler = cmd_handler_get(cmd);

	if (cmd_handler) {
		err = cmd_handler(it, encoder);
	} else {
		LOG_ERR("Unsupported command received");
		err = -ENOTSUP;
	}

	return response_code_encode(encoder, err);
}

struct bt_event_data_t {
	void  *fifo_reserved;
	u8_t evt;
	u8_t data[NETWORK_TO_APP_MAX_PKT_SIZE];
	size_t len;
};

static K_FIFO_DEFINE(bt_event_data_q);
static struct bt_conn_cb *callback_list;

static void bt_ready_evt(const u8_t *data, size_t length)
{
	bt_ready_cb_t cb;
	int err;

	if (length != 8) {
		LOG_ERR("bt_ready_evt parsing error: length != 8");
	}

	memcpy(&cb, data, sizeof(cb));
	data += sizeof(cb);

	memcpy(&err, data, sizeof(err));
	data += sizeof(err);

	cb(err);
}

static void bt_connected_evt(const u8_t *data, size_t length)
{
	struct bt_conn *conn;
	u8_t err;

	if (length != 5) {
		LOG_ERR("bt_connected_evt parsing error: length != 5");
	}

	memcpy(&conn, data, sizeof(conn));
	data += sizeof(conn);

	memcpy(&err, data, sizeof(err));
	data += sizeof(err);

	for (struct bt_conn_cb *cb = callback_list; cb; cb = cb->_next) {
		if (cb->connected) {
			cb->connected(conn, err);
		}
	}
}

static void bt_disconnected_evt(const u8_t *data, size_t length)
{
	struct bt_conn *conn;
	u8_t reason;

	if (length != 5) {
		LOG_ERR("bt_disconnected_evt parsing error: length != 5");
	}

	memcpy(&conn, data, sizeof(conn));
	data += sizeof(conn);

	memcpy(&reason, data, sizeof(reason));
	data += sizeof(reason);

	for (struct bt_conn_cb *cb = callback_list; cb; cb = cb->_next) {
		if (cb->disconnected) {
			cb->disconnected(conn, reason);
		}
	}
}

void bt_event_thread(void)
{
	for (;;) {
		/* Wait indefinitely for data to be sent over bluetooth */
		struct bt_event_data_t *buf = k_fifo_get(&bt_event_data_q, K_FOREVER);

		switch (buf->evt) {
		case BT_EVENT_READY:
			bt_ready_evt(buf->data, buf->len);
			break;

		case BT_EVENT_CONNECTED:
			bt_connected_evt(buf->data, buf->len);
			break;

		case BT_EVENT_DISCONNECTED:
			bt_disconnected_evt(buf->data, buf->len);
			break;

		default:
			LOG_ERR("Unkown event: %d", buf->evt);
		}

		k_free(buf);
	}
}

K_THREAD_DEFINE(ble_write_thread_id, 2048, bt_event_thread, NULL, NULL,
		NULL, 7, 0, K_NO_WAIT);

static int event_parse(u8_t evt, CborValue *it)
{
	struct bt_event_data_t *buf = k_malloc(sizeof(*buf));

	buf->evt = evt;
	buf->len = sizeof(buf->data);

	if (!cbor_value_is_byte_string(it) ||
	    cbor_value_copy_byte_string(it, buf->data,
		    &buf->len, it) != CborNoError) {
		return -EINVAL;
	}

	LOG_DBG("Event: 0x%02x", evt);

	/* Switch context to BT Event thread, so it is possible
	 * to use BT API calls in callbacks that are derived
	 * from event data without deadlocks.
	 */
	k_fifo_put(&bt_event_data_q, buf);

	return 0;
}

static int rsp_send(CborEncoder *encoder, const u8_t *buf)
{
	size_t buf_len;

	buf_len = cbor_encoder_get_buffer_size(encoder, buf);

	return ipc_transmit(buf, buf_len);
}

static int bt_response_parse(CborValue *it)
{
	int err;

	err = rsp_handler(it);
	k_sem_give(&rsp_sem);

	return err;
}

static int bt_event_parse(CborValue *it)
{
	u8_t event;

	if (!cbor_value_is_simple_type(it) ||
		cbor_value_get_simple_type(it, &event) != CborNoError) {
		return -EINVAL;
	}

	if (cbor_value_advance_fixed(it) != CborNoError) {
		return -EFAULT;
	}

	return event_parse(event, it);
}

static int bt_cmd_parse(CborValue *it)
{
	u8_t cmd;
	int err;
	CborEncoder encoder;
	u8_t buf[NETWORK_TO_APP_MAX_PKT_SIZE];
	size_t buf_len = sizeof(buf);

	cbor_encoder_init(&encoder, buf, buf_len, 0);

	if (!cbor_value_is_simple_type(it) ||
		cbor_value_get_simple_type(it, &cmd) != CborNoError) {
		return -EINVAL;
	}

	if (cbor_value_advance_fixed(it) != CborNoError) {
		return -EFAULT;
	}

	err = cmd_execute(cmd, it, &encoder);
	if (err) {
		return err;
	}

	LOG_DBG("Received command 0x%02x", cmd);

	return rsp_send(&encoder, buf);
}

int bt_rx_parse(const u8_t *data, size_t len)
{
	int err;
	CborParser parser;
	CborValue value;
	CborValue recursed;
	u8_t packet_type;

	err = cbor_parser_init(data, len, 0, &parser, &value);
	if (err) {
		return err;
	}

	if (!cbor_value_is_array(&value)) {
		return -EINVAL;
	}

	if (cbor_value_enter_container(&value, &recursed) != CborNoError) {
		return -EINVAL;
	}

	/* Get BLE packet type. */
	if (!cbor_value_is_simple_type(&recursed) ||
		cbor_value_get_simple_type(&recursed,
			&packet_type) != CborNoError) {
		return -EINVAL;
	}

	cbor_value_advance_fixed(&recursed);

	switch (packet_type) {
	case BT_TYPE_CMD:
		LOG_DBG("Command received");
		return bt_cmd_parse(&recursed);

	case BT_TYPE_EVENT:
		LOG_DBG("Event received");
		return bt_event_parse(&recursed);

	case BT_TYPE_RSP:
		LOG_DBG("Response received");
		return bt_response_parse(&recursed);

	default:
		LOG_ERR("Unknown packet received");
		return -ENOTSUP;

	}

	/* Be sure that we unpacked all data from the array */
	if (!cbor_value_at_end(&recursed)) {
		LOG_ERR("Received more data than expected");
		return -EFAULT;
	}

	return cbor_value_leave_container(&value, &recursed);
}

#if defined(NRF5340_XXAA_APPLICATION)
static int return_value;

static int bt_send_rsp(CborValue *it)
{
	int error;

	if (!cbor_value_is_integer(it) ||
		cbor_value_get_int(it, &error) != CborNoError) {
		return -EINVAL;
	}

	return_value = error;

	return 0;
}

static int cmd_send(const u8_t *data, size_t length,
		    cmd_rsp_handler_t rsp)
{
	int err = 0;

	rsp_handler = rsp;

	if (ipc_transmit(data, length) < 0) {
		return -EFAULT;
	}

	if (rsp) {
		err = k_sem_take(&rsp_sem, K_MSEC(APP_WAIT_FOR_RSP_TIME));
		if (err) {
			return err;
		}

		LOG_DBG("Got response with return value %d", return_value);

		return return_value;
	}

	return err;
}

static int bt_enable_encode(u32_t bt_ready_addr, u8_t *buf, size_t *buf_len)
{
	CborError err;
	CborEncoder encoder;
	CborEncoder array;

	cbor_encoder_init(&encoder, buf, *buf_len, 0);
	err = cbor_encoder_create_array(&encoder, &array,
					BT_CMD_BT_ENABLE_ARRAY_SIZE);
	if (err) {
		goto error;
	}

	err = cbor_encode_simple_value(&array, BT_TYPE_CMD);
	if (err) {
		goto error;
	}

	err = cbor_encode_simple_value(&array, BT_CMD_BT_ENABLE);
	if (err) {
		goto error;
	}

	err = cbor_encode_byte_string(&array, (const u8_t *) &bt_ready_addr, sizeof(bt_ready_addr));
	if (err) {
		goto error;
	}

	err = cbor_encoder_close_container(&encoder, &array);
	if (err) {
		goto error;
	}

	*buf_len = cbor_encoder_get_buffer_size(&encoder, buf);

	return 0;

error:
	LOG_ERR("Encoding 0x%02X command error %d", BT_CMD_BT_ENABLE, err);
	return err;

}

static int bt_ad_encode(const struct bt_data *ad, size_t ad_len, u8_t *buf, size_t *buf_len)
{
	u8_t *ad_buf_pos = buf;
	for (int i = 0; i < ad_len; i++) {
		if ((ad_buf_pos - buf) + 2 + ad[i].data_len > *buf_len) {
			return -EINVAL;
		}

		*ad_buf_pos++ = ad[i].type;
		*ad_buf_pos++ = ad[i].data_len;

		memcpy(ad_buf_pos, ad[i].data, ad[i].data_len);
		ad_buf_pos += ad[i].data_len;
	}

	*buf_len = (ad_buf_pos - buf);

	return 0;
}

static int bt_le_adv_start_encode(const struct bt_le_adv_param *param,
		    const struct bt_data *ad, size_t ad_len,
		    const struct bt_data *sd, size_t sd_len,
		    u8_t *buf, size_t *buf_len)
{
	CborError err;
	CborEncoder encoder;
	CborEncoder array;
	size_t container_size = BT_CMD_BT_LE_ADV_START_ARRAY_SIZE;

	if (ad_len == 0) {
		container_size--;
	}

	if (sd_len == 0) {
		container_size--;
	}

	cbor_encoder_init(&encoder, buf, *buf_len, 0);
	err = cbor_encoder_create_array(&encoder, &array, container_size);
	if (err) {
		goto error;
	}

	err = cbor_encode_simple_value(&array, BT_TYPE_CMD);
	if (err) {
		goto error;
	}

	err = cbor_encode_simple_value(&array, BT_CMD_BT_LE_ADV_START);
	if (err) {
		goto error;
	}

	err = cbor_encode_byte_string(&array, (const u8_t *) param, sizeof(*param));
	if (err) {
		goto error;
	}

	err = cbor_encode_byte_string(&array, (const u8_t *) &ad_len, sizeof(ad_len));
	if (err) {
		goto error;
	}

	err = cbor_encode_byte_string(&array, (const u8_t *) &sd_len, sizeof(sd_len));
	if (err) {
		goto error;
	}

	if (ad_len != 0) {
		u8_t ad_buf[APP_TO_NETWORK_MAX_PKT_SIZE];
		size_t ad_buf_len = sizeof(ad_buf);

		err = bt_ad_encode(ad, ad_len, ad_buf, &ad_buf_len);
		if (err) {
			goto error;
		}

		err = cbor_encode_byte_string(&array, ad_buf, ad_buf_len);
		if (err) {
			goto error;
		}
	}

	if (sd_len != 0) {
		u8_t sd_buf[APP_TO_NETWORK_MAX_PKT_SIZE];
		size_t sd_buf_len = sizeof(sd_buf);

		err = bt_ad_encode(sd, sd_len, sd_buf, &sd_buf_len);
		if (err) {
			goto error;
		}

		err = cbor_encode_byte_string(&array, sd_buf, sd_buf_len);
		if (err) {
			goto error;
		}
	}

	err = cbor_encoder_close_container(&encoder, &array);
	if (err) {
		goto error;
	}

	*buf_len = cbor_encoder_get_buffer_size(&encoder, buf);

	return 0;

error:
	LOG_ERR("Encoding 0x%02X command error %d", BT_CMD_BT_LE_ADV_START, err);
	return err;
}

int bt_enable(bt_ready_cb_t cb)
{
	int err;
	u8_t buf[APP_TO_NETWORK_MAX_PKT_SIZE];
	size_t buf_len = sizeof(buf);

	err = bt_enable_encode((u32_t) cb, buf, &buf_len);
	if (err) {
		return err;
	}

	return cmd_send(buf, buf_len, bt_send_rsp);
}

int bt_le_adv_start(const struct bt_le_adv_param *param,
		    const struct bt_data *ad, size_t ad_len,
		    const struct bt_data *sd, size_t sd_len)
{
	int err;
	u8_t buf[2*APP_TO_NETWORK_MAX_PKT_SIZE];
	size_t buf_len = sizeof(buf);

	err = bt_le_adv_start_encode(param, ad, ad_len, sd, sd_len, buf, &buf_len);
	if (err) {
		return err;
	}

	return cmd_send(buf, buf_len, bt_send_rsp);
}

void bt_conn_cb_register(struct bt_conn_cb *cb)
{
	cb->_next = callback_list;
	callback_list = cb;
}

#endif
