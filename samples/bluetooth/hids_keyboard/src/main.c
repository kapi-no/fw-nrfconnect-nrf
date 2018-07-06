/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: BSD-5-Clause-Nordic
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <misc/printk.h>
#include <misc/byteorder.h>
#include <zephyr.h>
#include <gpio.h>
#include <board.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include <gatt/dis.h>
#include <gatt/bas.h>
#include <bluetooth/services/hids.h>

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define BASE_USB_HID_SPEC_VERSION   0x0101

#define OUTPUT_REPORT_INDEX			0
#define OUTPUT_REPORT_MAX_LEN			1
#define INPUT_REPORT_KEYS_INDEX			0
#define INPUT_REPORT_KEYS_MAX_LEN		8
#define OUTPUT_REPORT_BIT_MASK_CAPS_LOCK	0x02
#define INPUT_REP_REF_ID			0
#define OUTPUT_REP_REF_ID			0
#define MODIFIER_KEY_POS			0
#define SHIFT_KEY_CODE				0x02
#define SCAN_CODE_POS				2
#define KEYS_MAX_LEN				(INPUT_REPORT_KEYS_MAX_LEN - \
						SCAN_CODE_POS)

#define CAPS_LOCK_LED		LED0_GPIO_PIN
#define SHIFT_BUTTON		SW1_GPIO_PIN
#define SHIFT_BUTTON_PORT	1
#define SHIFT_BUTTON_PRESSED	0

struct key_pack {
	void *private;
	u8_t *elements;
	u8_t len;
};

HIDS_DEF(hids_obj); /* HIDS instance. */

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      0x12, 0x18,       /* HID Service */
		      0x0f, 0x18),      /* Battery Service */

};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static u8_t hello_world_str[] = {
	0x0b,	/* Key h */
	0x08,	/* Key e */
	0x0f,	/* Key l */
	0x0f,	/* Key l */
	0x12,	/* Key o */
	0x28,	/* Key Return */
};

static u8_t caps_on_key_scan_str[] = {
	0x06,	/* Key C */
	0x04,	/* Key a */
	0x13,	/* Key p */
	0x16,	/* Key s */
	0x12,	/* Key o */
	0x11,	/* Key n */
};

static u8_t caps_off_key_scan_str[] = {
	0x06,	/* Key C */
	0x04,	/* Key a */
	0x13,	/* Key p */
	0x16,	/* Key s */
	0x12,	/* Key o */
	0x09,	/* Key f */
};

static struct device		*led_dev;
static struct device		*gpio_devs[4];
static struct gpio_callback	gpio_cbs[4];
static bool			in_boot_mode;
static bool			caps_on;
static struct k_fifo		keys_fifo;


static void advertising_start(void)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
				  sd, ARRAY_SIZE(sd));

	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}


static void connected(struct bt_conn *conn, u8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Failed to connect to %s (%u)\n", addr, err);
		return;
	}

	printk("Connected %s\n", addr);

	if (bt_conn_security(conn, BT_SECURITY_MEDIUM)) {
		printk("Failed to set security\n");
	}
}


static void disconnected(struct bt_conn *conn, u8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected from %s (reason %u)\n", addr, reason);

	advertising_start();
}


static void security_changed(struct bt_conn *conn, bt_security_t level)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Security changed: %s level %u\n", addr, level);
}


static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
};



static void key_pack_queue(char *string, u8_t len)
{
	struct key_pack *keys;

	__ASSERT(len <= KEYS_MAX_LEN, "String is too long.");

	printk("Getting key: %02X\n", (char) *string);

	keys = k_malloc(sizeof(struct key_pack));
	if (keys == NULL) {
		printk("Alloc failed\n");
		return;
	}
	keys->elements = string;
	keys->len = len;

	k_fifo_put(&keys_fifo, keys);
}


static void caps_lock_handler(const struct hids_rep *rep)
{
	u8_t report_val = (*rep->data) & OUTPUT_REPORT_BIT_MASK_CAPS_LOCK;

	if (!caps_on && (report_val != 0)) {
		gpio_pin_write(led_dev, CAPS_LOCK_LED, 0);

		key_pack_queue(caps_on_key_scan_str,
			       sizeof(caps_on_key_scan_str));
		caps_on = true;
	} else if (caps_on && (report_val == 0)) {
		gpio_pin_write(led_dev, CAPS_LOCK_LED, 1);

		key_pack_queue(caps_off_key_scan_str,
			       sizeof(caps_off_key_scan_str));
		caps_on = false;
	}
}


static void hids_outp_rep_handler(const struct hids_rep *rep)
{
	printk("Output report has been received\n");
	caps_lock_handler(rep);
}


static void hids_boot_kb_outp_rep_handler(const struct hids_rep *rep)
{
	printk("Boot Keyboard Output report has been received\n");
	caps_lock_handler(rep);
}


static void hids_pm_evt_handler(enum hids_pm_evt evt)
{
	switch (evt) {
	case HIDS_PM_EVT_BOOT_MODE_ENTERED:
		printk("Boot mode entered\n");
		in_boot_mode = true;
		break;

	case HIDS_PM_EVT_REPORT_MODE_ENTERED:
		printk("Report mode entered\n");
		in_boot_mode = false;
		break;

	default:
		break;
	}
}


static void hog_init(void)
{
	int err;
	struct hids_init hids_init_obj = { 0 };
	struct hids_inp_rep *hids_inp_rep;
	struct hids_outp_rep *hids_outp_rep;

	static u8_t inp_buff[INPUT_REPORT_KEYS_MAX_LEN];
	static u8_t outp_buff[OUTPUT_REPORT_MAX_LEN];
	static const u8_t report_map[] = {
		0x05, 0x01,	  /* Usage Page (Generic Desktop) */
		0x09, 0x06,	  /* Usage (Keyboard) */
		0xA1, 0x01,	  /* Collection (Application) */
		0x05, 0x07,	  /* Usage Page (Key Codes) */
		0x19, 0xe0,	  /* Usage Minimum (224) */
		0x29, 0xe7,	  /* Usage Maximum (231) */
		0x15, 0x00,	  /* Logical Minimum (0) */
		0x25, 0x01,	  /* Logical Maximum (1) */
		0x75, 0x01,	  /* Report Size (1) */
		0x95, 0x08,	  /* Report Count (8) */
		0x81, 0x02,	  /* Input (Data, Variable, Absolute) */

		0x95, 0x01,	  /* Report Count (1) */
		0x75, 0x08,	  /* Report Size (8) */
		0x81, 0x01,	  /* Input (Constant) reserved byte(1) */

		0x95, 0x05,	  /* Report Count (5) */
		0x75, 0x01,	  /* Report Size (1) */
		0x05, 0x08,	  /* Usage Page (Page# for LEDs) */
		0x19, 0x01,	  /* Usage Minimum (1) */
		0x29, 0x05,	  /* Usage Maximum (5) */
		0x91, 0x02,	  /* Output (Data, Variable, Absolute), */
				  /* Led report */
		0x95, 0x01,	  /* Report Count (1) */
		0x75, 0x03,	  /* Report Size (3) */
		0x91, 0x01,	  /* Output (Data, Variable, Absolute), */
				  /* Led report padding */

		0x95, 0x06,	  /* Report Count (6) */
		0x75, 0x08,	  /* Report Size (8) */
		0x15, 0x00,	  /* Logical Minimum (0) */
		0x25, 0x65,	  /* Logical Maximum (101) */
		0x05, 0x07,	  /* Usage Page (Key codes) */
		0x19, 0x00,	  /* Usage Minimum (0) */
		0x29, 0x65,	  /* Usage Maximum (101) */
		0x81, 0x00,	  /* Input (Data, Array) Key array(6 bytes) */

		0x09, 0x05,	  /* Usage (Vendor Defined) */
		0x15, 0x00,	  /* Logical Minimum (0) */
		0x26, 0xFF, 0x00, /* Logical Maximum (255) */
		0x75, 0x08,	  /* Report Count (2) */
		0x95, 0x02,	  /* Report Size (8 bit) */
		0xB1, 0x02,	  /* Feature (Data, Variable, Absolute) */

		0xC0		  /* End Collection (Application) */
	};

	hids_init_obj.rep_map.data = report_map;
	hids_init_obj.rep_map.size = sizeof(report_map);

	hids_init_obj.info.bcd_hid = BASE_USB_HID_SPEC_VERSION;
	hids_init_obj.info.b_country_code = 0x00;
	hids_init_obj.info.flags = (HIDS_REMOTE_WAKE |
			HIDS_NORMALLY_CONNECTABLE);

	hids_inp_rep = &hids_init_obj.inp_rep_group_init.reports[0];
	hids_inp_rep->buff.data = inp_buff;
	hids_inp_rep->buff.size = sizeof(inp_buff);
	hids_inp_rep->id = INPUT_REP_REF_ID;
	hids_init_obj.inp_rep_group_init.cnt++;

	hids_outp_rep = &hids_init_obj.outp_rep_group_init.reports[0];
	hids_outp_rep->buff.data = outp_buff;
	hids_outp_rep->buff.size = sizeof(outp_buff);
	hids_outp_rep->id = OUTPUT_REP_REF_ID;
	hids_outp_rep->handler = hids_outp_rep_handler;
	hids_init_obj.outp_rep_group_init.cnt++;

	hids_init_obj.is_kb = true;
	hids_init_obj.boot_kb_outp_rep_handler = hids_boot_kb_outp_rep_handler;
	hids_init_obj.pm_evt_handler = hids_pm_evt_handler;

	err = hids_init(&hids_obj, &hids_init_obj);
	__ASSERT(err == 0, "HIDS initialization failed\n");
}


static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	bas_init();
	dis_init(CONFIG_SOC, "Manufacturer");
	hog_init();

	advertising_start();
}


static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}


static void auth_done(struct bt_conn *conn)
{
	printk("%s()\n", __func__);
	bt_conn_auth_pairing_confirm(conn);
}


static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = NULL,
	.passkey_entry = NULL,
	.cancel = auth_cancel,
	.pairing_confirm = auth_done,
};


void key_press_send(u8_t *string, u8_t len)
{
	u8_t  offset = 0;
	u8_t  data[INPUT_REPORT_KEYS_MAX_LEN];
	u32_t shift_btn;

	if (len > KEYS_MAX_LEN) {
		printk("Text is too big\n");
		return;
	}

	while (offset <= len) {
		memset(data, 0, sizeof(data));
		memcpy(data + SCAN_CODE_POS + offset, string + offset,
		       len - offset);

		gpio_pin_read(gpio_devs[SHIFT_BUTTON_PORT], SHIFT_BUTTON,
			      &shift_btn);
		if (shift_btn == SHIFT_BUTTON_PRESSED) {
			data[MODIFIER_KEY_POS] |= SHIFT_KEY_CODE;
		}

		if (in_boot_mode) {
			hids_boot_kb_inp_rep_send(&hids_obj, data,
						  sizeof(data));
		} else {
			hids_inp_rep_send(&hids_obj, INPUT_REPORT_KEYS_INDEX,
					  data, sizeof(data));
		}
		offset++;
	}
}


void button_pressed(struct device *gpio_dev, struct gpio_callback *cb,
		    u32_t pins)
{
	static u8_t *chr = hello_world_str;

	if (pins & (1 << SW0_GPIO_PIN)) {
		key_pack_queue(chr++, 1);
		if (chr == (hello_world_str + sizeof(hello_world_str))) {
			chr = hello_world_str;
		}
	}
}


void configure_gpio(void)
{
	static const u32_t pin_id[4] = { SW0_GPIO_PIN, SW1_GPIO_PIN,
		SW2_GPIO_PIN, SW3_GPIO_PIN };
	static const char *port_name[4] = { SW0_GPIO_NAME, SW1_GPIO_NAME,
		SW2_GPIO_NAME, SW3_GPIO_NAME };

	for (size_t i = 0; i < ARRAY_SIZE(pin_id); i++) {
		gpio_devs[i] = device_get_binding(port_name[i]);
		if (gpio_devs[i]) {
			printk("%s(): port %zu bound\n", __func__, i);

			gpio_pin_configure(gpio_devs[i], pin_id[i],
					   GPIO_PUD_PULL_UP | GPIO_DIR_IN |
					   GPIO_INT | GPIO_INT_EDGE |
					   GPIO_INT_ACTIVE_LOW);
			gpio_init_callback(&gpio_cbs[i], button_pressed,
					   BIT(pin_id[i]));
			gpio_add_callback(gpio_devs[i], &gpio_cbs[i]);
			gpio_pin_enable_callback(gpio_devs[i], pin_id[i]);
		}
	}

	led_dev = device_get_binding(LED0_GPIO_PORT);

	gpio_pin_configure(led_dev, CAPS_LOCK_LED, GPIO_DIR_OUT);
	gpio_pin_write(led_dev, CAPS_LOCK_LED, 1);
}


void main(void)
{
	int err;

	printk("Start zephyr\n");

	bt_conn_cb_register(&conn_callbacks);
	bt_conn_auth_cb_register(&auth_cb_display);

	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	k_fifo_init(&keys_fifo);
	configure_gpio();

	while (true) {
		struct key_pack *keys;

		keys = k_fifo_get(&keys_fifo, K_FOREVER);
		if ((keys != NULL) && (keys->len != 0)) {

			printk("Sending %d keys:", keys->len);
			for (u8_t i = 0; i < keys->len; i++) {
				printk(" %02X", *(keys->elements + i));
			}
			printk("\n");

			key_press_send(keys->elements, keys->len);
			k_free(keys);
		}
	}
}


