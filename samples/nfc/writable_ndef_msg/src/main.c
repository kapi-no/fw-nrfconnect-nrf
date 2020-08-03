/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

/** @file
 *
 * @defgroup nfc_writable_ndef_msg_example_main main.c
 * @{
 * @ingroup nfc_writable_ndef_msg_example
 * @brief The application main file of NFC writable NDEF message example.
 *
 */

#include <zephyr.h>
#include <power/reboot.h>
#include <stdbool.h>
#include <nfc_t4t_lib.h>

#include "ndef_file_m.h"
#include <nfc/ndef/msg.h>

#include <dk_buttons_and_leds.h>

#define NFC_FIELD_LED		DK_LED1
#define NFC_WRITE_LED		DK_LED2
#define NFC_READ_LED		DK_LED4

#define NDEF_RESTORE_BTN_MSK	DK_BTN1_MSK

static uint8_t ndef_msg_buf[CONFIG_NDEF_FILE_SIZE]; /**< Buffer for NDEF file. */

enum {
	FLASH_WRITE_FINISHED,
	FLASH_BUF_PREP_STARTED,
	FLASH_BUF_PREP_FINISHED,
	FLASH_WRITE_STARTED,
};
static atomic_t op_flags;
static uint8_t flash_buf[CONFIG_NDEF_FILE_SIZE]; /**< Buffer for flash update. */
static uint8_t flash_buf_len; /**< Length of the flash buffer. */

static void flash_buffer_prepare(size_t data_length)
{
	if (atomic_cas(&op_flags, FLASH_WRITE_FINISHED,
			FLASH_BUF_PREP_STARTED)) {
		flash_buf_len = data_length + NLEN_FIELD_SIZE;
		memcpy(flash_buf, ndef_msg_buf, sizeof(flash_buf));

		atomic_set(&op_flags, FLASH_BUF_PREP_FINISHED);
	} else {
		printk("Flash update pending. Discarding new data...\n");
	}

}

static char test_data[] = {
	'L', 'o', 'r', 'e', 'm', ' ', 'i', 'p', 's', 'u', 'm', ' ', 'd', 'o', 'l', 'o', 'r', ' ', 's', 'i', 't', ' ', 'a', 'm', 'e', 't', ',', ' ', 'c', 'o', 'n', 's', 'e', 'c', 't', 'e', 't', 'u', 'r', ' ', 
	'a', 'd', 'i', 'p', 'i', 's', 'c', 'i', 'n', 'g', ' ', 'e', 'l', 'i', 't', '.', ' ', 'P', 'r', 'o', 'i', 'n', ' ', 'n', 'i', 'b', 'h', ' ', 'a', 'u', 'g', 'u', 'e', ',', ' ', 's', 'u', 's', 'c', 'i', 
	'p', 'i', 't', ' ', 'a', ',', ' ', 's', 'c', 'e', 'l', 'e', 'r', 'i', 's', 'q', 'u', 'e', ' ', 's', 'e', 'd', ',', ' ', 'l', 'a', 'c', 'i', 'n', 'i', 'a', ' ', 'i', 'n', ',', ' ', 'm', 'i', '.', ' ', 
	'C', 'r', 'a', 's', ' ', 'v', 'e', 'l', ' ', 'l', 'o', 'r', 'e', 'm', '.', ' ', 'E', 't', 'i', 'a', 'm', ' ', 'p', 'e', 'l', 'l', 'e', 'n', 't', 'e', 's', 'q', 'u', 'e', ' ', 'a', 'l', 'i', 'q', 'u', 
	'e', 't', ' ', 't', 'e', 'l', 'l', 'u', 's', '.', 'L', 'o', 'r', 'e', 'm', ' ', 'i', 'p', 's', 'u', 'm', ' ', 'd', 'o', 'l', 'o', 'r', ' ', 's', 'i', 't', ' ', 'a', 'm', 'e', 't', ',', ' ', 'c', 'o', 
	'n', 's', 'e', 'c', 't', 'e', 't', 'u', 'r', ' ', 'a', 'd', 'i', 'p', 'i', 's', 'c', 'i', 'n', 'g', ' ', 'e', 'l', 'i', 't', '.', ' ', 'P', 'r', 'o', 'i', 'n', ' ', 'n', 'i', 'b', 'h', ' ', 'a', 'u', 
	'g', 'u', 'e', ',', ' ', 's', 'u', 's', 'c', 'i', 'p', 'i', 't', ' ', 'a', ',', ' ', 's', 'c', 'e', 'l', 'e', 'r', 'i', 's', 'q', 'u', 'e', ' ', 's', 'e', 'd', ',', ' ', 'l', 'a', 'c', 'i', 'n', 'i', 
	'a', ' ', 'i', 'n', ',', ' ', 'm', 'i', '.', ' ', 'C', 'r', 'a', 's', ' ', 'v', 'e', 'l', ' ', 'l', 'o', 'r', 'e', 'm', '.', ' ', 'E', 't', 'i', 'a', 'm', ' ', 'p', 'e', 'l', 'l', 'e', 'n', 't', 'e', 
	's', 'q', 'u', 'e', ' ', 'a', 'l', 'i', 'q', 'u', 'e', 't', ' ', 't', 'e', 'l', 'l', 'u', 's', '.', 
};
static bool tx_to_be_sent = false;

/**
 * @brief Callback function for handling NFC events.
 */
static void nfc_callback(void *context,
			 enum nfc_t4t_event event,
			 const uint8_t *data,
			 size_t data_length,
			 uint32_t flags)
{
	int err;

	ARG_UNUSED(context);
	ARG_UNUSED(data);
	ARG_UNUSED(flags);

	switch (event) {
	case NFC_T4T_EVENT_FIELD_ON:
		dk_set_led_on(NFC_FIELD_LED);
		break;

	case NFC_T4T_EVENT_FIELD_OFF:
		dk_set_leds(DK_NO_LEDS_MSK);
		break;

	case NFC_T4T_EVENT_NDEF_READ:
		dk_set_led_on(NFC_READ_LED);
		break;

	case NFC_T4T_EVENT_NDEF_UPDATED:
		if (data_length > 0) {
			dk_set_led_on(NFC_WRITE_LED);
			flash_buffer_prepare(data_length);
		}
		break;

        case NFC_T4T_EVENT_DATA_IND:
		printk("Data ind\n");
		tx_to_be_sent = true;
		break;

	default:
		break;
	}
}

static int board_init(void)
{
	int err;

	err = dk_buttons_init(NULL);
	if (err) {
		printk("Cannot init buttons (err: %d)\n", err);
		return err;
	}

	err = dk_leds_init();
	if (err) {
		printk("Cannot init LEDs (err: %d)\n", err);
	}

	return err;
}

/**
 * @brief   Function for application main entry.
 */
int main(void)
{
	printk("Starting Nordic NFC Writable NDEF Message example\n");

	/* Configure LED-pins as outputs. */
	if (board_init() < 0) {
		printk("Cannot initialize board!\n");
		goto fail;
	}

	/* Set up NFC */
	int err = nfc_t4t_setup(nfc_callback, NULL);

	if (err < 0) {
		printk("Cannot setup t4t library!\n");
		goto fail;
	}

	/* Start sensing NFC field */
	if (nfc_t4t_emulation_start() < 0) {
		printk("Cannot start emulation!\n");
		goto fail;
	}
	printk("Starting NFC Writable NDEF Message example\n");

	while (true) {
		if (atomic_cas(&op_flags, FLASH_BUF_PREP_FINISHED,
				FLASH_WRITE_STARTED)) {
			if (ndef_file_update(flash_buf, flash_buf_len) < 0) {
				printk("Cannot flash NDEF message!\n");
			} else {
				printk("NDEF message successfully flashed.\n");
			}

			atomic_set(&op_flags, FLASH_WRITE_FINISHED);
		}

		if (tx_to_be_sent) {
			tx_to_be_sent = false;
			printk("Data sending\n");

			err = nfc_t4t_response_pdu_send(test_data, sizeof(test_data));
			if (err) {
				printk("Data ind err: %d\n", err);
			}
		}


		__WFE();
	}

fail:
	#if CONFIG_REBOOT
		sys_reboot(SYS_REBOOT_COLD);
	#endif /* CONFIG_REBOOT */
		return -EIO;
}
/** @} */
