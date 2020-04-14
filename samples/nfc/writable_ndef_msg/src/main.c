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
#include <stdbool.h>
#include <nfc_t4t_lib.h>

#include "ndef_file_m.h"
#include <nfc/ndef/nfc_ndef_msg.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(main, 3);

#include <dk_buttons_and_leds.h>

#define NFC_FIELD_LED		DK_LED1
#define NFC_WRITE_LED		DK_LED2
#define NFC_READ_LED		DK_LED4

#define NDEF_RESTORE_BTN_MSK	DK_BTN1_MSK

volatile bool     m_sched_delayed_resp = false;  // Flag indicating to send delayed resp
volatile uint32_t m_delay_time = 0;              // Delay Time in us.

static const uint8_t ack_resp[]  = {'A', 'C', 'K'};
static const uint8_t nack_resp[] = {'N', 'A', 'C', 'K'};

static u8_t ndef_msg_buf[CONFIG_NDEF_FILE_SIZE]; /**< Buffer for NDEF file. */

enum {
	FLASH_WRITE_FINISHED,
	FLASH_BUF_PREP_STARTED,
	FLASH_BUF_PREP_FINISHED,
	FLASH_WRITE_STARTED,
};
static atomic_t op_flags;
static u8_t flash_buf[CONFIG_NDEF_FILE_SIZE]; /**< Buffer for flash update. */
static u8_t flash_buf_len; /**< Length of the flash buffer. */

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

/**
 * @brief Callback function for handling NFC events.
 */
static void nfc_callback(void *context,
			 enum nfc_t4t_event event,
			 const u8_t *data,
			 size_t data_length,
			 u32_t flags)
{
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
                // m_delay_time = uint32_big_decode(data);
		printk("Data ind\n");
		// LOG_HEXDUMP_INF(data, data_length, "Xx");
                m_sched_delayed_resp = true;
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


	/* Restore default NDEF message if button is pressed. */
	u32_t button_state;

	dk_read_buttons(&button_state, NULL);
	if (button_state & NDEF_RESTORE_BTN_MSK) {
		printk("Default NDEF message restored!\n");
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
		if (m_sched_delayed_resp)
		{
			k_usleep(32*5000);
			nfc_t4t_response_pdu_send(ack_resp, sizeof(ack_resp));
			m_sched_delayed_resp = false;
        	}
	}

fail:
	#if CONFIG_REBOOT
		sys_reboot(SYS_REBOOT_COLD);
	#endif /* CONFIG_REBOOT */
		return -EIO;
}
/** @} */
