/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_SER_H_
#define BT_SER_H_

/**
 * @file
 * @defgroup bt_ser BLE Nordic UART Service serialization
 * @{
 * @brief BLE Nordic UART(NUS) serialization API.
 */

#ifdef __cplusplus
extern "C" {
#endif

#define BT_EVENT_READY 0x01
#define BT_EVENT_CONNECTED 0x02
#define BT_EVENT_DISCONNECTED 0x03

/**@brief Parse received data from other core. This function
 *        should be used on Application and Network core always
 *        when serialization data was received.
 *
 * @param[in] data Received serialization data.
 * @param[in] len Data length.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int bt_rx_parse(const u8_t *data, size_t len);

int bt_evt_send(u8_t evt, const u8_t *evt_data, size_t length);

#ifdef __cplusplus
}
#endif

/**
 *@}
 */

#endif /* BT_SER_H_ */
