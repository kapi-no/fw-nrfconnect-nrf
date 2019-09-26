/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief RemoteProc Mgr
 *
 * This file contains APIs for remoteproc mgr.
 */

#ifndef ZEPHYR_INCLUDE_REMOTEPROC_MGR_REMOTEPROC_MGR_H_
#define ZEPHYR_INCLUDE_REMOTEPROC_MGR_REMOTEPROC_MGR_H_

#include <kernel.h>
#include <zephyr/types.h>
#include <sys/dlist.h>

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Configure and Boot a remote processor.
 *
 * This call is used to configure and start the remote processor.
 *
 * @return Returns 0 in case of success and an error code otherwise.
 */
int remoteproc_mgr_boot(void);

/**@brief   Shutdown remote processor.
 *
 * This call is used to shutdown the remote processor.
 *
 * @return N/A
 */
void remoteproc_mgr_shutdown(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_REMOTEPROC_MGR_REMOTEPROC_MGR_H_ */
