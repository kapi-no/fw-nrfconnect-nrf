/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>

/* See https://www.kernel.org/doc/Documentation/remoteproc.txt */

#if defined(CONFIG_TRUSTED_EXECUTION_NONSECURE)
#error "Platform-specific API supported only in Secure TrustZone-M domain"
#endif

/* TODO This should come from DTS, possibly an overlay. */
#define CPU1_UARTE_PIN_TX  25
#define CPU1_UARTE_PIN_RX  26
#define CPU1_UARTE_PIN_RTS 10
#define CPU1_UARTE_PIN_CTS 12

int remoteproc_mgr_boot(void)
{
    /* TODO: Evaluate if this goes into a remoteproc_mgr_config() function,
     * or if we would like to do this configuration (GPIO, and other?) as 
     * part of the pre/post-kernel boot process. For example, the SPM 
     * configuration of peripheral security (secure vs non-secure), has its
     * own Kconfig, along with the actual configuration happening automatically.
     * Perhaps we would like to mimic this for the GPIO (and other?) configuration.
     */
    /* LEDs */
    /* Allow Network MCU to use LED 0 and LED 1 on the DK */
    NRF_P0->PIN_CNF[DT_GPIO_LEDS_LED_0_GPIOS_PIN] =
        GPIO_PIN_CNF_MCUSEL_NetworkMCU << GPIO_PIN_CNF_MCUSEL_Pos;
    NRF_P0->PIN_CNF[DT_GPIO_LEDS_LED_1_GPIOS_PIN] =
        GPIO_PIN_CNF_MCUSEL_NetworkMCU << GPIO_PIN_CNF_MCUSEL_Pos;

    /* UARTE */
    /* Assign specific GPIOs that can be used to get UART from Network */
    NRF_P0->PIN_CNF[CPU1_UARTE_PIN_TX] =
        GPIO_PIN_CNF_MCUSEL_NetworkMCU << GPIO_PIN_CNF_MCUSEL_Pos;
    NRF_P0->PIN_CNF[CPU1_UARTE_PIN_RX] =
        GPIO_PIN_CNF_MCUSEL_NetworkMCU << GPIO_PIN_CNF_MCUSEL_Pos;
    NRF_P0->PIN_CNF[CPU1_UARTE_PIN_RTS] =
        GPIO_PIN_CNF_MCUSEL_NetworkMCU << GPIO_PIN_CNF_MCUSEL_Pos;
    NRF_P0->PIN_CNF[CPU1_UARTE_PIN_CTS] =
        GPIO_PIN_CNF_MCUSEL_NetworkMCU << GPIO_PIN_CNF_MCUSEL_Pos;

    /* Retain NETWORK MCU in Secure domain */
    /* To be decided: Should this also be a configuration option?
     * Should NETWORK always be kept in Non-Secure mode? Why would it 
     * Need to be secure?
     */
    NRF_SPU->EXTDOMAIN[0].PERM = 1 << 4;

	/* Release the Network MCU, 'Release force off signal' */
	NRF_RESET_S->NETWORK.FORCEOFF = RESET_NETWORK_FORCEOFF_FORCEOFF_Release;

	return 0;
}

void remoteproc_mgr_shutdown(void)
{
	/* Turn off the Network MCU, 'Hold force off signal' */
	NRF_RESET_S->NETWORK.FORCEOFF = RESET_NETWORK_FORCEOFF_FORCEOFF_Hold;
}
