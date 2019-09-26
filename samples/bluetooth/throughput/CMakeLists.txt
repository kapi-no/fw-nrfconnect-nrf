#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

cmake_minimum_required(VERSION 3.8.2)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(NONE)

target_include_directories(app PRIVATE remote/)

FILE(GLOB app_sources src/*.c)
# NORDIC SDK APP START
target_sources(app PRIVATE
	${app_sources}
)

if(CONFIG_SOC_NRF5340_CPU0)
	FILE(GLOB remote_sources remote/*.c)
	target_sources(app PRIVATE
		${remote_sources}
	)
endif()
# NORDIC SDK APP END

zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)
