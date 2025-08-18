# Copyright (c) 2020 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_SOA44_BATTERY_VOLTAGE_DIVIDER battery_voltage_divider.c)
zephyr_library_sources(battery_common.c)