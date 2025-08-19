# Copyright (c) 2020 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_ZMK_BATTERY_VOLTAGE_DIVIDER battery_voltage_divider.c)