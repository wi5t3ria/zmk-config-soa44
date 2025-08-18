/*
 * Copyright (c) 2021 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>
#include <zephyr/drivers/sensor.h>

#include "battery_common.h"

int battery_channel_get(const struct battery_value *value, enum sensor_channel chan,
                        struct sensor_value *val_out) {
    switch (chan) {
    case SENSOR_CHAN_GAUGE_VOLTAGE:
        val_out->val1 = value->millivolts / 1000;
        val_out->val2 = (value->millivolts % 1000) * 1000U;
        break;

    case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
        val_out->val1 = value->state_of_charge;
        val_out->val2 = 0;
        break;

    default:
        return -ENOTSUP;
    }

    return 0;
}

uint8_t lithium_ion_mv_to_pct(int16_t bat_mv) {
    // Simple linear approximation of a battery based off adafruit's discharge graph
    // https://learn.adafruit.com/li-ion-and-lipoly-batteries/voltages

    if (bat_mv >= 4200) {
        return 100;
    } else if (bat_mv <= 3450) {
        return 0;
    }

    return bat_mv * 2 / 15 - 459;
}

uint8_t mv_to_pct_linear_interpolation(int16_t bat_mv, int16_t *mv_thresholds,
                                       size_t mv_thresholds_size) {
    if (bat_mv < mv_thresholds[0]) {
        return 0;
    }
    if (bat_mv >= mv_thresholds[mv_thresholds_size - 1]) {
        return 100;
    }
    for (size_t i = 1; i < mv_thresholds_size; i++) {
        if (bat_mv < mv_thresholds[i]) {
            int low = mv_thresholds[i - 1];
            int high = mv_thresholds[i];
            
            // Linear interpolation between low and high thresholds
            // Calculate percentage based on position between thresholds
            int voltage_range = high - low;
            int voltage_offset = bat_mv - low;
            int threshold_step = 100 / (mv_thresholds_size - 1);
            int base_percentage = (i - 1) * threshold_step;
            
            return base_percentage + (voltage_offset * threshold_step) / voltage_range;
        }
    }
    return 100;
}