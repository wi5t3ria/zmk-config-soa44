/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_battery_voltage_divider

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_ADC_NRFX_SAADC
#include <nrfx_saadc.h>
#endif

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct io_channel_config {
    uint8_t channel;
};

struct battery_config {
    struct io_channel_config io_channel;
    struct gpio_dt_spec power;
    uint32_t output_ohm;
    uint32_t full_ohm;
};

struct battery_data {
    const struct device *adc;
    struct adc_channel_cfg acc;
    struct adc_sequence as;
    uint16_t adc_raw;
    uint16_t millivolts;
    uint8_t state_of_charge;
};

// Ni-Mh battery voltage to percentage conversion
static uint8_t nimh_mv_to_pct(int16_t bat_mv) {
    // For Ni-Mh 1U battery: 1100mV = 0%, 1400mV = 100%
    if (bat_mv >= 1400) {
        return 100;
    } else if (bat_mv <= 1100) {
        return 0;
    }
    
    // Linear interpolation between 1100mV and 1400mV
    return (bat_mv - 1100) * 100 / (1400 - 1100);
}

static int battery_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct battery_data *drv_data = dev->data;
    const struct battery_config *drv_cfg = dev->config;
    int rc;

    if (drv_cfg->power.port) {
        rc = gpio_pin_set_dt(&drv_cfg->power, 1);
        if (rc != 0) {
            LOG_ERR("Failed to enable voltage divider power GPIO");
            return rc;
        }
        k_sleep(K_MSEC(1));
    }

    rc = adc_read(drv_data->adc, &drv_data->as);

    if (drv_cfg->power.port) {
        int rc2 = gpio_pin_set_dt(&drv_cfg->power, 0);
        if (rc2 != 0) {
            LOG_ERR("Failed to disable voltage divider power GPIO");
        }
    }

    if (rc != 0) {
        LOG_ERR("Failed to read ADC: %d", rc);
        return rc;
    }

    int32_t val = drv_data->adc_raw;
    adc_raw_to_millivolts(adc_ref_internal(drv_data->adc), drv_data->acc.gain, 
                          drv_data->as.resolution, &val);

    // Apply voltage divider calculation
    uint16_t millivolts = val * (uint64_t)drv_cfg->full_ohm / drv_cfg->output_ohm;
    drv_data->millivolts = millivolts;

    // Use Ni-Mh specific conversion
    uint8_t percent = nimh_mv_to_pct(millivolts);
    drv_data->state_of_charge = percent;

    LOG_DBG("ADC raw %d ~ %d mV => %d mV, SoC: %d%%", 
            drv_data->adc_raw, val, millivolts, percent);

    return 0;
}

static int battery_channel_get(const struct device *dev, enum sensor_channel chan, 
                              struct sensor_value *val_out) {
    struct battery_data *drv_data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_GAUGE_VOLTAGE:
        val_out->val1 = drv_data->millivolts / 1000;
        val_out->val2 = (drv_data->millivolts % 1000) * 1000U;
        break;
    case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
        val_out->val1 = drv_data->state_of_charge;
        val_out->val2 = 0;
        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}

static const struct sensor_driver_api battery_api = {
    .sample_fetch = battery_sample_fetch,
    .channel_get = battery_channel_get,
};

static int battery_init(const struct device *dev) {
    struct battery_data *drv_data = dev->data;
    const struct battery_config *drv_cfg = dev->config;

    drv_data->adc = DEVICE_DT_GET(DT_IO_CHANNELS_CTLR(DT_DRV_INST(0)));
    if (!device_is_ready(drv_data->adc)) {
        LOG_ERR("ADC device is not ready");
        return -ENODEV;
    }

    if (drv_cfg->power.port && !device_is_ready(drv_cfg->power.port)) {
        LOG_ERR("Power GPIO device is not ready");
        return -ENODEV;
    }

    if (drv_cfg->power.port) {
        int rc = gpio_pin_configure_dt(&drv_cfg->power, GPIO_OUTPUT_INACTIVE);
        if (rc != 0) {
            LOG_ERR("Failed to configure power GPIO");
            return rc;
        }
    }

    drv_data->acc = (struct adc_channel_cfg){
        .gain = ADC_GAIN_1_6,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id = drv_cfg->io_channel.channel,
#ifdef CONFIG_ADC_NRFX_SAADC
        .input_positive = NRFX_SAADC_INPUT_AIN0 + drv_cfg->io_channel.channel,
#endif
    };

    drv_data->as = (struct adc_sequence){
        .buffer = &drv_data->adc_raw,
        .buffer_size = sizeof(drv_data->adc_raw),
        .resolution = 12,
        .channels = BIT(drv_cfg->io_channel.channel),
        .calibrate = true,
    };

    adc_channel_setup(drv_data->adc, &drv_data->acc);

    return 0;
}

#define BATTERY_INST(inst) \
    static const struct battery_config battery_cfg_##inst = { \
        .io_channel = {.channel = DT_INST_IO_CHANNELS_INPUT(inst)}, \
        .power = GPIO_DT_SPEC_INST_GET_OR(inst, power_gpios, {}), \
        .output_ohm = DT_INST_PROP(inst, output_ohms), \
        .full_ohm = DT_INST_PROP(inst, full_ohms), \
    }; \
    static struct battery_data battery_data_##inst; \
    DEVICE_DT_INST_DEFINE(inst, &battery_init, NULL, &battery_data_##inst, \
                          &battery_cfg_##inst, POST_KERNEL, \
                          CONFIG_SENSOR_INIT_PRIORITY, &battery_api);

DT_INST_FOREACH_STATUS_OKAY(BATTERY_INST)