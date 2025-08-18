/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT soa44_battery_voltage_divider

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_ADC_NRFX_SAADC
#include <hal/nrf_saadc.h>
#endif

#include "battery_common.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct io_channel_config {
    uint8_t channel;
};

struct bvd_config {
    struct io_channel_config io_channel;
    struct gpio_dt_spec power;
    uint32_t output_ohm;
    uint32_t full_ohm;
    int16_t *mv_to_pct_thresholds;
    uint8_t mv_to_pct_thresholds_size;
};

struct bvd_data {
    const struct device *adc;
    struct adc_channel_cfg acc;
    struct adc_sequence as;
    struct battery_value value;
};

static int bvd_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct bvd_data *drv_data = dev->data;
    const struct bvd_config *drv_cfg = dev->config;
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
    drv_data->as.calibrate = false;

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

    int32_t val = drv_data->value.adc_raw;

    adc_raw_to_millivolts(adc_ref_internal(drv_data->adc), drv_data->acc.gain, drv_data->as.resolution, &val);

    uint16_t millivolts = val * drv_cfg->full_ohm / drv_cfg->output_ohm;
    drv_data->value.millivolts = millivolts;

    uint8_t percent;
    if (drv_cfg->mv_to_pct_thresholds_size > 0) {
        percent = mv_to_pct_linear_interpolation(millivolts, drv_cfg->mv_to_pct_thresholds,
                                                 drv_cfg->mv_to_pct_thresholds_size);
    } else {
        percent = lithium_ion_mv_to_pct(millivolts);
    }
    drv_data->value.state_of_charge = percent;

    LOG_DBG("ADC raw: %d, millivolts: %d, percent: %d", drv_data->value.adc_raw, millivolts, percent);

    return rc;
}

static int bvd_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val_out) {
    struct bvd_data *drv_data = dev->data;
    return battery_channel_get(&drv_data->value, chan, val_out);
}

static const struct sensor_driver_api bvd_api = {
    .sample_fetch = bvd_sample_fetch,
    .channel_get = bvd_channel_get,
};

static int bvd_init(const struct device *dev) {
    struct bvd_data *drv_data = dev->data;
    const struct bvd_config *drv_cfg = dev->config;

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
        .input_positive = NRF_SAADC_INPUT_AIN0 + drv_cfg->io_channel.channel,
#endif
    };

    drv_data->as = (struct adc_sequence){
        .buffer = &drv_data->value.adc_raw,
        .buffer_size = sizeof(drv_data->value.adc_raw),
        .resolution = 12,
        .channels = BIT(drv_cfg->io_channel.channel),
        .calibrate = true,
    };

    adc_channel_setup(drv_data->adc, &drv_data->acc);

    return 0;
}

#define BVD_INST(inst) \
    static int16_t mv_to_pct_thresholds_##inst[] = DT_INST_PROP_OR(inst, mv_to_pct_thresholds, {}); \
    static const struct bvd_config bvd_cfg_##inst = { \
        .io_channel = {.channel = DT_INST_IO_CHANNELS_INPUT(inst)}, \
        .power = GPIO_DT_SPEC_INST_GET_OR(inst, power_gpios, {}), \
        .output_ohm = DT_INST_PROP(inst, output_ohms), \
        .full_ohm = DT_INST_PROP(inst, full_ohms), \
        .mv_to_pct_thresholds = mv_to_pct_thresholds_##inst, \
        .mv_to_pct_thresholds_size = ARRAY_SIZE(mv_to_pct_thresholds_##inst), \
    }; \
    static struct bvd_data bvd_data_##inst; \
    DEVICE_DT_INST_DEFINE(inst, &bvd_init, NULL, &bvd_data_##inst, &bvd_cfg_##inst, POST_KERNEL, \
                          CONFIG_SENSOR_INIT_PRIORITY, &bvd_api);

DT_INST_FOREACH_STATUS_OKAY(BVD_INST)