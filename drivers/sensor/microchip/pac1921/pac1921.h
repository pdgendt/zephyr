/*
 * Copyright (c) 2025, Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_PAC1921_PAC1921_H_
#define ZEPHYR_DRIVERS_SENSOR_PAC1921_PAC1921_H_

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

#define DT_DRV_COMPAT microchip_pac1921

#define PAC1921_REG_GAIN_CFG 0x00
#define PAC1921_REG_INT_CFG  0x01
#define PAC1921_REG_VBUS     0x10
#define PAC1921_REG_VSENSE   0x12
#define PAC1921_REG_VPOWER   0x1d

struct pac1921_reg_int_cfg {
	union {
		struct {
			uint8_t inten: 1;
			uint8_t riov: 1;
			uint8_t vbfen: 1;
			uint8_t vsfen: 1;
			uint8_t smpl: 4;
		};
		uint8_t raw;
	};
};

#define PAC1921_ANY_HAS_GPIO DT_ANY_INST_HAS_PROP_STATUS_OKAY(read_int_gpios)

struct pac1921_config {
	struct i2c_dt_spec bus;
#if PAC1921_ANY_HAS_GPIO
	struct gpio_dt_spec read_int;
#endif
};

struct pac1921_data {
	struct rtio *rtio_ctx;
	struct rtio_iodev *iodev;
	struct rtio_iodev_sqe *streaming_sqe;
	struct pac1921_reg_int_cfg int_cfg;
	k_timeout_t period;
};

struct pac1921_decoder_header {
	uint64_t timestamp;
	bool is_fifo;
};

struct pac1921_rtio_data {
	struct pac1921_decoder_header header;
	uint16_t raw;
};

int pac1921_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder);

#endif /* ZEPHYR_DRIVERS_SENSOR_PAC1921_PAC1921_H_ */
