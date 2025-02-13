/*
 * Copyright (c) 2025 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_gpio_reset

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/reset.h>

struct reset_gpio_config {
	struct gpio_dt_spec *gpios;
	size_t num_gpios;

	uint16_t assert_time;
	uint16_t deassert_time;
};

static int reset_gpio_status(const struct device *dev, uint32_t id, uint8_t *status)
{
	const struct reset_gpio_config *cfg = dev->config;
	int ret;

	if (id >= cfg->num_gpios) {
		return -EINVAL;
	}

	ret = gpio_pin_get_dt(&cfg->gpios[id]);
	if (ret < 0) {
		return ret;
	}

	*status = ret;

	return 0;
}

static int reset_gpio_line_assert(const struct device *dev, uint32_t id)
{
	const struct reset_gpio_config *cfg = dev->config;
	int ret;

	if (id >= cfg->num_gpios) {
		return -EINVAL;
	}

	ret = gpio_pin_set_dt(&cfg->gpios[id], 1);
	if (ret < 0) {
		return ret;
	}

	if (cfg->assert_time > 0) {
		k_msleep(cfg->assert_time);
	}

	return 0;
}

static int reset_gpio_line_deassert(const struct device *dev, uint32_t id)
{
	const struct reset_gpio_config *cfg = dev->config;
	int ret;

	if (id >= cfg->num_gpios) {
		return -EINVAL;
	}

	ret = gpio_pin_set_dt(&cfg->gpios[id], 0);
	if (ret < 0) {
		return ret;
	}

	if (cfg->deassert_time > 0) {
		k_msleep(cfg->deassert_time);
	}

	return 0;
}

static int reset_gpio_line_toggle(const struct device *dev, uint32_t id)
{
	int ret = reset_gpio_line_assert(dev, id);
	if (ret < 0) {
		return ret;
	}

	return reset_gpio_line_deassert(dev, id);
}

static int reset_gpio_init(const struct device *dev)
{
	const struct reset_gpio_config *cfg = dev->config;
	int ret;

	for (size_t i = 0; i < cfg->num_gpios; ++i) {
		if (!gpio_is_ready_dt(&cfg->gpios[i])) {
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->gpios[i], GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static DEVICE_API(reset, reset_gpio_driver_api) = {
	.status = reset_gpio_status,
	.line_assert = reset_gpio_line_assert,
	.line_deassert = reset_gpio_line_deassert,
	.line_toggle = reset_gpio_line_toggle,
};

#define RESET_GPIO_INIT(inst)                                                                      \
	static const struct gpio_dt_spec reset_gpio_pins_##inst[] = {                              \
		DT_INST_FOREACH_PROP_ELEM_SEP(inst, gpios, GPIO_DT_SPEC_GET_BY_IDX, (, )),         \
	};                                                                                         \
	static const struct reset_gpio_config reset_gpio_config_##inst = {                         \
		.gpios = reset_gpio_pins_##inst,                                                   \
		.num_gpios = ARRAY_SIZE(reset_gpio_pins_##inst),                                   \
		.assert_time = DT_INST_PROP_OR(inst, assert_time, 0),                              \
		.deassert_time = DT_INST_PROP_OR(inst, assert_time, 0),                            \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, reset_gpio_init, NULL, NULL, &reset_gpio_config_##inst,        \
			      PRE_KERNEL_1, CONFIG_RESET_INIT_PRIORITY, &reset_gpio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RESET_GPIO_INIT);
