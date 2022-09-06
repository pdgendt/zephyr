/*
 * Copyright (c) 2022, Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_gpio_revision

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
	     "A single zephyr,gpio-revision compatible is rquired");

#define GPIO_AND_COMMA(_node_id, _prop, _idx) \
	GPIO_DT_SPEC_GET_BY_IDX(_node_id, _prop, _idx),

static const struct gpio_dt_spec revision_gpios[] = {
	DT_INST_FOREACH_PROP_ELEM(0, gpios, GPIO_AND_COMMA)
};

int z_impl_hwinfo_get_device_revision(uint32_t *revision)
{
	uint32_t result = 0;

	for (int i = 0; i < ARRAY_SIZE(revision_gpios); ++i) {
		if (!device_is_ready(revision_gpios[i].port)) {
			return -ENODEV;
		}

		result <<= 1;
		result |= gpio_pin_get_dt(&revision_gpios[i]) ? 1 : 0;
	}

	*revision = result;

	return 0;
}

static int hwinfo_gpio_rev_init(const struct device *dev)
{
	int ret;

	ARG_UNUSED(dev);

	for (int i = 0; i < ARRAY_SIZE(revision_gpios); ++i) {
		if (!device_is_ready(revision_gpios[i].port)) {
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&revision_gpios[i], GPIO_INPUT);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

SYS_INIT(hwinfo_gpio_rev_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
