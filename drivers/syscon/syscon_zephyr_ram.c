/*
 * Copyright (c) 2025 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_syscon_ram

#include <zephyr/drivers/syscon.h>
#include <zephyr/kernel.h>

struct syscon_ram_config {
	void *buf;
	size_t size;
	uint8_t reg_width;
};

static int syscon_ram_read_reg(const struct device *dev, uint16_t reg, uint32_t *val)
{
	const struct syscon_ram_config *cfg = dev->config;

	if (reg >= cfg->size / cfg->reg_width) {
		return -EINVAL;
	}

	switch (cfg->reg_width) {
	case 1:
		*val = ((uint8_t *)cfg->buf)[reg];
		break;
	case 2:
		*val = ((uint16_t *)cfg->buf)[reg];
		break;
	case 4:
		*val = ((uint32_t *)cfg->buf)[reg];
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int syscon_ram_write_reg(const struct device *dev, uint16_t reg, uint32_t val)
{
	const struct syscon_ram_config *cfg = dev->config;

	if (reg >= cfg->size / cfg->reg_width) {
		return -EINVAL;
	}

	switch (cfg->reg_width) {
	case 1:
		((uint8_t *)cfg->buf)[reg] = val & 0xff;
		break;
	case 2:
		((uint16_t *)cfg->buf)[reg] = val & 0xffff;
		break;
	case 4:
		((uint32_t *)cfg->buf)[reg] = val;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int syscon_ram_get_base(const struct device *dev, uintptr_t *addr)
{
	const struct syscon_ram_config *cfg = dev->config;

	*addr = (uintptr_t)cfg->buf;

	return 0;
}

static int syscon_ram_get_size(const struct device *dev, size_t *size)
{
	const struct syscon_ram_config *cfg = dev->config;

	*size = cfg->size;

	return 0;
}

static int syscon_ram_get_reg_width(const struct device *dev)
{
	const struct syscon_ram_config *cfg = dev->config;

	return cfg->reg_width;
}

static DEVICE_API(syscon, syscon_ram_driver_api) = {
	.read = syscon_ram_read_reg,
	.write = syscon_ram_write_reg,
	.get_base = syscon_ram_get_base,
	.get_size = syscon_ram_get_size,
	.get_reg_width = syscon_ram_get_reg_width,
};

static int syscon_ram_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

#define SYSCON_INIT(inst)                                                                          \
	static uint8_t syscon_ram_buf_##inst[DT_INST_PROP(inst, size)];                            \
	static const struct syscon_ram_config syscon_ram_config_##inst = {                         \
		.buf = syscon_ram_buf_##inst,                                                      \
		.size = DT_INST_PROP(inst, size),                                                  \
		.reg_width = DT_INST_PROP_OR(inst, reg_io_width, 4),                               \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, syscon_ram_init, NULL, NULL, &syscon_ram_config_##inst,        \
			      PRE_KERNEL_1, CONFIG_SYSCON_INIT_PRIORITY, &syscon_ram_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SYSCON_INIT);
