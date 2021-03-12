/*
 * Copyright 2021 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_spi_generic

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(spi_generic_device);

#include <device.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <drivers/spi_generic.h>

/** DTS defined device config */
struct spi_generic_device_config {
	const struct device *spi_dev;
	const struct spi_config spi_config;

	const char *int_gpio_port_name;
	gpio_pin_t int_gpio_pin;
	gpio_flags_t int_gpio_flags;

	const char *rst_gpio_port_name;
	gpio_pin_t rst_gpio_pin;
	gpio_flags_t rst_gpio_flags;
};

/** Working data for the device */
struct spi_generic_device_data {
	/* pointer to the device itself for easy access */
	const struct device *self;

	const struct device *int_gpio_port;
	const struct device *rst_gpio_port;

	struct gpio_callback int_gpio_callback;

	spi_generic_interrupt_callback user_callback;
	void* user_callback_data;
};

static int spi_generic_transceive(const struct device *dev,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	const struct spi_generic_device_config *config = dev->config;

	return spi_transceive(config->spi_dev, &config->spi_config, tx_bufs, rx_bufs);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_generic_transceive_async(const struct device *dev,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs,
				struct k_poll_signal *async)
{
	const struct spi_generic_device_config *config = dev->config;

	return spi_transceive_async(config->spi_dev, &config->spi_config, tx_bufs, rx_bufs, async);
}
#endif

static int spi_generic_reset(const struct device *dev, bool active)
{
	const struct spi_generic_device_config *config = dev->config;
	struct spi_generic_device_data *data = dev->data;

	if (!data->rst_gpio_port) {
		return -ENODEV;
	}

	return gpio_pin_set(data->rst_gpio_port, config->rst_gpio_pin, active);
}

static int spi_generic_interrupt_callback_set(const struct device *dev,
				spi_generic_interrupt_callback user_callback,
				void *user_data)
{
	struct spi_generic_device_data *data = dev->data;

	if (!data->int_gpio_port) {
		return -ENODEV;
	}

	data->user_callback = user_callback;
	data->user_callback_data = user_data;

	if (user_callback) {
		return gpio_add_callback(data->int_gpio_port, &data->int_gpio_callback);
	} else {
		return gpio_remove_callback(data->int_gpio_port, &data->int_gpio_callback);
	}
}

static void spi_generic_interrupt_handler(const struct device *port,
					struct gpio_callback *cb,
					gpio_port_pins_t pins)
{
	const struct spi_generic_device_data *data
			= CONTAINER_OF(cb, struct spi_generic_device_data, int_gpio_callback);

	if (data->user_callback) {
		data->user_callback(data->self, data->user_callback_data);
	}
}

static int spi_generic_init(const struct device *dev)
{
	const struct spi_generic_device_config *config = dev->config;
	struct spi_generic_device_data *data = dev->data;

	data->self = dev;

	/* configure the optional interrupt input gpio */
	data->int_gpio_port = device_get_binding(config->int_gpio_port_name);
	if (data->int_gpio_port) {
		gpio_pin_configure(data->int_gpio_port,
				config->int_gpio_pin,
				GPIO_INPUT | config->int_gpio_flags);

		gpio_pin_interrupt_configure(data->int_gpio_port,
				config->int_gpio_pin,
				GPIO_INT_EDGE_TO_ACTIVE);

		gpio_init_callback(&data->int_gpio_callback, spi_generic_interrupt_handler,
				BIT(config->int_gpio_pin));
	}

	/* configure the optional reset output gpio */
	data->rst_gpio_port = device_get_binding(config->rst_gpio_port_name);
	if (data->rst_gpio_port) {
		gpio_pin_configure(data->rst_gpio_port,
				config->rst_gpio_pin,
				GPIO_OUTPUT | config->rst_gpio_flags);
	}

	return 0;
}

/* Device instantiation */

static struct spi_generic_device_api spi_generic_api = {
	.transceive             = spi_generic_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async       = spi_generic_transceive_async,
#endif
	.reset                  = spi_generic_reset,
	.interrupt_callback_set = spi_generic_interrupt_callback_set,
};

#define SPI_GENERIC_DEVICE_CFG_OPERATION(n) \
		SPI_WORD_SET(DT_INST_PROP_OR(n, spi_bits_per_word, 8)) \
		| COND_CODE_1(DT_INST_PROP(n, spi_slave), (SPI_OP_MODE_SLAVE), (0)) \
		| COND_CODE_1(DT_INST_PROP(n, spi_cpol), (SPI_MODE_CPOL), (0)) \
		| COND_CODE_1(DT_INST_PROP(n, spi_cpha), (SPI_MODE_CPHA), (0)) \
		| COND_CODE_1(DT_INST_PROP(n, spi_loop), (SPI_MODE_LOOP), (0)) \
		| COND_CODE_1(DT_INST_PROP(n, spi_lsb_first), (SPI_TRANSFER_LSB), (0)) \
		| COND_CODE_1(DT_INST_PROP(n, spi_cs_high), (SPI_CS_ACTIVE_HIGH), (0)) \
		| (DT_ENUM_IDX_OR(DT_DRV_INST(n), spi_bus_width, 0) << 11) // # SPI MISO LINES

#define SPI_GENERIC_DEVICE_GPIO_INT(n) \
		.int_gpio_port_name = DT_INST_GPIO_LABEL(n, int_gpios), \
		.int_gpio_pin = DT_INST_GPIO_PIN(n, int_gpios), \
		.int_gpio_flags = DT_INST_GPIO_FLAGS(n , int_gpios),

#define SPI_GENERIC_DEVICE_GPIO_RST(n) \
		.rst_gpio_port_name = DT_INST_GPIO_LABEL(n, reset_gpios), \
		.rst_gpio_pin = DT_INST_GPIO_PIN(n, reset_gpios), \
		.rst_gpio_flags = DT_INST_GPIO_FLAGS(n , reset_gpios),

#define SPI_GENERIC_DEVICE_INIT(n) \
	static const struct spi_generic_device_config spi_generic_cfg_##n = { \
		.spi_dev = DEVICE_DT_GET(DT_INST_BUS(n)), \
		.spi_config = SPI_CONFIG_DT_INST(n, SPI_GENERIC_DEVICE_CFG_OPERATION(n), \
				DT_INST_PROP_OR(n, spi_cs_delay, 0)), \
		COND_CODE_1(DT_INST_NODE_HAS_PROP(n, int_gpios), \
				(SPI_GENERIC_DEVICE_GPIO_INT(n)), ()) \
		COND_CODE_1(DT_INST_NODE_HAS_PROP(n, reset_gpios), \
				(SPI_GENERIC_DEVICE_GPIO_RST(n)), ()) \
	}; \
	static struct spi_generic_device_data spi_generic_data_##n; \
	DEVICE_DT_INST_DEFINE(n, \
				spi_generic_init, \
				device_pm_control_nop, \
				&spi_generic_data_##n, \
				&spi_generic_cfg_##n, \
				POST_KERNEL, \
				CONFIG_SPI_INIT_PRIORITY, \
				&spi_generic_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_GENERIC_DEVICE_INIT)
