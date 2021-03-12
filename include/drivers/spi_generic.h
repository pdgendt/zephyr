/*
 * Copyright 2021 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SPI_SPI_GENERIC_H_
#define ZEPHYR_INCLUDE_DRIVERS_SPI_SPI_GENERIC_H_

/**
 * @file
 *
 * @brief Public APIs for generic SPI devices.
 */

#include <zephyr/types.h>
#include <device.h>
#include <drivers/spi.h>

#ifdef __cplusplus
extern "C" {
#endif

struct spi_generic_device_api;

typedef void (*spi_generic_interrupt_callback)(const struct device *dev,
				void *user_data);

typedef int (*spi_generic_api_io)(const struct device *dev,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs);
#ifdef CONFIG_SPI_ASYNC
typedef int (*spi_generic_api_io_async)(const struct device *dev,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs,
				struct k_poll_signal *async);
#endif
typedef int (*spi_generic_api_reset)(const struct device *dev, bool active);
typedef int (*spi_generic_api_interrupt_callback_set)(const struct device *dev,
				spi_generic_interrupt_callback user_callback,
				void *user_data);

struct spi_generic_device_api {
	spi_generic_api_io transceive;
#ifdef CONFIG_SPI_ASYNC
	spi_generic_api_io_async transceive_async;
#endif
	spi_generic_api_reset reset;
	spi_generic_api_interrupt_callback_set interrupt_callback_set;
};

static inline int spi_generic_device_transceive(const struct device *dev,
					const struct spi_buf_set *tx_bufs,
					const struct spi_buf_set *rx_bufs)
{
	const struct spi_generic_device_api *api =
		(const struct spi_generic_device_api *)dev->api;

	return api->transceive(dev, tx_bufs, rx_bufs);
}

static inline int spi_generic_device_read(const struct device *dev,
			   const struct spi_buf_set *rx_bufs)
{
	return spi_generic_device_transceive(dev, NULL, rx_bufs);
}

static inline int spi_generic_device_write(const struct device *dev,
			    const struct spi_buf_set *tx_bufs)
{
	return spi_generic_device_transceive(dev, tx_bufs, NULL);
}

#ifdef CONFIG_SPI_ASYNC

static inline int spi_generic_device_transceive_async(const struct device *dev,
				       const struct spi_buf_set *tx_bufs,
				       const struct spi_buf_set *rx_bufs,
				       struct k_poll_signal *async)
{
	const struct spi_generic_device_api *api =
		(const struct spi_generic_device_api *)dev->api;

	return api->transceive_async(dev, tx_bufs, rx_bufs, async);
}

static inline int spi_generic_device_read_async(const struct device *dev,
				 const struct spi_buf_set *rx_bufs,
				 struct k_poll_signal *async)
{
	return spi_generic_device_transceive_async(dev, NULL, rx_bufs, async);
}

static inline int spi_generic_device_write_async(const struct device *dev,
				  const struct spi_buf_set *tx_bufs,
				  struct k_poll_signal *async)
{
	return spi_generic_device_transceive_async(dev, tx_bufs, NULL, async);
}

#endif /* CONFIG_SPI_ASYNC */

static inline int spi_generic_device_reset(const struct device *dev, bool active)
{
	const struct spi_generic_device_api *api =
		(const struct spi_generic_device_api *)dev->api;

	return api->reset(dev, active);
}

static inline int spi_generic_device_interrupt_callback_set(const struct device *dev,
				spi_generic_interrupt_callback user_callback,
				void *user_data)
{
	const struct spi_generic_device_api *api =
		(const struct spi_generic_device_api *)dev->api;

	return api->interrupt_callback_set(dev, user_callback, user_data);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SPI_SPI_GENERIC_H_ */
