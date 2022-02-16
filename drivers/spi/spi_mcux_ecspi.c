/*
 * Copyright (c) 2022, Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_imx_ecspi

#include <errno.h>
#include <drivers/spi.h>
#include <drivers/clock_control.h>
#include <fsl_ecspi.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(spi_mcux_ecspi, CONFIG_SPI_LOG_LEVEL);

#include "spi_context.h"

struct spi_mcux_config {
	ECSPI_Type *base;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	void (*irq_config_func)(const struct device *dev);
};
struct spi_mcux_data {
	const struct device *dev;
	ecspi_master_handle_t handle;
	struct spi_context ctx;
	size_t transfer_len;
	uint32_t tx_data[CONFIG_SPI_MCUX_ECSPI_MAX_TRANSFER_SIZE];
	uint32_t rx_data[CONFIG_SPI_MCUX_ECSPI_MAX_TRANSFER_SIZE];
};

static void spi_mcux_transfer_next_packet(const struct device *dev)
{
	const struct spi_mcux_config *config = dev->config;
	struct spi_mcux_data *data = dev->data;
	ECSPI_Type *base = config->base;
	struct spi_context *ctx = &data->ctx;
	ecspi_transfer_t transfer;
	status_t status;

	if ((ctx->tx_len == 0) && (ctx->rx_len == 0)) {
		/* nothing left to rx or tx, we're done! */
		spi_context_cs_control(&data->ctx, false);
		spi_context_complete(&data->ctx, 0);
		return;
	}

	transfer.channel = ctx->config->slave;

	if (ctx->tx_len == 0) {
		/* rx only, nothing to tx */
		transfer.dataSize = MIN(ctx->rx_len, CONFIG_SPI_MCUX_ECSPI_MAX_TRANSFER_SIZE);
		transfer.txData = NULL;
		transfer.rxData = data->rx_data;
	} else if (ctx->rx_len == 0) {
		/* tx only, nothing to rx */
		transfer.dataSize = MIN(ctx->tx_len, CONFIG_SPI_MCUX_ECSPI_MAX_TRANSFER_SIZE);
		for (size_t i = 0; i < transfer.dataSize; ++i) {
			data->tx_data[i] = ctx->tx_buf[i];
		}
		transfer.txData = data->tx_data;
		transfer.rxData = NULL;
	} else {
		transfer.dataSize = MIN(ctx->tx_len, ctx->rx_len);
		if (transfer.dataSize > CONFIG_SPI_MCUX_ECSPI_MAX_TRANSFER_SIZE) {
			transfer.dataSize = CONFIG_SPI_MCUX_ECSPI_MAX_TRANSFER_SIZE;
		}
		for (size_t i = 0; i < transfer.dataSize; ++i) {
			data->tx_data[i] = ctx->tx_buf[i];
		}
		transfer.txData = data->tx_data;
		transfer.rxData = data->rx_data;
	}

	data->transfer_len = transfer.dataSize;

	status = ECSPI_MasterTransferNonBlocking(base, &data->handle,
						 &transfer);
	if (status != kStatus_Success) {
		LOG_ERR("Transfer could not start");
	}
}

static void spi_mcux_isr(const struct device *dev)
{
	const struct spi_mcux_config *config = dev->config;
	struct spi_mcux_data *data = dev->data;
	ECSPI_Type *base = config->base;

	ECSPI_MasterTransferHandleIRQ(base, &data->handle);
}

static void spi_mcux_master_transfer_callback(ECSPI_Type *base,
		ecspi_master_handle_t *handle, status_t status, void *user_data)
{
	struct spi_mcux_data *data = user_data;

	if (spi_context_rx_buf_on(&data->ctx)) {
		for (size_t i = 0; i < data->transfer_len; ++i) {
			data->ctx.rx_buf[i] = data->rx_data[i] & 0xff;
		}
	}

	spi_context_update_tx(&data->ctx, 1, data->transfer_len);
	spi_context_update_rx(&data->ctx, 1, data->transfer_len);

	spi_mcux_transfer_next_packet(data->dev);
}

static int spi_mcux_configure(const struct device *dev,
			      const struct spi_config *spi_cfg)
{
	const struct spi_mcux_config *config = dev->config;
	struct spi_mcux_data *data = dev->data;
	ECSPI_Type *base = config->base;
	ecspi_master_config_t master_config;
	uint32_t clock_freq;

	if (spi_context_configured(&data->ctx, spi_cfg)) {
		/* This configuration is already in use */
		return 0;
	}

	if (spi_cfg->operation & SPI_HALF_DUPLEX) {
		LOG_ERR("Half-duplex not supported");
		return -ENOTSUP;
	}

	if (SPI_WORD_SIZE_GET(spi_cfg->operation) != 8) {
		LOG_ERR("Only 8-bit supported");
		return -ENOTSUP;
	}

	ECSPI_MasterGetDefaultConfig(&master_config);

	if (spi_cfg->slave > kECSPI_Channel3) {
		LOG_ERR("Slave %d is greater than %d",
			    spi_cfg->slave,
			    kECSPI_Channel3);
		return -EINVAL;
	}

	master_config.channel = (ecspi_channel_source_t)spi_cfg->slave;
	master_config.burstLength = SPI_WORD_SIZE_GET(spi_cfg->operation);

	master_config.channelConfig.polarity =
		(SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPOL)
		? kECSPI_PolarityActiveLow
		: kECSPI_PolarityActiveHigh;

	master_config.channelConfig.phase =
		(SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPHA)
		? kECSPI_ClockPhaseSecondEdge
		: kECSPI_ClockPhaseFirstEdge;

	master_config.chipSelectDelay = 10;

	// if (!(spi_cfg->operation & SPI_TRANSFER_LSB)) {
	// 	LOG_ERR("HW byte re-ordering not supported");
	// 	return -ENOTSUP;
	// }

	master_config.baudRate_Bps = spi_cfg->frequency;

	if (clock_control_get_rate(config->clock_dev, config->clock_subsys,
				   &clock_freq)) {
		return -EINVAL;
	}

	ECSPI_MasterInit(base, &master_config, clock_freq);

	ECSPI_MasterTransferCreateHandle(base, &data->handle,
					 spi_mcux_master_transfer_callback,
					 data);

	data->ctx.config = spi_cfg;

	return 0;
}

static int transceive(const struct device *dev,
		      const struct spi_config *spi_cfg,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      bool asynchronous,
		      struct k_poll_signal *signal)
{
	struct spi_mcux_data *data = dev->data;
	int ret;

	spi_context_lock(&data->ctx, asynchronous, signal, spi_cfg);

	ret = spi_mcux_configure(dev, spi_cfg);
	if (ret) {
		goto out;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	spi_context_cs_control(&data->ctx, true);

	spi_mcux_transfer_next_packet(dev);

	ret = spi_context_wait_for_completion(&data->ctx);
out:
	spi_context_release(&data->ctx, ret);

	return ret;
}

static int spi_mcux_transceive(const struct device *dev,
			       const struct spi_config *spi_cfg,
			       const struct spi_buf_set *tx_bufs,
			       const struct spi_buf_set *rx_bufs)
{
	return transceive(dev, spi_cfg, tx_bufs, rx_bufs, false, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_mcux_transceive_async(const struct device *dev,
				     const struct spi_config *spi_cfg,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs,
				     struct k_poll_signal *async)
{
	return transceive(dev, spi_cfg, tx_bufs, rx_bufs, true, async);
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_mcux_release(const struct device *dev,
			    const struct spi_config *spi_cfg)
{
	struct spi_mcux_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static int spi_mcux_init(const struct device *dev)
{
	int err;
	const struct spi_mcux_config *config = dev->config;
	struct spi_mcux_data *data = dev->data;

	config->irq_config_func(dev);

	err = spi_context_cs_configure_all(&data->ctx);
	if (err < 0) {
		return err;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	data->dev = dev;

	return 0;
}

static const struct spi_driver_api spi_mcux_driver_api = {
	.transceive = spi_mcux_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_mcux_transceive_async,
#endif
	.release = spi_mcux_release,
};

#define SPI_MCUX_ECSPI_INIT(n)						\
	static void spi_mcux_config_func_##n(const struct device *dev);	\
									\
	static const struct spi_mcux_config spi_mcux_config_##n = {	\
		.base = (ECSPI_Type *) DT_INST_REG_ADDR(n),		\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),	\
		.clock_subsys =						\
		(clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),	\
		.irq_config_func = spi_mcux_config_func_##n,		\
	};								\
									\
	static struct spi_mcux_data spi_mcux_data_##n = {		\
		SPI_CONTEXT_INIT_LOCK(spi_mcux_data_##n, ctx),		\
		SPI_CONTEXT_INIT_SYNC(spi_mcux_data_##n, ctx),		\
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)	\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, &spi_mcux_init, NULL,			\
			    &spi_mcux_data_##n,				\
			    &spi_mcux_config_##n, POST_KERNEL,		\
			    CONFIG_SPI_INIT_PRIORITY,			\
			    &spi_mcux_driver_api);			\
									\
	static void spi_mcux_config_func_##n(const struct device *dev)	\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),	\
			    spi_mcux_isr, DEVICE_DT_INST_GET(n), 0);	\
									\
		irq_enable(DT_INST_IRQN(n));				\
	}

DT_INST_FOREACH_STATUS_OKAY(SPI_MCUX_ECSPI_INIT)
