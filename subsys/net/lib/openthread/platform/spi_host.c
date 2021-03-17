/*
 * Copyright (c) 2021 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME net_otPlat_spi_host

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_OPENTHREAD_LOG_LEVEL);

#include "openthread-core-zephyr-config.h"

#include <openthread-system.h>
#include <openthread/platform/radio.h>
#include <openthread/message.h>
#include <openthread/ip6.h>

#include <net/openthread.h>
#include <net/net_if.h>
#include <drivers/spi_generic.h>

#include "platform-zephyr.h"

K_SEM_DEFINE(recv_sem, 0, UINT32_MAX);
K_FIFO_DEFINE(tx_pkt_fifo);

static const struct device *spi_dev;
static bool spi_polling_only;

int notify_new_rx_frame(struct net_pkt *pkt)
{
	// There isn't any actual radio, openthread stack already handled this
	net_pkt_unref(pkt);

	// Return ok, no need to print errors
	return 0;
}

int notify_new_tx_frame(struct net_pkt *pkt)
{
	k_fifo_put(&tx_pkt_fifo, pkt);
	otSysEventSignalPending();

	return 0;
}

static void openthread_handle_frame_to_send(otInstance *instance,
					    struct net_pkt *pkt)
{
	struct net_buf *buf;
	otMessage *message;
	otMessageSettings settings;

	NET_DBG("Sending Ip6 packet to ot stack");

	settings.mPriority = OT_MESSAGE_PRIORITY_NORMAL;
	settings.mLinkSecurityEnabled = true;
	message = otIp6NewMessage(instance, &settings);
	if (message == NULL) {
		goto exit;
	}

	for (buf = pkt->buffer; buf; buf = buf->frags) {
		if (otMessageAppend(message, buf->data, buf->len) != OT_ERROR_NONE) {
			NET_ERR("Error while appending to otMessage");
			otMessageFree(message);
			goto exit;
		}
	}

	if (otIp6Send(instance, message) != OT_ERROR_NONE) {
		NET_ERR("Error while calling otIp6Send");
		goto exit;
	}

exit:
	net_pkt_unref(pkt);
}

static void interrupt_callback(const struct device *dev, void *user_data)
{
	k_sem_give(&recv_sem);

	otSysEventSignalPending();
}

void otPlatSpiHostInit(void)
{
	spi_dev = device_get_binding(CONFIG_OPENTHREAD_HOSTPROCESSOR_SPINEL_ON_GENERIC_SPI_DEV_NAME);

	if (spi_dev) {
		spi_polling_only = spi_generic_device_interrupt_callback_set(spi_dev, interrupt_callback, NULL) != 0;

		if (spi_generic_device_reset(spi_dev, true) != -ENODEV) {
			k_sleep(K_MSEC(10));

			spi_generic_device_reset(spi_dev, false);

			k_sleep(K_MSEC(10));
		}
	}
}

bool otPlatSpiHostCheckInterrupt(void)
{
	if (spi_polling_only) {
		return true;
	}

	return k_sem_count_get(&recv_sem) > 0;
}

bool otPlatSpiHostWaitForFrame(uint64_t aTimeoutUs)
{
	if (spi_polling_only) {
		return true;
	}

	return k_sem_take(&recv_sem, K_USEC(aTimeoutUs)) == 0;
}

void otPlatSpiHostProcess(otInstance *context)
{
	struct net_pkt *tx_pkt;

	while ((tx_pkt = (struct net_pkt *) k_fifo_get(&tx_pkt_fifo, K_NO_WAIT)) != NULL) {
		openthread_handle_frame_to_send(context, tx_pkt);
	}
}

int otPlatSpiHostTransfer(uint8_t *aSpiTxFrameBuffer,
			  uint8_t *aSpiRxFrameBuffer,
			  uint32_t aTransferLength)
{
	if (!spi_dev) {
		return -ENODEV;
	}

	struct spi_buf tx_buf = {
		.buf = aSpiTxFrameBuffer,
		.len = aTransferLength
	};
	struct spi_buf_set tx_bufs = {
		.buffers = &tx_buf,
		.count = 1,
	};

	struct spi_buf rx_buf = {
		.buf = aSpiRxFrameBuffer,
		.len = aTransferLength
	};
	struct spi_buf_set rx_bufs = {
		.buffers = &rx_buf,
		.count = 1,
	};

	return spi_generic_device_transceive(spi_dev, &tx_bufs, &rx_bufs);
}
