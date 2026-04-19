/*
 * SPDX-FileCopyrightText: 2026 Basalte bv
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/ipc/ipc_service.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "i2c_ipc.h"

#define DT_DRV_COMPAT zephyr_i2c_ipc

LOG_MODULE_REGISTER(i2c_ipc, CONFIG_I2C_LOG_LEVEL);

#define BUF_SIZE CONFIG_I2C_IPC_BUFFER_SIZE

struct i2c_ipc_data {
	struct ipc_ept ept;
	struct k_sem bound_sem;
	struct k_sem resp_sem;
	struct k_mutex lock;
	uint8_t tx_buf[BUF_SIZE];
	uint8_t rx_buf[BUF_SIZE];
	size_t rx_len;
};

struct i2c_ipc_config {
	const struct device *ipc;
	const char *ept_name;
};

static void ept_bound(void *priv)
{
	const struct device *dev = priv;
	struct i2c_ipc_data *data = dev->data;

	k_sem_give(&data->bound_sem);
}

static void ept_received(const void *buf, size_t len, void *priv)
{
	const struct device *dev = priv;
	struct i2c_ipc_data *data = dev->data;

	if (len > BUF_SIZE) {
		LOG_WRN("Response too large (%zu > %d), truncating", len, BUF_SIZE);
		len = BUF_SIZE;
	}

	memcpy(data->rx_buf, buf, len);
	data->rx_len = len;
	k_sem_give(&data->resp_sem);
}

static int i2c_ipc_configure(const struct device *dev, uint32_t dev_config)
{
	struct i2c_ipc_data *data = dev->data;
	struct i2c_ipc_configure_req req;
	int ret;

	req.cmd = I2C_IPC_CMD_CONFIGURE;
	req.dev_config = sys_cpu_to_le32(dev_config);

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = ipc_service_send(&data->ept, &req, sizeof(req));
	if (ret < 0) {
		goto out;
	}

	ret = k_sem_take(&data->resp_sem, K_FOREVER);
	if (ret < 0) {
		goto out;
	}

	if (data->rx_len < sizeof(struct i2c_ipc_configure_rsp)) {
		ret = -EIO;
		goto out;
	}

	const struct i2c_ipc_configure_rsp *rsp =
		(const struct i2c_ipc_configure_rsp *)data->rx_buf;

	ret = (int)sys_le32_to_cpu(rsp->ret);

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int i2c_ipc_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct i2c_ipc_data *data = dev->data;
	struct i2c_ipc_get_config_req req;
	int ret;

	req.cmd = I2C_IPC_CMD_GET_CONFIG;

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = ipc_service_send(&data->ept, &req, sizeof(req));
	if (ret < 0) {
		goto out;
	}

	ret = k_sem_take(&data->resp_sem, K_FOREVER);
	if (ret < 0) {
		goto out;
	}

	if (data->rx_len < sizeof(struct i2c_ipc_get_config_rsp)) {
		ret = -EIO;
		goto out;
	}

	const struct i2c_ipc_get_config_rsp *rsp =
		(const struct i2c_ipc_get_config_rsp *)data->rx_buf;

	ret = (int)sys_le32_to_cpu(rsp->ret);
	if (ret == 0) {
		*dev_config = sys_le32_to_cpu(rsp->dev_config);
	}

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int i2c_ipc_transfer(const struct device *dev, struct i2c_msg *msgs,
			     uint8_t num_msgs, uint16_t addr)
{
	struct i2c_ipc_data *data = dev->data;
	size_t offset;
	int ret;

	/* Calculate required buffer size */
	size_t needed = sizeof(struct i2c_ipc_transfer_hdr) +
			(size_t)num_msgs * sizeof(struct i2c_ipc_msg_desc);
	for (uint8_t i = 0; i < num_msgs; i++) {
		if (!(msgs[i].flags & I2C_MSG_READ)) {
			needed += msgs[i].len;
		}
	}

	if (needed > BUF_SIZE) {
		LOG_ERR("Transfer request too large (%zu > %d)", needed, BUF_SIZE);
		return -ENOMEM;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Build transfer header */
	struct i2c_ipc_transfer_hdr *hdr = (struct i2c_ipc_transfer_hdr *)data->tx_buf;

	hdr->cmd = I2C_IPC_CMD_TRANSFER;
	hdr->addr = sys_cpu_to_le16(addr);
	hdr->num_msgs = num_msgs;

	/* Build per-message descriptors */
	struct i2c_ipc_msg_desc *descs =
		(struct i2c_ipc_msg_desc *)(data->tx_buf + sizeof(*hdr));

	for (uint8_t i = 0; i < num_msgs; i++) {
		descs[i].flags = msgs[i].flags;
		descs[i].len = sys_cpu_to_le16((uint16_t)msgs[i].len);
	}

	/* Append write data */
	offset = sizeof(*hdr) + (size_t)num_msgs * sizeof(struct i2c_ipc_msg_desc);
	for (uint8_t i = 0; i < num_msgs; i++) {
		if (!(msgs[i].flags & I2C_MSG_READ)) {
			memcpy(&data->tx_buf[offset], msgs[i].buf, msgs[i].len);
			offset += msgs[i].len;
		}
	}

	ret = ipc_service_send(&data->ept, data->tx_buf, offset);
	if (ret < 0) {
		goto out;
	}

	ret = k_sem_take(&data->resp_sem, K_FOREVER);
	if (ret < 0) {
		goto out;
	}

	if (data->rx_len < sizeof(struct i2c_ipc_transfer_rsp)) {
		ret = -EIO;
		goto out;
	}

	const struct i2c_ipc_transfer_rsp *rsp =
		(const struct i2c_ipc_transfer_rsp *)data->rx_buf;

	ret = (int)sys_le32_to_cpu(rsp->ret);
	if (ret != 0) {
		goto out;
	}

	/* Copy read data back into message buffers */
	offset = sizeof(struct i2c_ipc_transfer_rsp);
	for (uint8_t i = 0; i < num_msgs; i++) {
		if (msgs[i].flags & I2C_MSG_READ) {
			if (offset + msgs[i].len > data->rx_len) {
				ret = -EIO;
				goto out;
			}
			memcpy(msgs[i].buf, &data->rx_buf[offset], msgs[i].len);
			offset += msgs[i].len;
		}
	}

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static DEVICE_API(i2c, i2c_ipc_api) = {
	.configure = i2c_ipc_configure,
	.get_config = i2c_ipc_get_config,
	.transfer = i2c_ipc_transfer,
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
};

static int i2c_ipc_init(const struct device *dev)
{
	struct i2c_ipc_data *data = dev->data;
	const struct i2c_ipc_config *cfg = dev->config;
	struct ipc_ept_cfg ept_cfg = {
		.name = cfg->ept_name,
		.cb = {
			.bound = ept_bound,
			.received = ept_received,
		},
		.priv = (void *)dev,
	};
	int ret;

	ret = ipc_service_open_instance(cfg->ipc);
	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("Failed to open IPC instance: %d", ret);
		return ret;
	}

	ret = ipc_service_register_endpoint(cfg->ipc, &data->ept, &ept_cfg);
	if (ret < 0) {
		LOG_ERR("Failed to register IPC endpoint: %d", ret);
		return ret;
	}

	return 0;
}

#define I2C_IPC_DEFINE(inst)                                                                       \
	static struct i2c_ipc_data i2c_ipc_data_##inst = {                                         \
		.bound_sem = Z_SEM_INITIALIZER(i2c_ipc_data_##inst.bound_sem, 0, 1),               \
		.resp_sem = Z_SEM_INITIALIZER(i2c_ipc_data_##inst.resp_sem, 0, 1),                 \
		.lock = Z_MUTEX_INITIALIZER(i2c_ipc_data_##inst.lock),                             \
	};                                                                                         \
                                                                                                   \
	static const struct i2c_ipc_config i2c_ipc_config_##inst = {                               \
		.ipc = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                        \
		.ept_name = DT_INST_PROP(inst, ipc_endpoint_name),                                 \
	};                                                                                         \
                                                                                                   \
	I2C_DEVICE_DT_INST_DEFINE(inst, i2c_ipc_init, NULL, &i2c_ipc_data_##inst,                 \
				  &i2c_ipc_config_##inst, POST_KERNEL,                             \
				  CONFIG_I2C_IPC_INIT_PRIORITY, &i2c_ipc_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_IPC_DEFINE)
