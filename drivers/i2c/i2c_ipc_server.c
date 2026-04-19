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

#define DT_DRV_COMPAT zephyr_i2c_ipc_server

LOG_MODULE_REGISTER(i2c_ipc_server, CONFIG_I2C_LOG_LEVEL);

#define BUF_SIZE CONFIG_I2C_IPC_SERVER_BUFFER_SIZE

struct i2c_ipc_server_data {
	struct ipc_ept ept;
	struct k_sem bound_sem;
	struct k_sem req_sem;
	uint8_t req_buf[BUF_SIZE];
	size_t req_len;
	uint8_t rsp_buf[BUF_SIZE];
	struct k_thread thread;
	K_KERNEL_STACK_MEMBER(stack, CONFIG_I2C_IPC_SERVER_STACK_SIZE);
};

struct i2c_ipc_server_config {
	const struct device *ipc;
	const struct device *controller;
	const char *ept_name;
};

static void ept_bound(void *priv)
{
	const struct device *dev = priv;
	struct i2c_ipc_server_data *data = dev->data;

	k_sem_give(&data->bound_sem);
}

static void ept_received(const void *buf, size_t len, void *priv)
{
	const struct device *dev = priv;
	struct i2c_ipc_server_data *data = dev->data;

	if (len > BUF_SIZE) {
		LOG_WRN("Request too large (%zu > %d), dropping", len, BUF_SIZE);
		return;
	}

	memcpy(data->req_buf, buf, len);
	data->req_len = len;
	k_sem_give(&data->req_sem);
}

static void handle_configure(const struct device *dev)
{
	struct i2c_ipc_server_data *data = dev->data;
	const struct i2c_ipc_server_config *cfg = dev->config;

	if (data->req_len < sizeof(struct i2c_ipc_configure_req)) {
		LOG_WRN("Configure request too short");
		return;
	}

	const struct i2c_ipc_configure_req *req =
		(const struct i2c_ipc_configure_req *)data->req_buf;

	uint32_t dev_config = sys_le32_to_cpu(req->dev_config);
	int ret = i2c_configure(cfg->controller, dev_config);

	struct i2c_ipc_configure_rsp rsp = {
		.cmd = I2C_IPC_CMD_CONFIGURE,
		.ret = sys_cpu_to_le32(ret),
	};

	ipc_service_send(&data->ept, &rsp, sizeof(rsp));
}

static void handle_get_config(const struct device *dev)
{
	struct i2c_ipc_server_data *data = dev->data;
	const struct i2c_ipc_server_config *cfg = dev->config;
	uint32_t dev_config = 0;

	int ret = i2c_get_config(cfg->controller, &dev_config);

	struct i2c_ipc_get_config_rsp rsp = {
		.cmd = I2C_IPC_CMD_GET_CONFIG,
		.ret = sys_cpu_to_le32(ret),
		.dev_config = sys_cpu_to_le32(dev_config),
	};

	ipc_service_send(&data->ept, &rsp, sizeof(rsp));
}

static void handle_transfer(const struct device *dev)
{
	struct i2c_ipc_server_data *data = dev->data;
	const struct i2c_ipc_server_config *cfg = dev->config;
	size_t offset;
	int ret;

	if (data->req_len < sizeof(struct i2c_ipc_transfer_hdr)) {
		LOG_WRN("Transfer request too short");
		return;
	}

	const struct i2c_ipc_transfer_hdr *hdr =
		(const struct i2c_ipc_transfer_hdr *)data->req_buf;

	uint16_t addr = sys_le16_to_cpu(hdr->addr);
	uint8_t num_msgs = hdr->num_msgs;

	size_t descs_size = (size_t)num_msgs * sizeof(struct i2c_ipc_msg_desc);

	if (data->req_len < sizeof(*hdr) + descs_size) {
		LOG_WRN("Transfer request missing descriptors");
		return;
	}

	const struct i2c_ipc_msg_desc *descs =
		(const struct i2c_ipc_msg_desc *)(data->req_buf + sizeof(*hdr));

	/* Build i2c_msg array on the stack */
	struct i2c_msg msgs[num_msgs];
	size_t rsp_offset = sizeof(struct i2c_ipc_transfer_rsp);

	/* Point to write data in request buffer */
	offset = sizeof(*hdr) + descs_size;

	for (uint8_t i = 0; i < num_msgs; i++) {
		msgs[i].flags = descs[i].flags;
		msgs[i].len = sys_le16_to_cpu(descs[i].len);

		if (msgs[i].flags & I2C_MSG_READ) {
			/* Point read buffers into response buffer */
			if (rsp_offset + msgs[i].len > BUF_SIZE) {
				LOG_ERR("Read data would exceed buffer");
				ret = -ENOMEM;
				goto send_error;
			}
			msgs[i].buf = &data->rsp_buf[rsp_offset];
			rsp_offset += msgs[i].len;
		} else {
			/* Point write buffers into request buffer */
			if (offset + msgs[i].len > data->req_len) {
				LOG_ERR("Write data exceeds request length");
				ret = -EINVAL;
				goto send_error;
			}
			msgs[i].buf = &data->req_buf[offset];
			offset += msgs[i].len;
		}
	}

	ret = i2c_transfer(cfg->controller, msgs, num_msgs, addr);

send_error:;
	struct i2c_ipc_transfer_rsp *rsp = (struct i2c_ipc_transfer_rsp *)data->rsp_buf;

	rsp->cmd = I2C_IPC_CMD_TRANSFER;
	rsp->ret = sys_cpu_to_le32(ret);

	/* rsp_offset already accounts for read data placed by i2c_transfer */
	ipc_service_send(&data->ept, data->rsp_buf, ret == 0 ? rsp_offset : sizeof(*rsp));
}

static void server_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct i2c_ipc_server_data *data = dev->data;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		k_sem_take(&data->req_sem, K_FOREVER);

		if (data->req_len < 1) {
			continue;
		}

		uint8_t cmd = data->req_buf[0];

		switch (cmd) {
		case I2C_IPC_CMD_CONFIGURE:
			handle_configure(dev);
			break;
		case I2C_IPC_CMD_GET_CONFIG:
			handle_get_config(dev);
			break;
		case I2C_IPC_CMD_TRANSFER:
			handle_transfer(dev);
			break;
		default:
			LOG_WRN("Unknown command: 0x%02x", cmd);
			break;
		}
	}
}

static int i2c_ipc_server_init(const struct device *dev)
{
	struct i2c_ipc_server_data *data = dev->data;
	const struct i2c_ipc_server_config *cfg = dev->config;
	struct ipc_ept_cfg ept_cfg = {
		.name = cfg->ept_name,
		.cb = {
			.bound = ept_bound,
			.received = ept_received,
		},
		.priv = (void *)dev,
	};
	int ret;

	if (!device_is_ready(cfg->controller)) {
		LOG_ERR("I2C controller not ready");
		return -ENODEV;
	}

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

	k_thread_create(&data->thread, data->stack,
			K_KERNEL_STACK_SIZEOF(data->stack),
			server_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(8), 0, K_NO_WAIT);
	k_thread_name_set(&data->thread, dev->name);

	return 0;
}

#define I2C_IPC_SERVER_DEFINE(inst)                                                                \
	static struct i2c_ipc_server_data i2c_ipc_server_data_##inst = {                           \
		.bound_sem = Z_SEM_INITIALIZER(i2c_ipc_server_data_##inst.bound_sem, 0, 1),        \
		.req_sem = Z_SEM_INITIALIZER(i2c_ipc_server_data_##inst.req_sem, 0, 1),            \
	};                                                                                         \
                                                                                                   \
	static const struct i2c_ipc_server_config i2c_ipc_server_config_##inst = {                 \
		.ipc = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                        \
		.controller = DEVICE_DT_GET(DT_INST_PROP(inst, controller)),                       \
		.ept_name = DT_INST_PROP(inst, ipc_endpoint_name),                                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, i2c_ipc_server_init, NULL, &i2c_ipc_server_data_##inst,       \
			      &i2c_ipc_server_config_##inst, POST_KERNEL,                          \
			      CONFIG_I2C_IPC_SERVER_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(I2C_IPC_SERVER_DEFINE)
