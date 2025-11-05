/*
 * Copyright (c) 2025 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/sensor_clock.h>
#include <zephyr/rtio/regmap.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pac1921, CONFIG_SENSOR_LOG_LEVEL);

#include "pac1921.h"

static int pac1921_attr_set(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr, const struct sensor_value *val)
{
	if (attr != SENSOR_ATTR_GAIN) {
		return -ENOTSUP;
	}

	switch (chan) {
	case SENSOR_CHAN_CURRENT:
	case SENSOR_CHAN_VOLTAGE:
		break;
	default:
		return -ENOTSUP;
	}

	/* TODO: set gain */

	return 0;
}

static void pac1921_one_shot_complete_cb(struct rtio *ctx, const struct rtio_sqe *sqe, int res,
					 void *arg)
{
	struct rtio_iodev_sqe *iodev_sqe = (struct rtio_iodev_sqe *)sqe->userdata;
	int err = 0;

	ARG_UNUSED(res);

	err = rtio_flush_completion_queue(ctx);

	if (err < 0) {
		rtio_iodev_sqe_err(iodev_sqe, err);
	} else {
		rtio_iodev_sqe_ok(iodev_sqe, 0);
	}
}

static void pac1921_assert_read_int(struct rtio *r, const struct rtio_sqe *sqe, int res, void *arg)
{
	const struct device *dev = (const struct device *)sqe->userdata;
	const struct pac1921_config *config = dev->config;
	uintptr_t val = (uintptr_t)arg;

	ARG_UNUSED(r);
	ARG_UNUSED(res);

	(void)gpio_pin_set_dt(&config->read_int, val > 0 ? 1U : 0U);
}

static struct rtio_sqe *pac1921_prep_read_int(const struct device *dev, bool active)
{
	const struct pac1921_config *config = dev->config;
	struct pac1921_data *data = dev->data;
	struct rtio_sqe *sqe;

	sqe = rtio_sqe_acquire(data->rtio_ctx);
	if (sqe == NULL) {
		return NULL;
	}

	if (config->read_int.port != NULL) {
		rtio_sqe_prep_callback(sqe, pac1921_assert_read_int,
				       (void *)(uintptr_t)(active ? 1U : 0U), (void *)dev);
		sqe->flags |= RTIO_SQE_CHAINED;
	} else {
		data->int_cfg.inten = active ? 1U : 0U;

		const uint8_t msg[2] = {
			PAC1921_REG_INT_CFG,
			data->int_cfg.raw,
		};

		rtio_sqe_prep_tiny_write(sqe, data->iodev, RTIO_PRIO_NORM, msg, sizeof(msg), NULL);
		sqe->iodev_flags = RTIO_IODEV_I2C_STOP;
	}

	return sqe;
}

static inline uint8_t pac1921_chan_type_to_reg(enum sensor_channel chan)
{
	switch (chan) {
	case SENSOR_CHAN_CURRENT:
		return PAC1921_REG_VSENSE;
	case SENSOR_CHAN_VOLTAGE:
		return PAC1921_REG_VBUS;
	case SENSOR_CHAN_POWER:
		return PAC1921_REG_VPOWER;
	default:
		break;
	}

	return UINT8_MAX;
}

static void pac1921_submit_one_shot(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	struct pac1921_data *data = dev->data;
	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;
	struct pac1921_rtio_data *rtio_data;
	struct rtio_sqe *sqe;
	uint8_t reg_addr;
	uint8_t *buf;
	uint32_t buf_len;
	// size_t reads = 0;
	int ret;

	if (cfg->count != 1) {
		rtio_iodev_sqe_err(iodev_sqe, -EINVAL);
		return;
	}

	reg_addr = pac1921_chan_type_to_reg(cfg->channels[0].chan_type);
	if (reg_addr == UINT8_MAX) {
		rtio_iodev_sqe_err(iodev_sqe, -EINVAL);
		return;
	}

	ret = rtio_sqe_rx_buf(iodev_sqe, sizeof(struct pac1921_rtio_data),
			      sizeof(struct pac1921_rtio_data), &buf, &buf_len);
	if (ret < 0) {
		rtio_iodev_sqe_err(iodev_sqe, ret);
		return;
	}

	rtio_data = (struct pac1921_rtio_data *)buf;

	sqe = pac1921_prep_read_int(dev, true);
	if (sqe == NULL) {
		ret = -ENOMEM;
		goto error;
	}

	sqe = rtio_sqe_acquire(data->rtio_ctx);
	if (sqe == NULL) {
		ret = -ENOMEM;
		goto error;
	}
	rtio_sqe_prep_delay(sqe, data->period, NULL);
	sqe->flags |= RTIO_SQE_CHAINED;

	sqe = pac1921_prep_read_int(dev, false);
	if (sqe == NULL) {
		ret = -ENOMEM;
		goto error;
	}

	sqe = i2c_rtio_copy_reg_burst_read(data->rtio_ctx, data->iodev, reg_addr, &rtio_data->raw,
					   sizeof(rtio_data->raw));
	if (sqe == NULL) {
		ret = -ENOMEM;
		goto error;
	}

	rtio_submit(data->rtio_ctx, 0);
	return;

error:
	rtio_iodev_sqe_err(iodev_sqe, ret);
	rtio_sqe_drop_all(data->rtio_ctx);
}

static void pac1921_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;

	if (!cfg->is_streaming) {
		pac1921_submit_one_shot(dev, iodev_sqe);
	} else if (IS_ENABLED(CONFIG_PAC1921_STREAM)) {
		/* TODO */
		rtio_iodev_sqe_err(iodev_sqe, -ENOTSUP);
	} else {
		rtio_iodev_sqe_err(iodev_sqe, -ENOTSUP);
	}
}

static DEVICE_API(sensor, pac1921_api) = {
	.attr_set = pac1921_attr_set,
	.submit = pac1921_submit,
	.get_decoder = pac1921_get_decoder,
};

static int pac1921_init(const struct device *dev)
{
	const struct pac1921_config *config = dev->config;
	struct pac1921_data *data = dev->data;

	if (!i2c_is_ready_dt(&config->bus)) {
		LOG_ERR("I2C is not ready");
		return -ENODEV;
	}

#if PAC1921_ANY_HAS_GPIO
	if (config->read_int.port != NULL) {
		int ret;

		if (!gpio_is_ready_dt(&config->read_int)) {
			LOG_ERR("READ/INT is not ready");
			return -ENODEV;
		}

		/* Start in READ */
		ret = gpio_pin_configure_dt(&config->read_int, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("READ/INT config failed (%d)", ret);
			return ret;
		}
	} else
#endif
	{
		/* No READ/INT GPIO so use the pin override READ/INT_OVR */
		data->int_cfg.riov = 1U;
	}

	return 0;
}

#if PAC1921_ANY_HAS_GPIO
#define PAC1921_GPIOS(inst) .read_int = GPIO_DT_SPEC_INST_GET_OR(inst, read_int_gpios, {}),
#else
#define PAC1921_GPIOS(inst)
#endif

#define PAC1921_I2C_RTIO_DEFINE(inst)                                                              \
	I2C_DT_IODEV_DEFINE(pac1921_iodev_##inst, DT_DRV_INST(inst));                              \
	RTIO_DEFINE(pac1921_iodev_ctx_##inst, 8, 8);

#define PAC1921_INIT(inst)                                                                         \
	PAC1921_I2C_RTIO_DEFINE(inst);                                                             \
	static const struct pac1921_config pac1921_##inst##_config = {                             \
		.bus = I2C_DT_SPEC_INST_GET(inst),                                                 \
		PAC1921_GPIOS(inst)                                                                \
	};                                                                                         \
	static struct pac1921_data pac1921_##inst##_data = {                                       \
		.iodev = &pac1921_iodev_##inst,                                                    \
		.rtio_ctx = &pac1921_iodev_ctx_##inst,                                             \
	};                                                                                         \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, pac1921_init, NULL, &pac1921_##inst##_data,             \
				     &pac1921_##inst##_config, POST_KERNEL,                        \
				     CONFIG_SENSOR_INIT_PRIORITY, &pac1921_api);

DT_INST_FOREACH_STATUS_OKAY(PAC1921_INIT)
