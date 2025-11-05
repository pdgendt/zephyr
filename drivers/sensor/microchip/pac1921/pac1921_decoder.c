/*
 * Copyright (c) 2025 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util_macro.h>

#include "pac1921.h"

static int pac1921_decoder_get_frame_count(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
					   uint16_t *frame_count)
{
	const struct pac1921_rtio_data *rdata = (const struct pac1921_rtio_data *)buffer;

	if (chan_spec.chan_idx != 0) {
		return -EINVAL;
	}

	if (!rdata->header.is_fifo) {
		switch (chan_spec.chan_type) {
		case SENSOR_CHAN_CURRENT:
		case SENSOR_CHAN_POWER:
		case SENSOR_CHAN_VOLTAGE:
			*frame_count = 1;
			return 0;
		default:
			*frame_count = 0;
			return -EINVAL;
		}
	}

	return 0;
}

static int pac1921_decoder_get_size_info(struct sensor_chan_spec chan_spec, size_t *base_size,
					 size_t *frame_size)
{
	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_CURRENT:
	case SENSOR_CHAN_POWER:
	case SENSOR_CHAN_VOLTAGE:
		*base_size = sizeof(struct sensor_q31_data);
		*frame_size = sizeof(struct sensor_q31_sample_data);
		return 0;
	default:
		return -ENOTSUP;
	}
}
static int pac1921_decoder_decode_one_shot(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
					   uint32_t *fit, uint16_t max_count, void *data_out)
{
	const struct pac1921_rtio_data *rdata = (const struct pac1921_rtio_data *)buffer;
	struct sensor_q31_data *out = data_out;

	if (*fit != 0) {
		return 0;
	}
	if (max_count == 0 || chan_spec.chan_idx != 0) {
		return -EINVAL;
	}

	out->header.base_timestamp_ns = rdata->header.timestamp;
	out->header.reading_count = 1;

	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_CURRENT:
	case SENSOR_CHAN_POWER:
	case SENSOR_CHAN_VOLTAGE:
		out->readings[0].value = rdata->raw << 15;
		break;
	default:
		return -EINVAL;
	}

	return 1;
}

static int pac1921_decoder_decode(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
				  uint32_t *fit, uint16_t max_count, void *data_out)
{
	const struct pac1921_rtio_data *rdata = (const struct pac1921_rtio_data *)buffer;

	if (rdata->header.is_fifo) {
		return -ENOTSUP;
	}

	return pac1921_decoder_decode_one_shot(buffer, chan_spec, fit, max_count, data_out);
}

static bool pac1921_decoder_has_trigger(const uint8_t *buffer, enum sensor_trigger_type trigger)
{
	ARG_UNUSED(buffer);
	ARG_UNUSED(trigger);

	return false;
}

SENSOR_DECODER_API_DT_DEFINE() = {
	.get_frame_count = pac1921_decoder_get_frame_count,
	.get_size_info = pac1921_decoder_get_size_info,
	.decode = pac1921_decoder_decode,
	.has_trigger = pac1921_decoder_has_trigger,
};

int pac1921_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder)
{
	ARG_UNUSED(dev);
	*decoder = &SENSOR_DECODER_NAME();

	return 0;
}
