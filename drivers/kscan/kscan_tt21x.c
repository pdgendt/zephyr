/*
 * Copyright (c) 2021 Basalte
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT paradetech_tt21x

#include <drivers/kscan.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(tt21x, CONFIG_KSCAN_LOG_LEVEL);

/** GPIO DT information. */
struct gpio_dt_info {
	/** Port. */
	const char *port;
	/** Pin. */
	gpio_pin_t pin;
	/** Flags. */
	gpio_dt_flags_t flags;
};

/** TT21X configuration (DT). */
struct tt21x_config {
	/** I2C controller device name. */
	char *i2c_name;
	/** I2C chip address. */
	uint8_t i2c_address;
#ifdef CONFIG_KSCAN_TT21X_INTERRUPT
	/** Interrupt GPIO information. */
	struct gpio_dt_info int_gpio;
#endif
	/** Reset GPIO information. */
	struct gpio_dt_info rst_gpio;
};

/** TT21X data. */
struct tt21x_data {
	/** Device pointer. */
	const struct device *dev;
	/** I2C controller device. */
	const struct device *i2c;
	bool enabled;
	/** KSCAN Callback. */
	kscan_callback_t callback;
	/** Work queue (for deferred read). */
	struct k_work work;
#ifdef CONFIG_KSCAN_TT21X_INTERRUPT
	/** Interrupt GPIO controller. */
	const struct device *int_gpio;
	/** Interrupt GPIO callback. */
	struct gpio_callback int_gpio_cb;
#else
	/** Timer (polling mode). */
	struct k_timer timer;
#endif
	/** Reset GPIO controller. */
	const struct device *rst_gpio;
};

#define TT21X_TOUCH_DATA_LEN     (50)
#define TT21X_TOUCH_MIN_PRESSURE (14)

// 4.6.1 Wake up Event Report
typedef struct tt21x_wakeup_event {
	uint8_t length_LSB;
	uint8_t length_MSB;
	uint8_t report_id;
	uint8_t event_code;
	uint8_t data_point_0_X_LSB;
	uint8_t data_point_0_X_MSB;
	uint8_t data_point_0_Y_LSB;
	uint8_t data_point_0_Y_MSB;
	uint8_t data_point_1_X_LSB;
	uint8_t data_point_1_X_MSB;
	uint8_t data_point_1_Y_LSB;
	uint8_t data_point_1_Y_MSB;
} tt21x_wakeup_event_t;

typedef enum tt21x_wakeup_event_code {
	TT21X_WAKEUP_EVENT_CODE_ONE_FINGER_DOUBLE_TAP_GESTURE           = 0x01,
	TT21X_WAKEUP_EVENT_CODE_RESERVED_2                              = 0x02,
	TT21X_WAKEUP_EVENT_CODE_ONE_FINGER_SINGLE_TAP_GESTURE           = 0x03,
	TT21X_WAKEUP_EVENT_CODE_RESERVED_5                              = 0x05,
	TT21X_WAKEUP_EVENT_CODE_SINGLE_FINGER_SLIDE_GESTURE_DECREASE_TX = 0x05,
	TT21X_WAKEUP_EVENT_CODE_SINGLE_FINGER_SLIDE_GESTURE_INCREASE_TX = 0x06,
	TT21X_WAKEUP_EVENT_CODE_SINGLE_FINGER_SLIDE_GESTURE_DECREASE_RX = 0x07,
	TT21X_WAKEUP_EVENT_CODE_SINGLE_FINGER_SLIDE_GESTURE_INCREASE_RX = 0x08,
	TT21X_WAKEUP_EVENT_CODE_EASY_WAKE_C_CHAR                        = 0x63,
	TT21X_WAKEUP_EVENT_CODE_EASY_WAKE_E_CHAR                        = 0x65,
	TT21X_WAKEUP_EVENT_CODE_EASY_WAKE_M_CHAR                        = 0x6D,
	TT21X_WAKEUP_EVENT_CODE_EASY_WAKE_W_CHAR                        = 0x77,
} tt21x_wakeup_event_code_t;

// 8.1 Touch reports
typedef struct tt21x_input_header {
	uint16_t length;
	uint8_t report_id;
	uint8_t timestamp_lsb;
	uint8_t timestamp_msb;
	uint8_t number_of_records : 5;
	uint8_t LO : 1;
	uint8_t reserved1 : 2;
	uint8_t noise_effects : 3;
	uint8_t reserved2 : 3;
	uint8_t report_counter : 2;
} tt21x_input_header_t;

typedef struct tt21x_input_touch_record {
	uint8_t touch_type : 3;
	uint8_t reserved : 5;
	uint8_t touch_id : 5;
	uint8_t event_id : 2;
	uint8_t tip : 1;
	uint16_t x;
	uint16_t y;
	uint8_t pressure;
	uint8_t axis_length_major;
	uint8_t axis_length_minor;
	uint8_t orientation;
} tt21x_input_touch_record_t;

typedef enum tt21x_report_id {
	TT21X_REPORT_ID_TOUCH_REPORT    = 0x01,
	TT21X_REPORT_ID_PING            = 0x1F,
} tt21x_report_id_t;

typedef struct tt21x_input_packet {
	tt21x_input_header_t header;
	tt21x_input_touch_record_t record;
} tt21x_input_packet_t;

typedef union tt21x_message {
	uint8_t buffer[TT21X_TOUCH_DATA_LEN];
	tt21x_input_packet_t packet;
} tt21x_message_t;

static int tt21x_process(const struct device *dev)
{
	int ret;
	uint16_t packet_size;
	uint16_t touch_x, touch_y;
	bool pressed;
	tt21x_message_t msg;

	const struct tt21x_config *config = dev->config;
	struct tt21x_data *data = dev->data;

	/* first we need to read the data length */
	ret = i2c_read(data->i2c, msg.buffer, 2, config->i2c_address);

	if (ret != 0) {
		LOG_ERR("Unable to read tt21x packet size (%d)", ret);
		return ret;
	}

	packet_size = msg.packet.header.length;

	if (packet_size == 0) {
		LOG_WRN("Empty tt21x packet");

		return -ENOMSG;
	}

	/* second is to read the actual data */
	ret = i2c_read(data->i2c, msg.buffer, packet_size, config->i2c_address);
	if (ret != 0) {
		LOG_ERR("Unable to read tt21x packet (%d)", ret);
		return ret;
	}

	if (msg.packet.header.report_id == TT21X_REPORT_ID_TOUCH_REPORT) {
		if (msg.packet.record.pressure >= TT21X_TOUCH_MIN_PRESSURE
		    || msg.packet.record.tip == 0) {
			touch_x = msg.packet.record.x;
			touch_y = msg.packet.record.y;
			pressed = msg.packet.record.tip > 0;

			if (data->enabled && data->callback) {
				data->callback(dev, touch_x, touch_y, pressed);
			}
		}
	}

	return 0;
}

static void tt21x_work_handler(struct k_work *work)
{
	struct tt21x_data *data = CONTAINER_OF(work, struct tt21x_data, work);

	tt21x_process(data->dev);
}

#ifdef CONFIG_KSCAN_TT21X_INTERRUPT
static void tt21x_isr_handler(const struct device *dev,
			      struct gpio_callback *cb, uint32_t pins)
{
	struct tt21x_data *data = CONTAINER_OF(cb, struct tt21x_data, int_gpio_cb);

	k_work_submit(&data->work);
}
#else
static void tt21x_timer_handler(struct k_timer *timer)
{
	struct tt21x_data *data = CONTAINER_OF(timer, struct tt21x_data, timer);

	k_work_submit(&data->work);
}
#endif

static int tt21x_configure(const struct device *dev,
			   kscan_callback_t callback)
{
	const struct tt21x_config *config = dev->config;
	struct tt21x_data *data = dev->data;

	if (!callback) {
		LOG_ERR("Invalid callback (NULL)");
		return -EINVAL;
	}

	data->callback = callback;

	// reset the controller
	gpio_pin_set(data->rst_gpio, config->rst_gpio.pin, 1);
	k_busy_wait(10);
	gpio_pin_set(data->rst_gpio, config->rst_gpio.pin, 0);

	return 0;
}

static int tt21x_enable_callback(const struct device *dev)
{
	struct tt21x_data *data = dev->data;

	data->enabled = true;

#ifndef CONFIG_KSCAN_TT21X_INTERRUPT
	k_timer_start(&data->timer, K_MSEC(CONFIG_KSCAN_TT21X_PERIOD),
		      K_MSEC(CONFIG_KSCAN_TT21X_PERIOD));
#endif

	return 0;
}

static int tt21x_disable_callback(const struct device *dev)
{
	struct tt21x_data *data = dev->data;

#ifndef CONFIG_KSCAN_TT21X_INTERRUPT
	k_timer_stop(&data->timer);
#endif

	data->enabled = false;

	return 0;
}

static int tt21x_init(const struct device *dev)
{
	const struct tt21x_config *config = dev->config;
	struct tt21x_data *data = dev->data;
	int r;

	data->i2c = device_get_binding(config->i2c_name);
	if (!data->i2c) {
		LOG_ERR("Could not find I2C controller");
		return -ENODEV;
	}

	data->dev = dev;
	data->enabled = false;

	k_work_init(&data->work, tt21x_work_handler);

	data->rst_gpio = device_get_binding(config->rst_gpio.port);
	if (!data->rst_gpio) {
		LOG_ERR("Could not find reset GPIO controller");
		return -ENODEV;
	}
	r = gpio_pin_configure(data->rst_gpio, config->rst_gpio.pin,
			       config->rst_gpio.flags | GPIO_OUTPUT);
	if (r < 0) {
		LOG_ERR("Could not configure reset GPIO pin");
		return r;
	}

	// keep the controller in reset
	gpio_pin_set(data->rst_gpio, config->rst_gpio.pin, 1);

#ifdef CONFIG_KSCAN_TT21X_INTERRUPT
	data->int_gpio = device_get_binding(config->int_gpio.port);
	if (!data->int_gpio) {
		LOG_ERR("Could not find interrupt GPIO controller");
		return -ENODEV;
	}

	r = gpio_pin_configure(data->int_gpio, config->int_gpio.pin,
			       config->int_gpio.flags | GPIO_INPUT);
	if (r < 0) {
		LOG_ERR("Could not configure interrupt GPIO pin");
		return r;
	}

	r = gpio_pin_interrupt_configure(data->int_gpio, config->int_gpio.pin,
					 GPIO_INT_EDGE_TO_ACTIVE);
	if (r < 0) {
		LOG_ERR("Could not configure interrupt GPIO interrupt.");
		return r;
	}

	gpio_init_callback(&data->int_gpio_cb, tt21x_isr_handler,
			   BIT(config->int_gpio.pin));

	gpio_add_callback(data->int_gpio, &data->int_gpio_cb);
#else
	k_timer_init(&data->timer, tt21x_timer_handler, NULL);
#endif

	return 0;
}

static const struct kscan_driver_api tt21x_driver_api = {
	.config = tt21x_configure,
	.enable_callback = tt21x_enable_callback,
	.disable_callback = tt21x_disable_callback,
};

#define DT_INST_GPIO(index, gpio_pha)		     \
	{					     \
		DT_INST_GPIO_LABEL(index, gpio_pha), \
		DT_INST_GPIO_PIN(index, gpio_pha),   \
		DT_INST_GPIO_FLAGS(index, gpio_pha), \
	}

#ifdef CONFIG_KSCAN_TT21X_INTERRUPT
#define TT21X_DEFINE_CONFIG(index)				  \
	static const struct tt21x_config tt21x_config_##index = { \
		.i2c_name = DT_INST_BUS_LABEL(index),		  \
		.i2c_address = DT_INST_REG_ADDR(index),		  \
		.int_gpio = DT_INST_GPIO(index, int_gpios),	  \
		.rst_gpio = DT_INST_GPIO(index, reset_gpios)	  \
	}
#else
#define TT21X_DEFINE_CONFIG(index)				  \
	static const struct tt21x_config tt21x_config_##index = { \
		.i2c_name = DT_INST_BUS_LABEL(index),		  \
		.i2c_address = DT_INST_REG_ADDR(index),		  \
		.rst_gpio = DT_INST_GPIO(index, reset_gpios)	  \
	}
#endif

#define TT21X_INIT(index)						  \
	TT21X_DEFINE_CONFIG(index);					  \
	static struct tt21x_data tt21x_data_##index;			  \
	DEVICE_DT_INST_DEFINE(index, tt21x_init, device_pm_control_nop,	  \
			      &tt21x_data_##index, &tt21x_config_##index, \
			      POST_KERNEL, CONFIG_KSCAN_INIT_PRIORITY,	  \
			      &tt21x_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TT21X_INIT)
