/*
 * NXP PN5180 NFC Driver
 *
 * Copyright (c) 2023 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_pn5180

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(nfc_pn5180, CONFIG_NFC_LOG_LEVEL);

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/nfc.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>

#include "nfc_pn5180.h"

#define PN5180_MIN_FREQUENCY 53000ULL
#define PN5180_MAX_FREQUENCY 13560000ULL
#define PN5180_BUSY_TIMEOUT  K_MSEC(2000)

struct nfc_pn5180_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec gpio_busy;
	struct gpio_dt_spec gpio_irq;
	struct gpio_dt_spec gpio_reset;
};

struct nfc_pn5180_data {
	const struct device *self;
	struct k_mutex api_lock;

	struct k_sem busy_sem;
	struct gpio_callback busy_cb;
	struct k_sem irq_sem;
	struct gpio_callback irq_cb;
	uint32_t irq_mask;
	uint32_t irq_status;

	uint8_t testbus_enabled;
	uint16_t fw_version;

	uint32_t timeout_us;
};

static inline size_t count_bits(uint32_t val)
{
	/* Implements Brian Kernighan’s Algorithm to count bits. */
	size_t cnt = 0U;

	while (val != 0U) {
		val = val & (val - 1U);
		cnt++;
	}

	return cnt;
}

static int nfc_pn5180_exchange(const struct device *dev, const uint8_t *cmd_buf, uint16_t cmd_len,
			       const uint8_t *tx_buf, uint16_t tx_len, uint8_t *rx_buf,
			       uint16_t rx_len)
{
	const struct nfc_pn5180_config *cfg = dev->config;
	struct nfc_pn5180_data *data = dev->data;
	const struct spi_buf spi_tx_buf[2] = {
		{
			.buf = (void *)cmd_buf,
			.len = cmd_len,
		},
		{
			.buf = (void *)tx_buf,
			.len = tx_len,
		},
	};
	const struct spi_buf_set tx = {
		.buffers = spi_tx_buf,
		.count = (tx_buf != NULL) ? 2U : 1U,
	};
	const struct spi_buf spi_rx_buf = {
		.buf = (void *)rx_buf,
		.len = rx_len,
	};
	const struct spi_buf_set rx = {
		.buffers = &spi_rx_buf,
		.count = 1U,
	};
	int ret;

	if (cmd_buf == NULL || cmd_len == 0U) {
		return -EINVAL;
	}

	ret = k_sem_take(&data->busy_sem, PN5180_BUSY_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("TX timeout (%d)", ret);
		return ret;
	}

	LOG_HEXDUMP_DBG(cmd_buf, cmd_len, "CMD");
	if (tx_len > 0U) {
		LOG_HEXDUMP_DBG(tx_buf, tx_len, "TX");
	}

	ret = spi_write_dt(&cfg->bus, &tx);
	if (ret < 0) {
		LOG_ERR("TX error (%d)", ret);
		return ret;
	}

	if (rx_buf != NULL) {
		ret = k_sem_take(&data->busy_sem, PN5180_BUSY_TIMEOUT);
		if (ret < 0) {
			LOG_ERR("RX timeout (%d)", ret);
			return ret;
		}

		ret = spi_read_dt(&cfg->bus, &rx);
		if (ret < 0) {
			LOG_ERR("RX error (%d)", ret);
			return ret;
		}

		LOG_HEXDUMP_DBG(rx_buf, rx_len, "RX");
	}

	return 0;
}

enum pn5180_register_action {
	PN5180_REG_ACTION_WRITE = PN5180_CMD_WRITE_REGISTER,
	PN5180_REG_ACTION_SET = PN5180_CMD_WRITE_REGISTER_OR_MASK,
	PN5180_REG_ACTION_CLEAR = PN5180_CMD_WRITE_REGISTER_AND_MASK,
};

static inline int nfc_pn5180_write_register(const struct device *dev, uint8_t address,
					    uint32_t value, enum pn5180_register_action action)
{
	uint8_t cmd_buf[2] = {
		(uint8_t)action,
		address,
	};
	uint8_t data_buf[4];

	if (action == PN5180_REG_ACTION_CLEAR) {
		value = ~value;
	}

	sys_put_le32(value, data_buf);

	return nfc_pn5180_exchange(dev, cmd_buf, sizeof(cmd_buf), data_buf, sizeof(data_buf), NULL,
				   0U);
}

static inline int nfc_pn5180_read_register(const struct device *dev, uint8_t address,
					   uint32_t *value)
{
	const uint8_t cmd_buf[2] = {
		PN5180_CMD_READ_REGISTER,
		address,
	};
	uint8_t data_buf[4];
	int ret;

	ret = nfc_pn5180_exchange(dev, cmd_buf, sizeof(cmd_buf), NULL, 0U, data_buf,
				  sizeof(data_buf));
	if (ret < 0) {
		return ret;
	}

	*value = sys_get_le32(data_buf);

	return 0;
}

static inline int nfc_pn5180_read_eeprom(const struct device *dev, uint8_t address, uint8_t *data,
					 uint8_t data_len)
{
	const uint8_t cmd_buf[3] = {
		PN5180_CMD_READ_EEPROM,
		address,
		data_len,
	};

	return nfc_pn5180_exchange(dev, cmd_buf, sizeof(cmd_buf), NULL, 0U, data, data_len);
}

static inline int nfc_pn5180_write_eeprom(const struct device *dev, uint8_t address,
					  const uint8_t *data, uint8_t data_len)
{
	const uint8_t cmd_buf[2] = {
		PN5180_CMD_WRITE_EEPROM,
		address,
	};

	return nfc_pn5180_exchange(dev, cmd_buf, sizeof(cmd_buf), data, data_len, NULL, 0);
}

static inline int nfc_pn5180_switch_mode_normal(const struct device *dev)
{
	uint8_t cmd_buf[2] = {
		PN5180_CMD_SWITCH_MODE,
		PN5180_MODE_NORMAL,
	};

	return nfc_pn5180_exchange(dev, cmd_buf, sizeof(cmd_buf), NULL, 0U, NULL, 0U);
}

static inline int nfc_pn5180_switch_mode_standby(const struct device *dev, uint8_t wakeup_ctrl,
						 uint16_t wakeup_counter_ms)
{
	uint8_t cmd_buf[5] = {
		PN5180_CMD_SWITCH_MODE,
		PN5180_MODE_STANDBY,
		wakeup_ctrl,
	};

	sys_put_le16(wakeup_counter_ms, &cmd_buf[3]);

	return nfc_pn5180_exchange(dev, cmd_buf, sizeof(cmd_buf), NULL, 0U, NULL, 0U);
}

static inline int nfc_pn5180_switch_mode_lpcd(const struct device *dev, uint16_t wakeup_counter_ms)
{
	uint8_t cmd_buf[4] = {
		PN5180_CMD_SWITCH_MODE,
		PN5180_MODE_LPCD,
	};

	sys_put_le16(wakeup_counter_ms, &cmd_buf[2]);

	return nfc_pn5180_exchange(dev, cmd_buf, sizeof(cmd_buf), NULL, 0U, NULL, 0U);
}

static inline int nfc_pn5180_switch_mode_autocoll(const struct device *dev, uint8_t rf_tech,
						  uint8_t autocoll_mode)
{
	const uint8_t cmd_buf[4] = {
		PN5180_CMD_SWITCH_MODE,
		PN5180_MODE_AUTOCOLL,
		rf_tech & 0x0FU,
		autocoll_mode & 0x03U,
	};

	return nfc_pn5180_exchange(dev, cmd_buf, sizeof(cmd_buf), NULL, 0U, NULL, 0U);
}

static inline int nfc_pn5180_load_rf_config(const struct device *dev, uint8_t tx_config,
					    uint8_t rx_config)
{
	const uint8_t cmd_buf[3] = {
		PN5180_CMD_LOAD_RF_CONFIG,
		tx_config,
		rx_config,
	};

	return nfc_pn5180_exchange(dev, cmd_buf, sizeof(cmd_buf), NULL, 0U, NULL, 0U);
}

static inline int nfc_pn5180_rf_on(const struct device *dev, uint8_t flags)
{
	const uint8_t cmd_buf[2] = {
		PN5180_CMD_RF_ON,
		flags,
	};

	return nfc_pn5180_exchange(dev, cmd_buf, sizeof(cmd_buf), NULL, 0U, NULL, 0U);
}

static inline int nfc_pn5180_rf_off(const struct device *dev)
{
	const uint8_t cmd_buf[2] = {
		PN5180_CMD_RF_OFF,
		0x00U,
	};

	return nfc_pn5180_exchange(dev, cmd_buf, sizeof(cmd_buf), NULL, 0U, NULL, 0U);
}

static inline int nfc_pn5180_send_data(const struct device *dev, const uint8_t *data,
				       uint16_t data_len, uint8_t last_byte_bitlen)
{
	const uint8_t cmd_buf[2] = {
		PN5180_CMD_SEND_DATA,
		last_byte_bitlen % 8U,
	};

	if (last_byte_bitlen > 8U || data_len > 260U) {
		return -EINVAL;
	}

	return nfc_pn5180_exchange(dev, cmd_buf, sizeof(cmd_buf), data, data_len, NULL, 0U);
}

static inline int nfc_pn5180_read_data(const struct device *dev, uint8_t *data, uint16_t data_len)
{
	uint8_t cmd_buf[2] = {
		PN5180_CMD_READ_DATA,
		0x00,
	};

	if (data_len > 508) {
		return -EINVAL;
	}

	return nfc_pn5180_exchange(dev, cmd_buf, sizeof(cmd_buf), NULL, 0U, data, data_len);
}

static int nfc_pn5180_init_for_irq(const struct device *dev, uint32_t irq_mask)
{
	struct nfc_pn5180_data *data = dev->data;
	int ret;

	if (irq_mask == 0U) {
		return -EINVAL;
	}

	data->irq_mask = irq_mask;

	ret = nfc_pn5180_write_register(dev, PN5180_REG_ADDR_IRQ_CLEAR, UINT32_MAX,
					PN5180_REG_ACTION_WRITE);
	if (ret < 0) {
		return ret;
	}

	k_sem_reset(&data->irq_sem);

	ret = nfc_pn5180_write_register(dev, PN5180_REG_ADDR_IRQ_ENABLE, irq_mask,
					PN5180_REG_ACTION_WRITE);

	return ret;
}

static int nfc_pn5180_wait_for_irq(const struct device *dev, k_timeout_t timeout)
{
	struct nfc_pn5180_data *data = dev->data;
	int ret;

	if (data->irq_mask == 0U) {
		return -EINVAL;
	}

	ret = k_sem_take(&data->irq_sem, timeout);
	ret = nfc_pn5180_read_register(data->self, PN5180_REG_ADDR_IRQ_STATUS, &data->irq_status);
	if (ret < 0) {
		LOG_ERR("Unable to read IRQ status (%d)", ret);
		return ret;
	}
	LOG_DBG("IRQ status %08x for mask %08x", data->irq_status, data->irq_mask);

	(void)nfc_pn5180_write_register(dev, PN5180_REG_ADDR_IRQ_CLEAR, data->irq_status,
					PN5180_REG_ACTION_WRITE);
	(void)nfc_pn5180_write_register(dev, PN5180_REG_ADDR_IRQ_ENABLE, data->irq_mask,
					PN5180_REG_ACTION_CLEAR);
	data->irq_mask = 0U;

	return ret;
}

static void nfc_pn5180_busy_handler(const struct device *port, struct gpio_callback *cb,
				    gpio_port_pins_t pins)
{
	struct nfc_pn5180_data *data = CONTAINER_OF(cb, struct nfc_pn5180_data, busy_cb);
	gpio_port_value_t val;
	int ret;

	ret = gpio_port_get(port, &val);
	if (ret < 0) {
		LOG_ERR("Failed to get BUSY port (%d)", ret);
	}

	if ((pins & val) == 0U) {
		k_sem_give(&data->busy_sem);
	} else {
		k_sem_reset(&data->busy_sem);
	}
}

static void nfc_pn5180_irq_handler(const struct device *port, struct gpio_callback *cb,
				   gpio_port_pins_t pins)
{
	struct nfc_pn5180_data *data = CONTAINER_OF(cb, struct nfc_pn5180_data, irq_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	k_sem_give(&data->irq_sem);
}

static int nfc_pn5180_claim(const struct device *dev)
{
	struct nfc_pn5180_data *data = dev->data;

	return k_mutex_lock(&data->api_lock, K_FOREVER);
}

static int nfc_pn5180_release(const struct device *dev)
{
	struct nfc_pn5180_data *data = dev->data;

	return k_mutex_unlock(&data->api_lock);
}

static int nfc_pn5180_load_protocol(const struct device *dev, nfc_proto_t proto, nfc_mode_t mode)
{
	struct nfc_pn5180_data *data = dev->data;
	uint8_t rf_tx = PN5180_RF_TX_NONE;
	uint8_t rf_rx = PN5180_RF_RX_NONE;
	int ret;

	/* Only a single mode can be active at the same time */
	if (count_bits(proto) != 1U || count_bits(mode & NFC_MODE_ROLE_MASK) != 1U ||
	    count_bits(mode & NFC_MODE_TX_MASK) > 1U || count_bits(mode & NFC_MODE_RX_MASK) > 1U) {
		return -EINVAL;
	}

	if (proto == NFC_PROTO_ISO14443A) {
		if ((mode & NFC_MODE_TX_MASK) != 0U) {
			switch (mode & (NFC_MODE_ROLE_MASK | NFC_MODE_TX_MASK)) {
			case NFC_MODE_INITIATOR | NFC_MODE_TX_106:
				rf_tx = PN5180_RF_TX_ISO14443A_106_MILLER;
				break;
			case NFC_MODE_INITIATOR | NFC_MODE_TX_212:
				rf_tx = PN5180_RF_TX_ISO14443A_212_MILLER;
				break;
			case NFC_MODE_INITIATOR | NFC_MODE_TX_424:
				rf_tx = PN5180_RF_TX_ISO14443A_424_MILLER;
				break;
			case NFC_MODE_INITIATOR | NFC_MODE_TX_848:
				rf_tx = PN5180_RF_TX_ISO14443A_848_MILLER;
				break;
			case NFC_MODE_TARGET | NFC_MODE_TX_106:
				rf_tx = PN5180_RF_TX_ISO14443A_PICC_106_MANCH_SUBC;
				break;
			case NFC_MODE_TARGET | NFC_MODE_TX_212:
				rf_tx = PN5180_RF_TX_ISO14443A_PICC_212_BPSK;
				break;
			case NFC_MODE_TARGET | NFC_MODE_TX_424:
				rf_tx = PN5180_RF_TX_ISO14443A_PICC_424_BPSK;
				break;
			case NFC_MODE_TARGET | NFC_MODE_TX_848:
				rf_tx = PN5180_RF_TX_ISO14443A_PICC_848_BPSK;
				break;
			default:
				return -EINVAL;
			}
		}

		if ((mode & NFC_MODE_RX_MASK) != 0U) {
			switch (mode & (NFC_MODE_ROLE_MASK | NFC_MODE_RX_MASK)) {
			case NFC_MODE_INITIATOR | NFC_MODE_RX_106:
				rf_rx = PN5180_RF_RX_ISO14443A_106_MANCH_SUBC;
				break;
			case NFC_MODE_INITIATOR | NFC_MODE_RX_212:
				rf_rx = PN5180_RF_RX_ISO14443A_212_BPSK;
				break;
			case NFC_MODE_INITIATOR | NFC_MODE_RX_424:
				rf_rx = PN5180_RF_RX_ISO14443A_424_BPSK;
				break;
			case NFC_MODE_INITIATOR | NFC_MODE_RX_848:
				rf_rx = PN5180_RF_RX_ISO14443A_848_BPSK;
				break;
			case NFC_MODE_TARGET | NFC_MODE_RX_106:
				rf_rx = PN5180_RF_RX_ISO14443A_PICC_106_MILLER;
				break;
			case NFC_MODE_TARGET | NFC_MODE_RX_212:
				rf_rx = PN5180_RF_RX_ISO14443A_PICC_212_MILLER;
				break;
			case NFC_MODE_TARGET | NFC_MODE_RX_424:
				rf_rx = PN5180_RF_RX_ISO14443A_PICC_424_MILLER;
				break;
			case NFC_MODE_TARGET | NFC_MODE_RX_848:
				rf_rx = PN5180_RF_RX_ISO14443A_PICC_848_MILLER;
				break;
			default:
				return -EINVAL;
			}
		}
	} else {
		return -EINVAL;
	}

	// TODO: add more RF configs

	k_mutex_lock(&data->api_lock, K_FOREVER);
	ret = nfc_pn5180_load_rf_config(dev, rf_tx, rf_rx);
	k_mutex_unlock(&data->api_lock);

	return ret;
}

static int nfc_pn5180_get_property(const struct device *dev, struct nfc_property *prop)
{
	// TODO: yes
	switch (prop->type) {
	case NFC_PROP_TIMEOUT:
		prop->timeout_us = ((struct nfc_pn5180_data *)dev->data)->timeout_us;
		return 0;
	default:
		break;
	}

	return -ENOTSUP;
}

static int nfc_pn5180_get_properties(const struct device *dev, struct nfc_property *props,
				     size_t props_len)
{
	struct nfc_pn5180_data *data = dev->data;

	__ASSERT_NO_MSG(props != NULL);

	k_mutex_lock(&data->api_lock, K_FOREVER);

	for (size_t index = 0U; index < props_len; ++index) {
		props[index].status = nfc_pn5180_get_property(dev, &props[index]);
	}

	k_mutex_unlock(&data->api_lock);

	return 0;
}

static int nfc_pn5180_set_property(const struct device *dev, struct nfc_property *prop)
{
	int ret;

	switch (prop->type) {
	case NFC_PROP_RF_FIELD: {
		union pn5180_rf_status rf_status;

		ret = nfc_pn5180_read_register(dev, PN5180_REG_ADDR_RF_STATUS, &rf_status.raw);
		if (ret < 0) {
			return ret;
		}

		/* Are we already on/off */
		if ((rf_status.tx_rf_status != 0U && prop->rf_on) ||
		    (rf_status.tx_rf_status == 0U && !prop->rf_on)) {
			return 0;
		}

		ret = nfc_pn5180_init_for_irq(dev, prop->rf_on ? PN5180_REG_IRQ_TX_RFON
							       : PN5180_REG_IRQ_TX_RFOFF);
		if (ret < 0) {
			return ret;
		}

		// TODO: Create property for RF ON flags?
		ret = prop->rf_on ? nfc_pn5180_rf_on(dev, 0x00U) : nfc_pn5180_rf_off(dev);
		if (ret < 0) {
			return ret;
		}

		// TODO: Create property for timeout
		return nfc_pn5180_wait_for_irq(dev, K_FOREVER);
	}
	case NFC_PROP_HW_TX_CRC:
		return nfc_pn5180_write_register(
			dev, PN5180_REG_ADDR_CRC_TX_CONFIG, PN5180_REG_CRC_TX_CONFIG_ENABLE,
			prop->hw_tx_crc ? PN5180_REG_ACTION_SET : PN5180_REG_ACTION_CLEAR);

	case NFC_PROP_HW_RX_CRC:
		return nfc_pn5180_write_register(
			dev, PN5180_REG_ADDR_CRC_RX_CONFIG, PN5180_REG_CRC_RX_CONFIG_ENABLE,
			prop->hw_rx_crc ? PN5180_REG_ACTION_SET : PN5180_REG_ACTION_CLEAR);

	case NFC_PROP_MFC_CRYPTO: {
		union pn5180_system_config system_config = {
			.mfc_crypto_on = 1U,
		};

		return nfc_pn5180_write_register(
			dev, PN5180_REG_ADDR_SYSTEM_CONFIG, system_config.raw,
			prop->mfc_crypto_on ? PN5180_REG_ACTION_SET : PN5180_REG_ACTION_CLEAR);
	}
	case NFC_PROP_TIMEOUT:
		((struct nfc_pn5180_data *)dev->data)->timeout_us = prop->timeout_us;

		return 0;

	case NFC_PROP_TX_GUARD_TIME:
	case NFC_PROP_RX_GUARD_TIME: {
		union pn5180_wait_config wait_config = {
			.wait_prescaler = 0x7FU,
		};
		uint32_t value = prop->type == NFC_PROP_TX_GUARD_TIME ? prop->tx_guard_us
								      : prop->rx_guard_us;

		value = (uint32_t)((PN5180_MAX_FREQUENCY * value) / USEC_PER_SEC /
				   wait_config.wait_prescaler);
		/* Max 20 bits */
		if (value > 0x0FFFFFU) {
			value = 0x0FFFFFU;
		}
		wait_config.wait_value = value;

		return nfc_pn5180_write_register(dev,
						 prop->type == NFC_PROP_TX_GUARD_TIME
							 ? PN5180_REG_ADDR_TX_WAIT_CONFIG
							 : PN5180_REG_ADDR_RX_WAIT_CONFIG,
						 wait_config.raw, PN5180_REG_ACTION_WRITE);
	}
	case NFC_PROP_RANDOM_UID: {
		uint8_t val = prop->random_uid ? 0x01U : 0x00U;

		return nfc_pn5180_write_eeprom(dev, PN5180_EEPROM_ADDR_RANDOM_UID_ENABLE, &val, 1U);
	}
	case NFC_PROP_SENS_RES:
		return nfc_pn5180_write_eeprom(dev, PN5180_EEPROM_ADDR_SENS_RES, prop->sens_res,
					       sizeof(prop->sens_res));

	case NFC_PROP_SEL_RES:
		return nfc_pn5180_write_eeprom(dev, PN5180_EEPROM_ADDR_SEL_RES, &prop->sel_res, 1U);

	default:
		break;
	}

	return -ENOTSUP;
}

static int nfc_pn5180_set_properties(const struct device *dev, struct nfc_property *props,
				     size_t props_len)
{
	struct nfc_pn5180_data *data = dev->data;

	__ASSERT_NO_MSG(props != NULL);

	k_mutex_lock(&data->api_lock, K_FOREVER);

	for (size_t index = 0; index < props_len; ++index) {
		props[index].status = nfc_pn5180_set_property(dev, &props[index]);
	}

	k_mutex_unlock(&data->api_lock);

	return 0;
}

static int nfc_pn5180_im_transceive(const struct device *dev, const uint8_t *tx_data,
				    uint16_t tx_len, uint8_t tx_last_bits, uint8_t *rx_data,
				    uint16_t *rx_len)
{
	struct nfc_pn5180_data *data = dev->data;
	union pn5180_system_config system_config = {0};
	union pn5180_rf_status rf_status = {0};
	union pn5180_rx_status rx_status = {0};
	union pn5180_timer_config timer_config = {0};
	uint32_t register_value;
	uint32_t irq_mask;
	int ret;

	k_mutex_lock(&data->api_lock, K_FOREVER);

	/* Disable TIMER1 */
	timer_config.raw = 0U;
	ret = nfc_pn5180_write_register(dev, PN5180_REG_ADDR_TIMER1_CONFIG, timer_config.raw,
					PN5180_REG_ACTION_WRITE);
	if (ret < 0) {
		LOG_ERR("Failed to disable timer1 (%d)", ret);
		goto unlock;
	}

	if (data->timeout_us > 0U) {
		/* Set TIMER1 reload */
		register_value =
			(uint32_t)((PN5180_MAX_FREQUENCY * data->timeout_us) / USEC_PER_SEC);
		ret = nfc_pn5180_write_register(dev, PN5180_REG_ADDR_TIMER1_RELOAD, register_value,
						PN5180_REG_ACTION_WRITE);
		if (ret < 0) {
			LOG_ERR("Failed to disable timer1 (%d)", ret);
			goto unlock;
		}

		/* Configure TIMER1 */
		timer_config.enable = 1U;
		timer_config.start_on_tx_ended = 1U;
		timer_config.stop_on_rx_started = 1U;
		ret = nfc_pn5180_write_register(dev, PN5180_REG_ADDR_TIMER1_CONFIG,
						timer_config.raw, PN5180_REG_ACTION_WRITE);
		if (ret < 0) {
			LOG_ERR("Failed to configure timer1 (%d)", ret);
			goto unlock;
		}
	}

	/* Set IDLE command */
	system_config.raw = 0U;
	system_config.command = 0x07U;
	ret = nfc_pn5180_write_register(dev, PN5180_REG_ADDR_SYSTEM_CONFIG, system_config.raw,
					PN5180_REG_ACTION_CLEAR);
	if (ret < 0) {
		LOG_ERR("Failed to set IDLE command (%d)", ret);
		goto unlock;
	}

	/* Set TRANSCEIVE state */
	system_config.command = 0x03U;
	ret = nfc_pn5180_write_register(dev, PN5180_REG_ADDR_SYSTEM_CONFIG, system_config.raw,
					PN5180_REG_ACTION_SET);
	if (ret < 0) {
		LOG_ERR("Failed to set TRANSCEIVE command (%d)", ret);
		goto unlock;
	}

	ret = nfc_pn5180_read_register(dev, PN5180_REG_ADDR_RF_STATUS, &rf_status.raw);
	if (ret < 0) {
		LOG_ERR("Failed read RF status (%d)", ret);
		goto unlock;
	}
	LOG_DBG("RF status 0x%08X", rf_status.raw);

	if (rf_status.transceive_state != PN5180_TRANSCEIVE_STATE_WAIT_TRANSMIT) {
		LOG_ERR("Invalid transceive state %02x", rf_status.transceive_state);
		ret = -EIO;
		goto unlock;
	}

	irq_mask = PN5180_REG_IRQ_RX | PN5180_REG_IRQ_TIMER1 | PN5180_REG_IRQ_GENERAL_ERROR;
	ret = nfc_pn5180_init_for_irq(dev, irq_mask);
	if (ret < 0) {
		LOG_ERR("Failed to init card IRQ (%d)", ret);
		goto unlock;
	}

	/* Send DATA */
	ret = nfc_pn5180_send_data(dev, tx_data, tx_len, tx_last_bits);
	if (ret < 0) {
		LOG_ERR("Failed to send data (%d)", ret);
		goto unlock;
	}

	ret = nfc_pn5180_wait_for_irq(dev, K_FOREVER);
	if (ret < 0) {
		LOG_ERR("Failed to wait data IRQ (%d)", ret);
		goto unlock;
	}

	if (data->irq_status & PN5180_REG_IRQ_TIMER1) {
		ret = -EAGAIN;
		goto unlock;
	} else if (data->irq_status & PN5180_REG_IRQ_GENERAL_ERROR) {
		ret = -EIO;
		goto unlock;
	}

	ret = nfc_pn5180_read_register(dev, PN5180_REG_ADDR_RX_STATUS, &rx_status.raw);
	if (ret < 0) {
		LOG_ERR("Failed read RX status (%d)", ret);
		goto unlock;
	}
	LOG_INF("Read %d bytes", rx_status.rx_num_bytes_received);

	if (rx_status.rx_num_bytes_received > *rx_len) {
		LOG_ERR("Invalid RX data length (%d)", rx_status.rx_num_bytes_received);
		ret = -EINVAL;
		goto unlock;
	}

	*rx_len = rx_status.rx_num_bytes_received;

	ret = nfc_pn5180_read_data(dev, rx_data, rx_status.rx_num_bytes_received);
	if (ret < 0) {
		LOG_ERR("Failed to read data (%d)", ret);
		goto unlock;
	}

	/* Disable timer again */
	timer_config.raw = 0U;
	ret = nfc_pn5180_write_register(dev, PN5180_REG_ADDR_TIMER1_CONFIG, timer_config.raw,
					PN5180_REG_ACTION_WRITE);
	if (ret < 0) {
		LOG_ERR("Failed to disable timer1 (%d)", ret);
		goto unlock;
	}

unlock:
	k_mutex_unlock(&data->api_lock);

	return ret;
}

static int nfc_pn5180_tm_transmit(const struct device *dev, const uint8_t *tx_data, uint16_t tx_len,
				  uint8_t tx_last_bits)
{
	int ret;
	struct nfc_pn5180_data *data = dev->data;
	union pn5180_rf_status rf_status = {0};
	union pn5180_timer_config timer_config = {0};
	uint32_t register_value;

	k_mutex_lock(&data->api_lock, K_FOREVER);

	ret = nfc_pn5180_read_register(dev, PN5180_REG_ADDR_RF_STATUS, &rf_status.raw);
	if (ret < 0) {
		LOG_ERR("Failed to read RF status (%d)", ret);
		goto unlock;
	}

	LOG_INF("RF status %08x", rf_status.raw);

	if (rf_status.rf_det_status != 1U ||
	    rf_status.transceive_state != PN5180_TRANSCEIVE_STATE_WAIT_TRANSMIT) {
		LOG_ERR("Invalid RF status (ext RF %d) (state %d)", rf_status.rf_det_status,
			rf_status.transceive_state);
		ret = -EINVAL;
		goto unlock;
	}

	/* Disable TIMER1 */
	timer_config.raw = 0U;
	ret = nfc_pn5180_write_register(dev, PN5180_REG_ADDR_TIMER1_CONFIG, timer_config.raw,
					PN5180_REG_ACTION_WRITE);
	if (ret < 0) {
		LOG_ERR("Failed to disable timer1 (%d)", ret);
		goto unlock;
	}

	if (data->timeout_us > 0U) {
		/* Set TIMER1 reload */
		register_value =
			(uint32_t)((PN5180_MAX_FREQUENCY * data->timeout_us) / USEC_PER_SEC);
		ret = nfc_pn5180_write_register(dev, PN5180_REG_ADDR_TIMER1_RELOAD, register_value,
						PN5180_REG_ACTION_WRITE);
		if (ret < 0) {
			LOG_ERR("Failed to disable timer1 (%d)", ret);
			goto unlock;
		}

		/* Configure TIMER1 */
		timer_config.enable = 1U;
		timer_config.start_now = 1U;
		timer_config.stop_on_tx_started = 1U;
		ret = nfc_pn5180_write_register(dev, PN5180_REG_ADDR_TIMER1_CONFIG,
						timer_config.raw, PN5180_REG_ACTION_WRITE);
		if (ret < 0) {
			LOG_ERR("Failed to configure timer1 (%d)", ret);
			goto unlock;
		}
	}

	ret = nfc_pn5180_init_for_irq(dev, PN5180_REG_IRQ_TX | PN5180_REG_IRQ_TIMER1 |
				      PN5180_REG_IRQ_GENERAL_ERROR | PN5180_REG_IRQ_RFOFF_DET);
	if (ret < 0) {
		LOG_ERR("Failed to init card IRQ (%d)", ret);
		goto unlock;
	}

	/* Send DATA */
	ret = nfc_pn5180_send_data(dev, tx_data, tx_len, tx_last_bits);
	if (ret < 0) {
		LOG_ERR("Failed to send data (%d)", ret);
		goto unlock;
	}

	ret = nfc_pn5180_wait_for_irq(dev, K_FOREVER);
	if (ret < 0) {
		LOG_ERR("Failed to wait data IRQ (%d)", ret);
		goto unlock;
	}

	if (data->irq_status & PN5180_REG_IRQ_TIMER1) {
		ret = -EAGAIN;
		goto unlock;
	} else if (data->irq_status & (PN5180_REG_IRQ_GENERAL_ERROR | PN5180_REG_IRQ_RFOFF_DET)) {
		ret = -EIO;
		goto unlock;
	}

	/* Disable timer again */
	timer_config.raw = 0U;
	ret = nfc_pn5180_write_register(dev, PN5180_REG_ADDR_TIMER1_CONFIG, timer_config.raw,
					PN5180_REG_ACTION_WRITE);
	if (ret < 0) {
		LOG_ERR("Failed to disable timer1 (%d)", ret);
		goto unlock;
	}

unlock:
	k_mutex_unlock(&data->api_lock);

	return ret;
}

static int nfc_pn5180_tm_receive(const struct device *dev, uint8_t *rx_data, uint16_t *rx_len)
{
	int ret;
	struct nfc_pn5180_data *data = dev->data;
	union pn5180_rf_status rf_status = {0};
	union pn5180_rx_status rx_status = {0};
	union pn5180_timer_config timer_config = {0};
	uint32_t register_value;

	k_mutex_lock(&data->api_lock, K_FOREVER);

	ret = nfc_pn5180_read_register(dev, PN5180_REG_ADDR_RF_STATUS, &rf_status.raw);
	if (ret < 0) {
		LOG_ERR("Failed to read RF status (%d)", ret);
		goto unlock;
	}

	LOG_INF("Receive RX status %08x", rf_status.raw);

	if (rf_status.rf_det_status != 1U) {
		LOG_ERR("No ext RF");
		ret = -EINVAL;
		goto unlock;
	}

	/* Disable TIMER1 */
	timer_config.raw = 0U;
	ret = nfc_pn5180_write_register(dev, PN5180_REG_ADDR_TIMER1_CONFIG, timer_config.raw,
					PN5180_REG_ACTION_WRITE);
	if (ret < 0) {
		LOG_ERR("Failed to disable timer1 (%d)", ret);
		goto unlock;
	}

	if (data->timeout_us > 0U) {
		/* Set TIMER1 reload */
		register_value =
			(uint32_t)((PN5180_MAX_FREQUENCY * data->timeout_us) / USEC_PER_SEC);
		ret = nfc_pn5180_write_register(dev, PN5180_REG_ADDR_TIMER1_RELOAD, register_value,
						PN5180_REG_ACTION_WRITE);
		if (ret < 0) {
			LOG_ERR("Failed to disable timer1 (%d)", ret);
			goto unlock;
		}

		/* Configure TIMER1 */
		timer_config.enable = 1U;
		timer_config.start_now = 1U;
		timer_config.stop_on_rx_started = 1U;
		ret = nfc_pn5180_write_register(dev, PN5180_REG_ADDR_TIMER1_CONFIG,
						timer_config.raw, PN5180_REG_ACTION_WRITE);
		if (ret < 0) {
			LOG_ERR("Failed to configure timer1 (%d)", ret);
			goto unlock;
		}
	}

	ret = nfc_pn5180_init_for_irq(dev, PN5180_REG_IRQ_RX | PN5180_REG_IRQ_TIMER1 |
				      PN5180_REG_IRQ_GENERAL_ERROR | PN5180_REG_IRQ_RFOFF_DET);
	if (ret < 0) {
		LOG_ERR("Failed to init card IRQ (%d)", ret);
		goto unlock;
	}

	ret = nfc_pn5180_wait_for_irq(dev, K_FOREVER);
	if (ret < 0) {
		LOG_ERR("Failed to wait data IRQ (%d)", ret);
		goto unlock;
	}

	if (data->irq_status & PN5180_REG_IRQ_TIMER1) {
		ret = -EAGAIN;
		goto unlock;
	} else if (data->irq_status & (PN5180_REG_IRQ_GENERAL_ERROR | PN5180_REG_IRQ_RFOFF_DET)) {
		ret = -EIO;
		goto unlock;
	}

	ret = nfc_pn5180_read_register(dev, PN5180_REG_ADDR_RX_STATUS, &rx_status.raw);
	if (ret < 0) {
		LOG_ERR("Failed read RX status (%d)", ret);
		goto unlock;
	}
	LOG_INF("Read %d bytes", rx_status.rx_num_bytes_received);

	if (rx_status.rx_num_bytes_received > *rx_len) {
		LOG_ERR("Invalid RX data length (%d)", rx_status.rx_num_bytes_received);
		ret = -EINVAL;
		goto unlock;
	}

	*rx_len = rx_status.rx_num_bytes_received;

	ret = nfc_pn5180_read_data(dev, rx_data, rx_status.rx_num_bytes_received);
	if (ret < 0) {
		LOG_ERR("Failed to read data (%d)", ret);
		goto unlock;
	}

	/* Disable timer again */
	timer_config.raw = 0U;
	ret = nfc_pn5180_write_register(dev, PN5180_REG_ADDR_TIMER1_CONFIG, timer_config.raw,
					PN5180_REG_ACTION_WRITE);
	if (ret < 0) {
		LOG_ERR("Failed to disable timer1 (%d)", ret);
		goto unlock;
	}

unlock:
	k_mutex_unlock(&data->api_lock);

	return ret;
}

static int nfc_pn5180_listen(const struct device *dev, nfc_proto_t proto, uint8_t *rx_data,
			     uint16_t *rx_len)
{
	struct nfc_pn5180_data *data = dev->data;
	union pn5180_rx_status rx_status = {0};
	union pn5180_system_config system_config = {0};
	union pn5180_emd_control emd_control = {0};
	int ret;
	uint8_t tech = 0U;
	uint32_t irq_mask;

	if (proto & NFC_PROTO_FELICA) {
		tech |= PN5180_AUTOCOLL_TECH_NFC_F_PASSIVE;
	}
	if (proto & NFC_PROTO_ISO14443A) {
		tech |= PN5180_AUTOCOLL_TECH_NFC_A_PASSIVE;
	}

	/* Only NFC-F/NFC-A supported */
	if (tech == 0U) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->api_lock, K_FOREVER);

	/* Disable EMD control */
	emd_control.enable = 1U;
	ret = nfc_pn5180_write_register(dev, PN5180_REG_ADDR_EMD_CONTROL, system_config.raw,
					PN5180_REG_ACTION_CLEAR);
	if (ret < 0) {
		LOG_ERR("Failed to disable EMD control (%d)", ret);
		goto unlock;
	}

	/* Set IDLE command */
	system_config.command = 0x07U;
	ret = nfc_pn5180_write_register(dev, PN5180_REG_ADDR_SYSTEM_CONFIG, system_config.raw,
					PN5180_REG_ACTION_CLEAR);
	if (ret < 0) {
		LOG_ERR("Failed to set IDLE command (%d)", ret);
		goto unlock;
	}

	ret = nfc_pn5180_rf_off(dev);
	if (ret < 0) {
		LOG_ERR("Failed to turn off RF (%d)", ret);
		goto unlock;
	}

	irq_mask = PN5180_REG_IRQ_RX | PN5180_REG_IRQ_CARD_ACTIVATED | PN5180_REG_IRQ_TIMER1 |
		   PN5180_REG_IRQ_GENERAL_ERROR;

	ret = nfc_pn5180_init_for_irq(dev, irq_mask);
	if (ret < 0) {
		LOG_ERR("Failed to init card emulation IRQ (%d)", ret);
		goto unlock;
	}

	LOG_INF("Starting autocoll %02x", tech);

	ret = nfc_pn5180_switch_mode_autocoll(dev, tech, PN5180_AUTOCOLL_MODE_AUTO_NO_STANDBY);
	if (ret < 0) {
		LOG_ERR("Failed to switch autocoll mode (%d)", ret);
		goto unlock;
	}

	ret = nfc_pn5180_wait_for_irq(dev, K_FOREVER);
	if (ret < 0) {
		LOG_ERR("Failed to wait emulation IRQ (%d)", ret);
		goto unlock;
	}

	if (data->irq_status & PN5180_REG_IRQ_TIMER1) {
		(void)nfc_pn5180_switch_mode_normal(dev);
		ret = -EAGAIN;
		goto unlock;
	} else if (data->irq_status & (PN5180_REG_IRQ_GENERAL_ERROR | PN5180_REG_IRQ_RFOFF_DET)) {
		(void)nfc_pn5180_switch_mode_normal(dev);
		ret = -EIO;
		goto unlock;
	}

	ret = nfc_pn5180_read_register(dev, PN5180_REG_ADDR_RX_STATUS, &rx_status.raw);
	if (ret < 0) {
		LOG_ERR("Failed read RX status (%d)", ret);
		goto unlock;
	}

	if (rx_status.rx_data_integrity_error != 0U) {
		ret = -EINVAL;
		goto unlock;
	}

	LOG_INF("Read %d bytes", rx_status.rx_num_bytes_received);

	ret = nfc_pn5180_read_data(dev, rx_data, rx_status.rx_num_bytes_received);
	if (ret < 0) {
		LOG_ERR("Failed to read data (%d)", ret);
		goto unlock;
	}

	*rx_len = rx_status.rx_num_bytes_received;

unlock:
	k_mutex_unlock(&data->api_lock);

	return ret;
}

static nfc_proto_t nfc_pn5180_supported_protocols(const struct device *dev)
{
	ARG_UNUSED(dev);

	return NFC_PROTO_ISO14443A;
}

static nfc_mode_t nfc_pn5180_supported_modes(const struct device *dev, nfc_proto_t proto)
{
	ARG_UNUSED(dev);

	if (proto == NFC_PROTO_ISO14443A) {
		return NFC_MODE_INITIATOR | NFC_MODE_TARGET | NFC_MODE_RX_106 | NFC_MODE_TX_106 |
		       NFC_MODE_RX_212 | NFC_MODE_TX_212 | NFC_MODE_RX_424 | NFC_MODE_TX_424 |
		       NFC_MODE_RX_848 | NFC_MODE_TX_848;
	}

	return 0;
}

static int nfc_pn5180_init(const struct device *dev)
{
	const struct nfc_pn5180_config *cfg = dev->config;
	struct nfc_pn5180_data *data = dev->data;
	uint8_t eeprom_buf[2];
	int ret;

	data->self = dev;

	(void)k_mutex_init(&data->api_lock);
	(void)k_sem_init(&data->irq_sem, 0, 1);
	(void)k_sem_init(&data->busy_sem, 1, 1);

	if (!spi_is_ready_dt(&cfg->bus)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&cfg->gpio_busy)) {
		LOG_ERR("BUSY GPIO not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&cfg->gpio_irq)) {
		LOG_ERR("IRQ GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->gpio_busy, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure BUSY GPIO (%d)", ret);
		return ret;
	}

	gpio_init_callback(&data->busy_cb, nfc_pn5180_busy_handler, BIT(cfg->gpio_busy.pin));

	ret = gpio_add_callback_dt(&cfg->gpio_busy, &data->busy_cb);
	if (ret < 0) {
		LOG_ERR("Failed to add BUSY GPIO callback (%d)", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->gpio_busy, GPIO_INT_EDGE_BOTH);
	if (ret < 0) {
		LOG_ERR("Failed to configure BUSY GPIO interrupt (%d)", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&cfg->gpio_irq, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure IRQ GPIO (%d)", ret);
		return ret;
	}

	gpio_init_callback(&data->irq_cb, nfc_pn5180_irq_handler, BIT(cfg->gpio_irq.pin));

	ret = gpio_add_callback_dt(&cfg->gpio_irq, &data->irq_cb);
	if (ret < 0) {
		LOG_ERR("Failed to add IRQ GPIO callback (%d)", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->gpio_irq, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure IRQ GPIO interrupt (%d)", ret);
		return ret;
	}

	if (cfg->gpio_reset.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->gpio_reset)) {
			LOG_ERR("RST GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->gpio_reset, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Failed to configure RST GPIO (%d)", ret);
			return ret;
		}

		k_busy_wait(10);

		ret = gpio_pin_set_dt(&cfg->gpio_reset, 0);
		if (ret < 0) {
			LOG_ERR("Failed to de-assert RST GPIO (%d)", ret);
			return ret;
		}
	} else {
		union pn5180_system_config cfg = {
			.soft_reset = 0x01U,
		};
		/* Soft reset */
		ret = nfc_pn5180_write_register(dev, PN5180_REG_ADDR_SYSTEM_CONFIG, cfg.raw,
						PN5180_REG_ACTION_SET);

		if (ret < 0) {
			LOG_ERR("Failed soft reset (%d)", ret);
			return ret;
		}
	}

	k_msleep(2);

	ret = nfc_pn5180_read_eeprom(dev, PN5180_EEPROM_ADDR_TESTBUS_ENABLE, &data->testbus_enabled,
				     1);
	if (ret < 0) {
		LOG_ERR("Failed to read testbus (%d)", ret);
		return ret;
	}
	LOG_DBG("Testbus: %02X", data->testbus_enabled);

	ret = nfc_pn5180_read_eeprom(dev, PN5180_EEPROM_ADDR_FIRMWARE_VERSION, eeprom_buf, 2);
	if (ret < 0) {
		LOG_ERR("Failed to read FW version (%d)", ret);
		return ret;
	}
	data->fw_version = sys_get_le16(eeprom_buf);

	if (data->fw_version == 0xFFFFU) {
		LOG_ERR("Read FW version SPI issue (got 0xFFFF)");
		return -ENODEV;
	}

	LOG_INF("FW version %04X", data->fw_version);
	if (data->fw_version < 0x0304U) {
		LOG_ERR("Invalid FW version %04x", data->fw_version);
		return -ENODEV;
	}

	ret = nfc_pn5180_read_eeprom(dev, PN5180_EEPROM_ADDR_IRQ_PIN_CONFIG, eeprom_buf, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read IRQ pin config (%d)", ret);
		return ret;
	}
	LOG_INF("IRQ pin config is 0x%02X", eeprom_buf[0]);

	// TODO: Check Digital Delay if FW >= 3.8

	return 0;
}

static const struct nfc_driver_api nfc_pn5180_driver_api = {
	.claim = nfc_pn5180_claim,
	.release = nfc_pn5180_release,
	.load_protocol = nfc_pn5180_load_protocol,
	.get_properties = nfc_pn5180_get_properties,
	.set_properties = nfc_pn5180_set_properties,
	.im_transceive = nfc_pn5180_im_transceive,
	.tm_transmit = nfc_pn5180_tm_transmit,
	.tm_receive = nfc_pn5180_tm_receive,
	.listen = nfc_pn5180_listen,
	.supported_protocols = nfc_pn5180_supported_protocols,
	.supported_modes = nfc_pn5180_supported_modes,
};

#define NFC_PN5180_SPI_CFG SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB

#define NFC_PN5180_INIT(inst)                                                                      \
	static const struct nfc_pn5180_config nfc_pn5180_config_##inst = {                         \
		.bus = SPI_DT_SPEC_INST_GET(inst, NFC_PN5180_SPI_CFG, 1),                          \
		.gpio_busy = GPIO_DT_SPEC_INST_GET(inst, busy_gpios),                              \
		.gpio_irq = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),                                \
		.gpio_reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, 0),                      \
	};                                                                                         \
	static struct nfc_pn5180_data nfc_pn5180_data_##inst;                                      \
	DEVICE_DT_INST_DEFINE(inst, nfc_pn5180_init, NULL, &nfc_pn5180_data_##inst,                \
			      &nfc_pn5180_config_##inst, POST_KERNEL, CONFIG_NFC_INIT_PRIORITY,    \
			      &nfc_pn5180_driver_api);

DT_INST_FOREACH_STATUS_OKAY(NFC_PN5180_INIT)
