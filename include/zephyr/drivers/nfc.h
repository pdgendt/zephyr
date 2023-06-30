/*
 * Copyright (c) 2023 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public API for NFC drivers
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_NFC_H_
#define ZEPHYR_INCLUDE_DRIVERS_NFC_H_

#include <errno.h>

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys_clock.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief NFC Interface
 * @defgroup nfc_interface NFC Interface
 * @ingroup io_interfaces
 * @{
 */

/**
 * @name NFC protocol definitions
 * @anchor NFC_PROTO_DEFS
 * @{
 */

/** NFC protocol Jewel tag by Broadcom */
#define NFC_PROTO_JEWEL		BIT(0)

/** NFC protocol MIFARE ICs by NXP */
#define NFC_PROTO_MIFARE	BIT(1)

/** NFC protocol Felica by Sony */
#define NFC_PROTO_FELICA	BIT(2)

/** NFC protocol ISO/IEC 14443 A */
#define NFC_PROTO_ISO14443A	BIT(3)

/** NFC protocol ISO/IEC 14443 B */
#define NFC_PROTO_ISO14443B	BIT(4)

/** NFC protocol ISO/IEC 7816-4 */
#define NFC_PROTO_NFC_DEP	BIT(5)

/** NFC protocol ISO/IEC 15693 */
#define NFC_PROTO_ISO15693	BIT(6)

/** @} */

/**
 * @brief Provides a type to hold NFC protocol definitions.
 *
 * @see @ref NFC_PROTO_DEFS.
 */
typedef uint32_t nfc_proto_t;

/**
 * @name NFC mode flags
 * @anchor NFC_MODE_FLAGS
 * @{
 */

/** NFC initiator mode */
#define NFC_MODE_INITIATOR		BIT(0)

/** NFC target mode */
#define NFC_MODE_TARGET			BIT(1)

/** NFC peer-to-peer mode */
#define NFC_MODE_P2P			BIT(2)

/** Helper mask to get Initiator/Target/P2P mode */
#define NFC_MODE_ROLE_MASK		(NFC_MODE_INITIATOR | NFC_MODE_TARGET | NFC_MODE_P2P)

/** NFC baudrate in both directions have to be the same */
#define NFC_MODE_TX_RX_SAME_RATE	BIT(3)

/** NFC transmit baudrate of 106 Kbps */
#define NFC_MODE_TX_106			BIT(4)

/** NFC transmit baudrate of 212 Kbps */
#define NFC_MODE_TX_212			BIT(5)

/** NFC transmit baudrate of 424 Kbps */
#define NFC_MODE_TX_424			BIT(6)

/** NFC transmit baudrate of 848 Kbps */
#define NFC_MODE_TX_848			BIT(7)

/** Helper mask to get TX modes */
#define NFC_MODE_TX_MASK		(NFC_MODE_TX_106 | NFC_MODE_TX_212 | \
					 NFC_MODE_TX_424 | NFC_MODE_TX_848)

/** NFC receive baudrate of 106 Kbps */
#define NFC_MODE_RX_106			BIT(8)

/** NFC receive baudrate of 212 Kbps */
#define NFC_MODE_RX_212			BIT(9)

/** NFC receive baudrate of 424 Kbps */
#define NFC_MODE_RX_424			BIT(10)

/** NFC receive baudrate of 848 Kbps */
#define NFC_MODE_RX_848			BIT(11)

/** Helper mask to get RX modes */
#define NFC_MODE_RX_MASK		(NFC_MODE_RX_106 | NFC_MODE_RX_212 | \
					 NFC_MODE_RX_424 | NFC_MODE_RX_848)

/** @} */

/**
 * @brief Provides a type to hold NFC mode flags.
 *
 * @see @ref NFC_MODE_FLAGS.
 */
typedef uint32_t nfc_mode_t;

/**
 * @brief Defines the properties for an NFC device driver.
 */
enum nfc_property_type {
	/** Turn RF field on/off (field rf_on) */
	NFC_PROP_RF_FIELD,

	/** Turn HW CRC on/off for transmitting (field hw_tx_crc) */
	NFC_PROP_HW_TX_CRC,

	/** Turn HW CRC on/off for receiving (field hw_rx_crc) */
	NFC_PROP_HW_RX_CRC,

	/** Turn HW parity on/off (field hw_parity) */
	NFC_PROP_HW_PARITY,

	/** Turn MIFARE Classic crypto on/off (field mfc_crypto_on) */
	NFC_PROP_MFC_CRYPTO,

	/** Timeout in microseconds to wait for a response (field timeout_us) */
	NFC_PROP_TIMEOUT,

	/** Time in microseconds to wait before sending data (field tx_guard_us) */
	NFC_PROP_TX_GUARD_TIME,

	/** Time in microseconds to wait before receiving incoming data (field rx_guard_us) */
	NFC_PROP_RX_GUARD_TIME,

	/** Generate a random UID (field random_uid) */
	NFC_PROP_RANDOM_UID,

	/** Response value to ReqA (ATQA) (field sens_res) */
	NFC_PROP_SENS_RES,

	/** Response value to SelectA (field sel_res) */
	NFC_PROP_SEL_RES,
};

/**
 * @brief NFC property structure
 */
struct nfc_property {
	/** NFC property type to access */
	enum nfc_property_type type;

	/** Negative error status set by callee e.g. -ENOTSUP for an unsupported property */
	int status;

	union {
		/* Fields have the format: */
		/* NFC_PROPERTY_FIELD */
		/* type property_field; */

		/* NFC_PROP_RF_FIELD */
		bool rf_on;
		/* NFC_PROP_HW_TX_CRC */
		bool hw_tx_crc;
		/* NFC_PROP_HW_RX_CRC */
		bool hw_rx_crc;
		/* NFC_PROP_HW_PARITY */
		bool hw_parity;
		/* NFC_PROP_MFC_CRYPTO */
		bool mfc_crypto_on;
		/* NFC_PROP_TIMEOUT */
		uint32_t timeout_us;
		/* NFC_PROP_TX_GUARD_TIME */
		uint32_t tx_guard_us;
		/* NFC_PROP_RX_GUARD_TIME */
		uint32_t rx_guard_us;
		/* NFC_PROP_RANDOM_UID */
		bool random_uid;
		/* NFC_PROP_SENS_RES */
		uint8_t sens_res[2];
		/* NFC_PROP_SEL_RES */
		uint8_t sel_res;
	};
};

/**
 * @cond INTERNAL_HIDDEN
 *
 * For internal driver use only, skip these in public documentation.
 */

/**
 * @brief Callback API upon claiming an NFC device.
 * See @a nfc_claim() for argument descriptions
 */
typedef int (*nfc_claim_t)(const struct device *dev);

/**
 * @brief Callback API upon releasing an NFC device.
 * See @a nfc_release() for argument descriptions
 */
typedef int (*nfc_release_t)(const struct device *dev);

/**
 * @brief Callback API upon loading an NFC protocol.
 * See @a nfc_load_protocol() for argument descriptions
 */
typedef int (*nfc_load_protocol_t)(const struct device *dev, nfc_proto_t proto, nfc_mode_t mode);

/**
 * @brief Callback API upon getting NFC properties.
 * See @a nfc_get_property() for argument descriptions
 */
typedef int (*nfc_properties_get_t)(const struct device *dev, struct nfc_property *props,
				    size_t props_len);

/**
 * @brief Callback API upon setting NFC properties.
 * See @a nfc_set_property() for argument descriptions
 */
typedef int (*nfc_properties_set_t)(const struct device *dev, struct nfc_property *props,
				    size_t props_len);

/**
 * @brief Callback API upon sending and receiving data as initiator/reader.
 * See @a nfc_initiator_transceive() for argument descriptions
 */
typedef int (*nfc_initiator_transceive_t)(const struct device *dev, const uint8_t *tx_data,
					  uint16_t tx_len, uint8_t tx_last_bits,
					  uint8_t *rx_data, uint16_t *rx_len);

/**
 * @brief Callback API upon transmitting as target.
 * See @a nfc_target_send() for argument descriptions
 */
typedef int (*nfc_target_transmit_t)(const struct device *dev, const uint8_t *tx_data,
				     uint16_t tx_len, uint8_t tx_last_bits);

/**
 * @brief Callback API upon receiving as target.
 * See @a nfc_target_send() for argument descriptions
 */
typedef int (*nfc_target_receive_t)(const struct device *dev, uint8_t *rx_data, uint16_t *rx_len);

/**
 * @brief Callback API upon listening for NFC data.
 * See @a nfc_listen() for argument descriptions
 */
typedef int (*nfc_listen_t)(const struct device *dev, nfc_proto_t proto, uint8_t *rx_data,
			    uint16_t *rx_len);

/**
 * @brief Callback API upon getting the NFC device driver supported protocols.
 * See @a nfc_supported_protocols() for argument descriptions
 */
typedef nfc_proto_t (*nfc_supported_protocols_t)(const struct device *dev);

/**
 * @brief Callback API upon getting the NFC device driver supported modes.
 * See @a nfc_supported_modes() for argument descriptions
 */
typedef nfc_mode_t (*nfc_supported_modes_t)(const struct device *dev, nfc_proto_t proto);

__subsystem struct nfc_driver_api {
	nfc_claim_t claim;
	nfc_release_t release;
	nfc_load_protocol_t load_protocol;
	nfc_properties_get_t get_properties;
	nfc_properties_set_t set_properties;
	nfc_initiator_transceive_t im_transceive;
	nfc_listen_t listen;
	nfc_target_transmit_t tm_transmit;
	nfc_target_receive_t tm_receive;
	nfc_supported_protocols_t supported_protocols;
	nfc_supported_modes_t supported_modes;
};

/** @endcond */

/**
 * @name NFC controller configuration
 *
 * @{
 */

/**
 * @brief
 *
 * TODO
 */
__syscall nfc_proto_t nfc_claim(const struct device *dev);

static inline nfc_proto_t z_impl_nfc_claim(const struct device *dev)
{
	const struct nfc_driver_api *api = (const struct nfc_driver_api *)dev->api;

	return api->claim(dev);
}

/**
 * @brief
 *
 * TODO
 */
__syscall nfc_proto_t nfc_release(const struct device *dev);

static inline nfc_proto_t z_impl_nfc_release(const struct device *dev)
{
	const struct nfc_driver_api *api = (const struct nfc_driver_api *)dev->api;

	return api->release(dev);
}

/**
 * @brief
 *
 * TODO
 */
__syscall int nfc_load_protocol(const struct device *dev, nfc_proto_t proto, nfc_mode_t mode);

static inline int z_impl_nfc_load_protocol(const struct device *dev, nfc_proto_t proto,
					   nfc_mode_t mode)
{
	const struct nfc_driver_api *api = (const struct nfc_driver_api *)dev->api;

	if (api->load_protocol == NULL) {
		return -ENOSYS;
	}

	return api->load_protocol(dev, proto, mode);
}

/**
 * @brief
 *
 * TODO
 */
__syscall int nfc_get_properties(const struct device *dev, struct nfc_property *props,
				 size_t props_len);

static inline int z_impl_nfc_get_properties(const struct device *dev, struct nfc_property *props,
					    size_t props_len)
{
	const struct nfc_driver_api *api = (const struct nfc_driver_api *)dev->api;

	if (api->get_properties == NULL) {
		return -ENOSYS;
	}

	return api->get_properties(dev, props, props_len);
}

/**
 * @brief
 *
 * TODO
 */
__syscall int nfc_set_properties(const struct device *dev, struct nfc_property *props,
				 size_t props_len);

static inline int z_impl_nfc_set_properties(const struct device *dev, struct nfc_property *props,
					    size_t props_len)
{
	const struct nfc_driver_api *api = (const struct nfc_driver_api *)dev->api;

	if (api->set_properties == NULL) {
		return -ENOSYS;
	}

	return api->set_properties(dev, props, props_len);
}

/**
 * @brief
 *
 * TODO
 */
__syscall int nfc_initiator_transceive(const struct device *dev, const uint8_t *tx_data,
				       uint16_t tx_len, uint8_t tx_last_bits,
				       uint8_t *rx_data, uint16_t *rx_len);

static inline int z_impl_nfc_initiator_transceive(const struct device *dev, const uint8_t *tx_data,
						  uint16_t tx_len, uint8_t tx_last_bits,
						  uint8_t *rx_data, uint16_t *rx_len)
{
	const struct nfc_driver_api *api = (const struct nfc_driver_api *)dev->api;

	if (api->im_transceive == NULL) {
		return -ENOSYS;
	}

	return api->im_transceive(dev, tx_data, tx_len, tx_last_bits, rx_data, rx_len);
}

/**
 * @brief
 *
 * TODO
 */
__syscall int nfc_target_transmit(const struct device *dev, const uint8_t *tx_data,
				  uint16_t tx_len, uint8_t tx_last_bits);

static inline int z_impl_nfc_target_transmit(const struct device *dev, const uint8_t *tx_data,
					     uint16_t tx_len, uint8_t tx_last_bits)
{
	const struct nfc_driver_api *api = (const struct nfc_driver_api *)dev->api;

	if (api->tm_transmit == NULL) {
		return -ENOSYS;
	}

	return api->tm_transmit(dev, tx_data, tx_len, tx_last_bits);
}

/**
 * @brief
 *
 * TODO
 */
__syscall int nfc_target_receive(const struct device *dev, uint8_t *rx_data, uint16_t *rx_len);

static inline int z_impl_nfc_target_receive(const struct device *dev, uint8_t *rx_data,
					    uint16_t *rx_len)
{
	const struct nfc_driver_api *api = (const struct nfc_driver_api *)dev->api;

	if (api->tm_receive == NULL) {
		return -ENOSYS;
	}

	return api->tm_receive(dev, rx_data, rx_len);
}

/**
 * @brief
 *
 * TODO
 */
__syscall int nfc_listen(const struct device *dev, nfc_proto_t proto, uint8_t *rx_data,
			 uint16_t *rx_len);

static inline int z_impl_nfc_listen(const struct device *dev, nfc_proto_t proto, uint8_t *rx_data,
				    uint16_t *rx_len)
{
	const struct nfc_driver_api *api = (const struct nfc_driver_api *)dev->api;

	if (api->listen == NULL) {
		return -ENOSYS;
	}

	return api->listen(dev, proto, rx_data, rx_len);
}

/**
 * @brief
 *
 * TODO
 */
__syscall nfc_proto_t nfc_supported_protocols(const struct device *dev);

static inline nfc_proto_t z_impl_nfc_supported_protocols(const struct device *dev)
{
	const struct nfc_driver_api *api = (const struct nfc_driver_api *)dev->api;

	if (api->supported_protocols == NULL) {
		return -ENOSYS;
	}

	return api->supported_protocols(dev);
}

/**
 * @brief
 *
 * TODO
 */
__syscall nfc_mode_t nfc_supported_modes(const struct device *dev, nfc_proto_t proto);

static inline nfc_mode_t z_impl_nfc_supported_modes(const struct device *dev, nfc_proto_t proto)
{
	const struct nfc_driver_api *api = (const struct nfc_driver_api *)dev->api;

	if (api->supported_modes == NULL) {
		return -ENOSYS;
	}

	return api->supported_modes(dev, proto);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#include <syscalls/nfc.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_NFC_H_ */
