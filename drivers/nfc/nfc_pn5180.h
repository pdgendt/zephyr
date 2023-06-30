/*
 * NXP PN5180 NFC Driver Definitions
 *
 * Copyright (c) 2023 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_NFC_PN5180_H_
#define ZEPHYR_DRIVERS_NFC_PN5180_H_

#include <zephyr/sys/util_macro.h>

#define PN5180_CMD_WRITE_REGISTER				0x00U
#define PN5180_CMD_WRITE_REGISTER_OR_MASK			0x01U
#define PN5180_CMD_WRITE_REGISTER_AND_MASK			0x02U
#define PN5180_CMD_WRITE_REGISTER_MULTIPLE			0x03U
#define PN5180_CMD_READ_REGISTER				0x04U
#define PN5180_CMD_READ_REGISTER_MULTIPLE			0x05U
#define PN5180_CMD_WRITE_EEPROM					0x06U
#define PN5180_CMD_READ_EEPROM					0x07U
#define PN5180_CMD_WRITE_TX_DATA				0x08U
#define PN5180_CMD_SEND_DATA					0x09U
#define PN5180_CMD_READ_DATA					0x0AU
#define PN5180_CMD_SWITCH_MODE					0x0BU
#define PN5180_CMD_MIFARE_AUTHENTICATE				0x0CU
#define PN5180_CMD_EPC_INVENTORY				0x0DU
#define PN5180_CMD_EPC_RESUME_INVENTORY				0x0EU
#define PN5180_CMD_EPC_RETRIEVE_INVENTORY_RESULT_SIZE		0x0FU
#define PN5180_CMD_EPC_RETRIEVE_INVENTORY_RESULT		0x10U
#define PN5180_CMD_LOAD_RF_CONFIG				0x11U
#define PN5180_CMD_UPDATE_RF_CONFIG				0x12U
#define PN5180_CMD_RETRIEVE_RF_CONFIG_SIZE			0x13U
#define PN5180_CMD_RETRIEVE_RF_CONFIG				0x14U
#define PN5180_CMD_RF_ON					0x16U
#define PN5180_CMD_RF_OFF					0x17U

#define PN5180_REG_ADDR_SYSTEM_CONFIG				0x00U
#define PN5180_REG_ADDR_IRQ_ENABLE				0x01U
#define PN5180_REG_ADDR_IRQ_STATUS				0x02U
#define PN5180_REG_ADDR_IRQ_CLEAR				0x03U
#define PN5180_REG_ADDR_TRANSCEIVER_CONFIG			0x04U
#define PN5180_REG_ADDR_PADCONFIG				0x05U
#define PN5180_REG_ADDR_PADOUT					0x07U
#define PN5180_REG_ADDR_TIMER0_STATUS				0x08U
#define PN5180_REG_ADDR_TIMER1_STATUS				0x09U
#define PN5180_REG_ADDR_TIMER2_STATUS				0x0AU
#define PN5180_REG_ADDR_TIMER0_RELOAD				0x0BU
#define PN5180_REG_ADDR_TIMER1_RELOAD				0x0CU
#define PN5180_REG_ADDR_TIMER2_RELOAD				0x0DU
#define PN5180_REG_ADDR_TIMER0_CONFIG				0x0EU
#define PN5180_REG_ADDR_TIMER1_CONFIG				0x0FU
#define PN5180_REG_ADDR_TIMER2_CONFIG				0x10U
#define PN5180_REG_ADDR_RX_WAIT_CONFIG				0x11U
#define PN5180_REG_ADDR_CRC_RX_CONFIG				0x12U
#define PN5180_REG_ADDR_RX_STATUS				0x13U
#define PN5180_REG_ADDR_TX_UNDERSHOOT_CONFIG			0x14U
#define PN5180_REG_ADDR_TX_OVERSHOOT_CONFIG			0x15U
#define PN5180_REG_ADDR_TX_DATA_MOD				0x16U
#define PN5180_REG_ADDR_TX_WAIT_CONFIG				0x17U
#define PN5180_REG_ADDR_TX_CONFIG				0x18U
#define PN5180_REG_ADDR_CRC_TX_CONFIG				0x19U
#define PN5180_REG_ADDR_SIGPRO_CONFIG				0x1AU
#define PN5180_REG_ADDR_SIGPRO_CM_CONFIG			0x1BU
#define PN5180_REG_ADDR_SIGPRO_RM_CONFIG			0x1CU
#define PN5180_REG_ADDR_RF_STATUS				0x1DU
#define PN5180_REG_ADDR_AGC_CONFIG				0x1EU
#define PN5180_REG_ADDR_AGC_VALUE				0x1FU
#define PN5180_REG_ADDR_RF_CONTROL_TX				0x20U
#define PN5180_REG_ADDR_RF_CONTROL_TX_CLK			0x21U
#define PN5180_REG_ADDR_RF_CONTROL_RX				0x22U
#define PN5180_REG_ADDR_LD_CONTROL				0x23U
#define PN5180_REG_ADDR_SYSTEM_STATUS				0x24U
#define PN5180_REG_ADDR_TEMP_CONTROL				0x25U
#define PN5180_REG_ADDR_AGC_REF_CONFIG				0x26U
#define PN5180_REG_ADDR_DPC_CONFIG				0x27U
#define PN5180_REG_ADDR_EMD_CONTROL				0x28U
#define PN5180_REG_ADDR_ANT_CONTROL				0x29U
#define PN5180_REG_ADDR_TX_CONTROL				0x36U
#define PN5180_REG_ADDR_SIGPRO_RM_CONFIG_EXTENSION		0x39U
#define PN5180_REG_ADDR_FELICA_EMD_CONTROL			0x43U

/* Flags for IRQ_ENABLE/IRQ_STATUS/IRQ_CLEAR registers */
#define PN5180_REG_IRQ_ALL	 				0xFFFFFFFFU
#define PN5180_REG_IRQ_LPCD	 				BIT(19)
#define PN5180_REG_IRQ_HV_ERROR	 				BIT(18)
#define PN5180_REG_IRQ_GENERAL_ERROR	 			BIT(17)
#define PN5180_REG_IRQ_TEMPSENS_ERROR	 			BIT(16)
#define PN5180_REG_IRQ_RX_SC_DET	 			BIT(15)
#define PN5180_REG_IRQ_RX_SOF_DET	 			BIT(14)
#define PN5180_REG_IRQ_TIMER2	 				BIT(13)
#define PN5180_REG_IRQ_TIMER1	 				BIT(12)
#define PN5180_REG_IRQ_TIMER0	 				BIT(11)
#define PN5180_REG_IRQ_RF_ACTIVE_ERROR	 			BIT(10)
#define PN5180_REG_IRQ_TX_RFON	 				BIT(9)
#define PN5180_REG_IRQ_TX_RFOFF	 				BIT(8)
#define PN5180_REG_IRQ_RFON_DET	 				BIT(7)
#define PN5180_REG_IRQ_RFOFF_DET	 			BIT(6)
#define PN5180_REG_IRQ_STATE_CHANGE	 			BIT(5)
#define PN5180_REG_IRQ_CARD_ACTIVATED	 			BIT(4)
#define PN5180_REG_IRQ_MODE_DETECTED	 			BIT(3)
#define PN5180_REG_IRQ_IDLE	 				BIT(2)
#define PN5180_REG_IRQ_TX	 				BIT(1)
#define PN5180_REG_IRQ_RX	 				BIT(0)

#define PN5180_REG_CRC_RX_CONFIG_ENABLE				BIT(0)

#define PN5180_REG_CRC_TX_CONFIG_ENABLE				BIT(0)

#define PN5180_MODE_STANDBY					0x00U /* 3 config bytes */
#define PN5180_MODE_LPCD					0x01U /* 2 config bytes */
#define PN5180_MODE_AUTOCOLL					0x02U /* 2 config bytes */
#define PN5180_MODE_NORMAL					0x03U /* 0 config bytes */

#define PN5180_EEPROM_ADDR_UID					0x00U /* 16 bytes */
#define PN5180_EEPROM_ADDR_PRODUCT_VERSION			0x10U /*  2 bytes */
#define PN5180_EEPROM_ADDR_FIRMWARE_VERSION			0x12U /*  2 bytes */
#define PN5180_EEPROM_ADDR_EEPROM_VERSION			0x14U /*  2 bytes */
#define PN5180_EEPROM_ADDR_IDLE_IRQ_AFTER_BOOT			0x16U /*  1 byte  */
#define PN5180_EEPROM_ADDR_TESTBUS_ENABLE			0x17U /*  1 byte  */
#define PN5180_EEPROM_ADDR_XTAL_BOOT_TIME			0x18U /*  2 bytes */
#define PN5180_EEPROM_ADDR_IRQ_PIN_CONFIG			0x1AU /*  1 byte  */
#define PN5180_EEPROM_ADDR_MISO_PULLUP_ENABLE			0x1BU /*  1 byte  */
#define PN5180_EEPROM_ADDR_PLL_DEFAULT_SETTING			0x1CU /*  8 bytes */
#define PN5180_EEPROM_ADDR_PLL_DEFAULT_SETTING_ALM		0x24U /*  8 bytes */
#define PN5180_EEPROM_ADDR_PLL_LOCK_SETTING			0x2CU /*  4 bytes */
#define PN5180_EEPROM_ADDR_CLOCK_CONFIG				0x30U /*  1 byte  */
#define PN5180_EEPROM_ADDR_MFC_AUTH_TIMEOUT			0x32U /*  2 bytes */
#define PN5180_EEPROM_ADDR_LPCD_REFERENCE_VALUE			0x34U /*  1 byte  */
#define PN5180_EEPROM_ADDR_LPCD_FIELD_ON_TIME			0x36U /*  1 byte  */
#define PN5180_EEPROM_ADDR_LPCD_THRESHOLD			0x37U /*  1 byte  */
#define PN5180_EEPROM_ADDR_LPCD_REFVAL_GPO_CONTROL		0x38U /*  1 byte  */
#define PN5180_EEPROM_ADDR_LPCD_GPO_TOGGLE_BEFORE_FIELD_ON	0x39U /*  1 byte  */
#define PN5180_EEPROM_ADDR_LPCD_GPO_TOGGLE_AFTER_FIELD_OFF	0x3AU /*  1 byte  */
#define PN5180_EEPROM_ADDR_NFCLD_SENSITIVITY_VAL		0x3BU /*  1 byte  */
#define PN5180_EEPROM_ADDR_FIELD_ON_CP_SETTLE_TIME		0x3CU /*  1 byte  */
#define PN5180_EEPROM_ADDR_RF_DEBOUNCE_TIMEOUT			0x3FU /*  1 byte  */
#define PN5180_EEPROM_ADDR_SENS_RES				0x40U /*  2 bytes */
#define PN5180_EEPROM_ADDR_NFCID1				0x42U /*  3 bytes */
#define PN5180_EEPROM_ADDR_SEL_RES				0x45U /*  1 byte  */
#define PN5180_EEPROM_ADDR_FELICA_POLLING_RESPONSE		0x46U /* 18 bytes */
#define PN5180_EEPROM_ADDR_RANDOM_UID_ENABLE_PRE_401		0x51U /*  1 byte  */
#define PN5180_EEPROM_ADDR_RANDOM_UID_ENABLE			0x58U /*  1 byte  */
#define PN5180_EEPROM_ADDR_DPC_CONTROL				0x59U /*  1 byte  */
#define PN5180_EEPROM_ADDR_DPC_TIME				0x5AU /*  2 bytes */
#define PN5180_EEPROM_ADDR_DPC_XI				0x5CU /*  1 byte  */
#define PN5180_EEPROM_ADDR_AGC_CONTROL				0x5DU /*  2 bytes */
// TODO: add missing addresses
#define PN5180_EEPROM_ADDR_MISC_CONFIG				0xE8U /*  1 byte  */

#define PN5180_RF_ON_RFCA					BIT(0)
#define PN5180_RF_ON_ACTIVE_MODE				BIT(1)

#define PN5180_RF_TX_NONE					0xFFU
#define PN5180_RF_TX_ISO14443A_106_MILLER			0x00U
#define PN5180_RF_TX_ISO14443A_212_MILLER			0x01U
#define PN5180_RF_TX_ISO14443A_424_MILLER			0x02U
#define PN5180_RF_TX_ISO14443A_848_MILLER			0x03U
#define PN5180_RF_TX_ISO14443B_106_NRZ				0x04U
#define PN5180_RF_TX_ISO14443B_212_NRZ				0x05U
#define PN5180_RF_TX_ISO14443B_424_NRZ				0x06U
#define PN5180_RF_TX_ISO14443B_848_NRZ				0x07U
#define PN5180_RF_TX_FELICA_212					0x08U
#define PN5180_RF_TX_FELICA_424					0x09U
#define PN5180_RF_TX_NFC_AI_106_106				0x0AU
#define PN5180_RF_TX_NFC_AI_212_212				0x0BU
#define PN5180_RF_TX_NFC_AI_424_424				0x0CU
#define PN5180_RF_TX_ISO15693_26_1OF4_ASK100			0x0DU
#define PN5180_RF_TX_ISO15693_26_1OF4_ASK10			0x0EU
#define PN5180_RF_TX_ISO180003M3_TARI_18_88_ASK			0x0FU
#define PN5180_RF_TX_ISO180003M3_TARI_9_44_ASK			0x10U
#define PN5180_RF_TX_ISO14443A_PICC_106_MANCH_SUBC		0x13U
#define PN5180_RF_TX_ISO14443A_PICC_212_BPSK			0x14U
#define PN5180_RF_TX_ISO14443A_PICC_424_BPSK			0x15U
#define PN5180_RF_TX_ISO14443A_PICC_848_BPSK			0x16U
#define PN5180_RF_TX_NFC_PT_212					0x17U
#define PN5180_RF_TX_NFC_PT_424					0x18U
#define PN5180_RF_TX_NFC_AT_106					0x19U
#define PN5180_RF_TX_NFC_AT_212					0x1AU
#define PN5180_RF_TX_NFC_AT_424					0x1BU
#define PN5180_RF_TX_GTM_ALL_AL					0x1CU

#define PN5180_RF_RX_NONE					0xFFU
#define PN5180_RF_RX_ISO14443A_106_MANCH_SUBC			0x80U
#define PN5180_RF_RX_ISO14443A_212_BPSK				0x81U
#define PN5180_RF_RX_ISO14443A_424_BPSK				0x82U
#define PN5180_RF_RX_ISO14443A_848_BPSK				0x83U
#define PN5180_RF_RX_ISO14443B_106_BPSK				0x84U
#define PN5180_RF_RX_ISO14443B_212_BPSK				0x85U
#define PN5180_RF_RX_ISO14443B_424_BPSK				0x86U
#define PN5180_RF_RX_ISO14443B_848_BPSK				0x87U
#define PN5180_RF_RX_FELICA_212					0x88U
#define PN5180_RF_RX_FELICA_424					0x89U
#define PN5180_RF_RX_NFC_AI_106					0x8AU
#define PN5180_RF_RX_NFC_AI_212					0x8BU
#define PN5180_RF_RX_NFC_AI_424					0x8CU
#define PN5180_RF_RX_ISO15693_26_1OF4_SC			0x8DU
#define PN5180_RF_RX_ISO15693_53_1OF4_SC			0x8EU
#define PN5180_RF_RX_ISO180003M3_MANCH424_4_PERIOD		0x8FU
#define PN5180_RF_RX_ISO180003M3_MANCH424_2_PERIOD		0x90U
#define PN5180_RF_RX_ISO180003M3_MANCH848_4_PERIOD		0x91U
#define PN5180_RF_RX_ISO180003M3_MANCH848_2_PERIOD		0x92U
#define PN5180_RF_RX_ISO14443A_PICC_106_MILLER			0x93U
#define PN5180_RF_RX_ISO14443A_PICC_212_MILLER			0x94U
#define PN5180_RF_RX_ISO14443A_PICC_424_MILLER			0x95U
#define PN5180_RF_RX_ISO14443A_PICC_848_MILLER			0x96U
#define PN5180_RF_RX_NFC_PT_212					0x97U
#define PN5180_RF_RX_NFC_PT_424					0x98U
#define PN5180_RF_RX_NFC_AT_106					0x99U
#define PN5180_RF_RX_NFC_AT_212					0x9AU
#define PN5180_RF_RX_NFC_AT_424					0x9BU
#define PN5180_RF_RX_GTM_ALL_ALL				0x9CU

#define PN5180_TRANSCEIVE_STATE_IDLE				0x00U
#define PN5180_TRANSCEIVE_STATE_WAIT_TRANSMIT			0x01U
#define PN5180_TRANSCEIVE_STATE_TRANSMITTING			0x02U
#define PN5180_TRANSCEIVE_STATE_WAIT_RECEIVE			0x03U
#define PN5180_TRANSCEIVE_STATE_WAIT_FOR_DATA			0x04U
#define PN5180_TRANSCEIVE_STATE_RECEIVING			0x05U
#define PN5180_TRANSCEIVE_STATE_LOOPBACK			0x06U
#define PN5180_TRANSCEIVE_STATE_RESERVED			0x07U

#define PN5180_AUTOCOLL_TECH_NFC_F_ACTIVE			BIT(3)
#define PN5180_AUTOCOLL_TECH_NFC_A_ACTIVE			BIT(2)
#define PN5180_AUTOCOLL_TECH_NFC_F_PASSIVE			BIT(1)
#define PN5180_AUTOCOLL_TECH_NFC_A_PASSIVE			BIT(0)

#define PN5180_AUTOCOLL_MODE_NONE				0x00U
#define PN5180_AUTOCOLL_MODE_AUTO_STANDBY			0x01U
#define PN5180_AUTOCOLL_MODE_AUTO_NO_STANDBY			0x02U

#ifdef __cplusplus
extern "C" {
#endif

union pn5180_system_config {
	struct {
		uint32_t command:3;
		uint32_t start_send:1;
		uint32_t reserved0:1;
		uint32_t prbs_type:1;
		uint32_t mfc_crypto_on:1;
		uint32_t reserved1:1;
		uint32_t soft_reset:1;
		uint32_t autocoll_picc_state:1;
		uint32_t active_mode_tx_rf_enable:1;
		uint32_t ldo_out_enable:1;
		uint32_t dpc_xi_ram_correction:8;
		uint32_t reserved2:12;
	} __packed;
	uint32_t raw;
};

union pn5180_wait_config {
	struct {
		uint32_t wait_prescaler:8;
		uint32_t wait_value:20;
		uint32_t reserved:4;

	} __packed;
	uint32_t raw;
};

union pn5180_crc_rx_config {
	struct {
		uint32_t rx_crc_enable:1;
		uint32_t rx_crc_inv:1;
		uint32_t rx_crc_type:1;
		uint32_t rx_crc_preset_sel:3;
		uint32_t rx_bit_align:3;
		uint32_t values_after_collision:1;
		uint32_t rx_parity_enable:1;
		uint32_t rx_parity_type:1;
		uint32_t reserved:4;
		uint32_t rx_crc_preset_value:16;
	} __packed;
	uint32_t raw;
};

union pn5180_crc_tx_config {
	struct {
		uint32_t tx_crc_enable:1;
		uint32_t tx_crc_inv:1;
		uint32_t tx_crc_type:1;
		uint32_t tx_crc_preset_sel:3;
		uint32_t tx_crc_byte2_enable:1;
		uint32_t reserved:9;
		uint32_t tx_crc_preset_value:16;
	} __packed;
	uint32_t raw;
};

union pn5180_rx_status {
	struct {
		uint32_t rx_num_bytes_received:9;
		uint32_t rx_num_frames_received:4;
		uint32_t rx_num_last_bits:3;
		uint32_t rx_data_integrity_error:1;
		uint32_t rx_protocol_error:1;
		uint32_t rx_collision_detected:1;
		uint32_t rx_coll_pos:7;
		uint32_t reserved:6;
	} __packed;
	uint32_t raw;
};

union pn5180_rf_status {
	struct {
		uint32_t agc_value:10;
		uint32_t rx_active:1;
		uint32_t tx_active:1;
		uint32_t rx_enable:1;
		uint32_t rf_active_error_cause:3;
		uint32_t rf_det_status:1;
		uint32_t tx_rf_status:1;
		uint32_t crc_ok:1;
		uint32_t dpll_enable:1;
		uint32_t dpc_current_gear:4;
		uint32_t transceive_state:3;
		uint32_t reserved:5;
	} __packed;
	uint32_t raw;
};

union pn5180_timer_config {
	struct {
		uint32_t enable:1;
		uint32_t reload_enable:1;
		uint32_t mode_sel:1;
		uint32_t prescale_sel:3;
		uint32_t start_now:1;
		uint32_t start_on_rf_off_int:1;
		uint32_t start_on_rf_on_int:1;
		uint32_t start_on_rf_off_ext:1;
		uint32_t start_on_rf_on_ext:1;
		uint32_t start_on_tx_ended:1;
		uint32_t start_on_tx_started:1;
		uint32_t start_on_rx_ended:1;
		uint32_t start_on_rx_started:1;
		uint32_t stop_on_rf_off_int:1;
		uint32_t stop_on_rf_on_int:1;
		uint32_t stop_on_rf_off_ext:1;
		uint32_t stop_on_rf_on_ext:1;
		uint32_t stop_on_tx_started:1;
		uint32_t stop_on_rx_started:1;
		uint32_t reserved:11;
	} __packed;
	uint32_t raw;
};

union pn5180_emd_control {
	struct {
		uint32_t enable:1;
		uint32_t transmission_error:1;
		uint32_t noise_bytes_threshold:4;
		uint32_t missing_crc_error_type_a:1;
		uint32_t missing_crc_error_type_b:1;
		uint32_t transmission_timer_used:2;
		uint32_t reserved:22;
	} __packed;
	uint32_t raw;
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_NFC_PN5180_H_ */
