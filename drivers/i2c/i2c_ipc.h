/*
 * SPDX-FileCopyrightText: 2026 Basalte bv
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_I2C_I2C_IPC_H_
#define ZEPHYR_DRIVERS_I2C_I2C_IPC_H_

#include <stdint.h>

/**
 * @brief I2C IPC proxy wire protocol definitions.
 *
 * Shared between the client (virtual I2C controller) and
 * server (forwards to real I2C controller) drivers.
 *
 * All multi-byte fields are little-endian on the wire.
 *
 * Request  (client -> server): [cmd:u8][payload...]
 * Response (server -> client): [cmd:u8][ret:i32le][payload...]
 */

enum i2c_ipc_cmd {
	I2C_IPC_CMD_CONFIGURE = 0x01,
	I2C_IPC_CMD_GET_CONFIG = 0x02,
	I2C_IPC_CMD_TRANSFER = 0x03,
};

/* Request: CONFIGURE */
struct i2c_ipc_configure_req {
	uint8_t cmd;
	uint32_t dev_config;
} __packed;

/* Response: CONFIGURE */
struct i2c_ipc_configure_rsp {
	uint8_t cmd;
	int32_t ret;
} __packed;

/* Request: GET_CONFIG (no payload beyond cmd) */
struct i2c_ipc_get_config_req {
	uint8_t cmd;
} __packed;

/* Response: GET_CONFIG */
struct i2c_ipc_get_config_rsp {
	uint8_t cmd;
	int32_t ret;
	uint32_t dev_config;
} __packed;

/*
 * Request: TRANSFER
 *   [cmd:u8][addr:u16le][num_msgs:u8][desc0..N][write_data...]
 *
 * Each descriptor:
 *   [flags:u8][len:u16le]
 *
 * write_data: concatenated buffers for messages without I2C_MSG_READ flag.
 */
struct i2c_ipc_transfer_hdr {
	uint8_t cmd;
	uint16_t addr;
	uint8_t num_msgs;
} __packed;

struct i2c_ipc_msg_desc {
	uint8_t flags;
	uint16_t len;
} __packed;

/*
 * Response: TRANSFER
 *   [cmd:u8][ret:i32le][read_data...]
 *
 * read_data: concatenated buffers for messages with I2C_MSG_READ flag.
 */
struct i2c_ipc_transfer_rsp {
	uint8_t cmd;
	int32_t ret;
} __packed;

#endif /* ZEPHYR_DRIVERS_I2C_I2C_IPC_H_ */
