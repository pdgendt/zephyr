/*
 * Copyright (c) 2025 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/nvmem.h>
#include <zephyr/sys/__assert.h>

int nvmem_cell_read(const struct nvmem_cell *cell, void *buf, off_t off, size_t len)
{
	__ASSERT_NO_MSG(cell != NULL);

	if (off < 0 || cell->size < off + len) {
		return -EINVAL;
	}

	if (IS_ENABLED(CONFIG_NVMEM_EEPROM) && DEVICE_API_IS(eeprom, cell->dev)) {
		return eeprom_read(cell->dev, cell->offset + off, buf, len);
	}

	if (IS_ENABLED(CONFIG_NVMEM_SYSCON) && DEVICE_API_IS(syscon, cell->dev)) {
		int width = syscon_get_reg_width(cell->dev);
		uint8_t *data = buf;
		uint32_t val;
		int ret;

		if (width <= 0) {
			return width < 0 ? width : -EFAULT;
		}

		while (len > 0) {
			ret = syscon_read_reg(cell->dev, cell->offset + off++, &val);
			if (ret < 0) {
				return ret;
			}

			for (int i = 0; i < width && len > 0; ++i) {
				*data++ = val & 0xff;
				val >>= 8;
				--len;
			}
		}
	}

	return -ENXIO;
}

int nvmem_cell_write(const struct nvmem_cell *cell, const void *buf, off_t off, size_t len)
{
	__ASSERT_NO_MSG(cell != NULL);

	if (off < 0 || cell->size < off + len) {
		return -EINVAL;
	}

	if (cell->read_only) {
		return -EROFS;
	}

	if (IS_ENABLED(CONFIG_NVMEM_EEPROM) && DEVICE_API_IS(eeprom, cell->dev)) {
		return eeprom_write(cell->dev, cell->offset + off, buf, len);
	}

	if (IS_ENABLED(CONFIG_NVMEM_SYSCON) && DEVICE_API_IS(syscon, cell->dev)) {
		int width = syscon_get_reg_width(cell->dev);
		const uint8_t *data = buf;
		uint32_t val;
		int ret;

		if (width <= 0) {
			return width < 0 ? width : -EFAULT;
		}

		while (len > 0) {
			val = 0;

			for (int i = 0; i < width && len > 0; ++i) {
				val <<= 8;
				val |= *data++;
				--len;
			}

			ret = syscon_write_reg(cell->dev, cell->offset + off++, val);
			if (ret < 0) {
				return ret;
			}
		}
	}

	return -ENXIO;
}
