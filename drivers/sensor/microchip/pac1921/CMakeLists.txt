# Copyright (c) 2025 Basalte bv
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(pac1921.c)
zephyr_library_sources_ifdef(CONFIG_SENSOR_ASYNC_API pac1921_decoder.c)
