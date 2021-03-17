/* main.c - OpenThread Host */

/*
 * Copyright (c) 2021 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(ot_host, LOG_LEVEL_DBG);

#include <zephyr.h>
#define APP_BANNER "***** OpenThread Host on Zephyr *****"

void main(void)
{
	LOG_INF(APP_BANNER);
}
