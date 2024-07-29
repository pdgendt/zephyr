/*
 * Copyright (c) 2024 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/ztest.h>

ZTEST(test_kconfig_functions, test_substring)
{
	const char *sym_string = CONFIG_SYM_STRING;

	zassert_str_equal(sym_string + 5, CONFIG_SYM_SUBSTRING_5);
	zassert_mem_equal(sym_string + 10, CONFIG_SYM_SUBSTRING_10_15, 5);
}

ZTEST_SUITE(test_kconfig_functions, NULL, NULL, NULL, NULL, NULL);
