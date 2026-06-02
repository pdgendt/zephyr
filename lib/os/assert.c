/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/__assert.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/llext/symbol.h>

/**
 * @brief Assert Action Handler
 *
 * This routine implements the action to be taken when an assertion fails.
 *
 * System designers may wish to substitute this implementation to take other
 * actions, such as logging program counter, line number, debug information
 * to a persistent repository and/or rebooting the system.
 *
 * @param N/A
 */
#ifdef CONFIG_ASSERT_NO_FILE_INFO
__weak void assert_post_action(void)
#else
__weak void assert_post_action(const char *file, unsigned int line)
#endif
{
#ifndef CONFIG_ASSERT_NO_FILE_INFO
	ARG_UNUSED(file);
	ARG_UNUSED(line);
#endif

#ifdef CONFIG_USERSPACE
	/* User threads aren't allowed to induce kernel panics; generate
	 * an oops instead.
	 */
	if (k_is_user_context()) {
		k_oops();
	}
#endif

	k_panic();
}
EXPORT_SYMBOL(assert_post_action);

/**
 * @brief Assert Pre Action Handler
 *
 * Override point for the assertion output. Called once per failing assertion,
 * before assert_post_action(). The default implementation prints the
 * assertion location via assert_print() and the user message via vprintk().
 *
 * System designers may wish to substitute this implementation to capture the
 * assertion output to a buffer, redirect it elsewhere, etc.
 */
__weak void assert_pre_action(const char *cond, const char *file,
			      unsigned int line, const char *fmt, va_list ap)
{
	assert_print("ASSERTION FAIL [%s] @ %s:%d\n", cond, file, line);

	if (fmt != NULL) {
		vprintk(fmt, ap);
	}
}
EXPORT_SYMBOL(assert_pre_action);

/**
 * @brief Assert Print Handler
 *
 * This routine implements printing the assertion message.
 *
 * System designers may wish to substitute this implementation to store the
 * assertion, or otherwise change the behavior of the assertion print
 *
 * @param N/A
 */
__weak void assert_print(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);

	vprintk(fmt, ap);

	va_end(ap);
}
EXPORT_SYMBOL(assert_print);
