/*
 * Copyright (c) 2011-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SYS___ASSERT_H_
#define ZEPHYR_INCLUDE_SYS___ASSERT_H_

#include <stdarg.h>
#include <stdbool.h>
#include <zephyr/toolchain.h>

/*
 * __ASSERT_ON resolution, in priority order:
 *   1. CONFIG_FORCE_NO_ASSERT forces 0 (overrides everything, including an
 *      externally-defined __ASSERT_ON).
 *   2. An externally-defined __ASSERT_ON is honored.
 *   3. CONFIG_ASSERT + CONFIG_ASSERT_LEVEL: use the configured level.
 *   4. Default to 0.
 */
#if defined(CONFIG_FORCE_NO_ASSERT)
#undef __ASSERT_ON
#define __ASSERT_ON 0
#elif defined(__ASSERT_ON)
/* external __ASSERT_ON: leave as-is */
#elif defined(CONFIG_ASSERT) && defined(CONFIG_ASSERT_LEVEL)
#define __ASSERT_ON CONFIG_ASSERT_LEVEL
#else
#define __ASSERT_ON 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Wrapper around printk to avoid including printk.h in assert.h */
void __printf_like(1, 2) assert_print(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#if defined(CONFIG_ASSERT_VERBOSE)
#define __ASSERT_PRINT(fmt, ...) assert_print(fmt, ##__VA_ARGS__)
#else /* CONFIG_ASSERT_VERBOSE */
#define __ASSERT_PRINT(fmt, ...)
#endif /* CONFIG_ASSERT_VERBOSE */

#ifdef CONFIG_ASSERT_NO_MSG_INFO
#define __ASSERT_MSG_INFO(fmt, ...)
#else  /* CONFIG_ASSERT_NO_MSG_INFO */
#define __ASSERT_MSG_INFO(fmt, ...) __ASSERT_PRINT("\t" fmt "\n", ##__VA_ARGS__)
#endif /* CONFIG_ASSERT_NO_MSG_INFO */

#if !defined(CONFIG_ASSERT_NO_COND_INFO) && !defined(CONFIG_ASSERT_NO_FILE_INFO)
#define __ASSERT_LOC(test)                                                     \
	__ASSERT_PRINT("ASSERTION FAIL [%s] @ %s:%d\n",                        \
		       Z_STRINGIFY(test), __FILE__, __LINE__)
#elif defined(CONFIG_ASSERT_NO_COND_INFO) && !defined(CONFIG_ASSERT_NO_FILE_INFO)
#define __ASSERT_LOC(test)                                                     \
	__ASSERT_PRINT("ASSERTION FAIL @ %s:%d\n", __FILE__, __LINE__)
#elif !defined(CONFIG_ASSERT_NO_COND_INFO) && defined(CONFIG_ASSERT_NO_FILE_INFO)
#define __ASSERT_LOC(test)                                                     \
	__ASSERT_PRINT("ASSERTION FAIL [%s]\n", Z_STRINGIFY(test))
#else
#define __ASSERT_LOC(test)                                                     \
	__ASSERT_PRINT("ASSERTION FAIL\n")
#endif

#if (__ASSERT_ON < 0) || (__ASSERT_ON > 2)
#error "Invalid __ASSERT() level: must be between 0 and 2"
#endif

#if __ASSERT_ON

/*
 * Internal: route the failure path through a single out-of-line handler
 * (z_assert_handler) when the default config applies — i.e. verbose output
 * with both file and condition info. The NO_FILE_INFO / NO_COND_INFO knobs
 * are about stripping per-call-site emissions, which the legacy per-piece
 * expansion does naturally; the handler path is the optimization for the
 * common case.
 */
#if defined(CONFIG_ASSERT_VERBOSE) &&                                          \
	!defined(CONFIG_ASSERT_NO_COND_INFO) &&                                \
	!defined(CONFIG_ASSERT_NO_FILE_INFO)
#define _Z_ASSERT_USE_HANDLER 1
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Override point: called once per failing assertion, before
 * assert_post_action(). Default implementation prints the location via
 * assert_print() and the user message via vprintk(). System designers may
 * override this to capture the assertion output to a buffer or redirect it
 * elsewhere.
 */
void assert_pre_action(const char *cond, const char *file, unsigned int line,
		       const char *fmt, va_list ap);

#ifdef CONFIG_ASSERT_NO_FILE_INFO
void assert_post_action(void);
#define __ASSERT_POST_ACTION() assert_post_action()
#else  /* CONFIG_ASSERT_NO_FILE_INFO */
void assert_post_action(const char *file, unsigned int line);
#define __ASSERT_POST_ACTION() assert_post_action(__FILE__, __LINE__)
#endif /* CONFIG_ASSERT_NO_FILE_INFO */

#ifdef _Z_ASSERT_USE_HANDLER
/*
 * Out-of-line failure handler. Calls assert_pre_action() and
 * assert_post_action() in turn. A single call here replaces the three
 * emissions (location print, message print, post_action) the legacy
 * expansion produces at each __ASSERT() call site, shrinking the cold path.
 */
void __printf_like(4, 5) z_assert_handler(const char *cond, const char *file,
					  unsigned int line, const char *fmt, ...);
#endif

/*
 * When the assert test mode is enabled, the default kernel fatal error handler
 * and the custom assert hook function may return in order to allow the test to
 * proceed.
 */
#ifdef CONFIG_ASSERT_TEST
#define __ASSERT_UNREACHABLE
#else
#define __ASSERT_UNREACHABLE CODE_UNREACHABLE
#endif

#ifdef __cplusplus
}
#endif

/*
 * Internal: the per-assert failure body. __ASSERT() and __ASSERT_NO_MSG()
 * call into these so the user-facing macros have a single definition; the
 * body switches between an out-of-line handler and a per-piece expansion
 * depending on what assert info is enabled.
 */
#ifdef _Z_ASSERT_USE_HANDLER

#define _Z_ASSERT_FAIL_NO_MSG(test)                                            \
	z_assert_handler(Z_STRINGIFY(test), __FILE__, __LINE__, NULL)

#define _Z_ASSERT_FAIL(test, fmt, ...)                                         \
	z_assert_handler(Z_STRINGIFY(test), __FILE__, __LINE__,                \
			 "\t" fmt "\n", ##__VA_ARGS__)

#else /* per-piece expansion */

#define _Z_ASSERT_FAIL_NO_MSG(test)                                            \
	do {                                                                   \
		__ASSERT_LOC(test);                                            \
		__ASSERT_POST_ACTION();                                        \
	} while (false)

#define _Z_ASSERT_FAIL(test, fmt, ...)                                         \
	do {                                                                   \
		__ASSERT_LOC(test);                                            \
		__ASSERT_MSG_INFO(fmt, ##__VA_ARGS__);                         \
		__ASSERT_POST_ACTION();                                        \
	} while (false)

#endif

#define __ASSERT_NO_MSG(test)                                                  \
	do {                                                                   \
		if (!(test)) {                                                 \
			_Z_ASSERT_FAIL_NO_MSG(test);                           \
			__ASSERT_UNREACHABLE;                                  \
		}                                                              \
	} while (false)

#define __ASSERT(test, fmt, ...)                                               \
	do {                                                                   \
		if (!(test)) {                                                 \
			_Z_ASSERT_FAIL(test, fmt, ##__VA_ARGS__);              \
			__ASSERT_UNREACHABLE;                                  \
		}                                                              \
	} while (false)

#define __ASSERT_EVAL(expr1, expr2, test, fmt, ...)                \
	do {                                                       \
		expr2;                                             \
		__ASSERT(test, fmt, ##__VA_ARGS__);                \
	} while (false)

#if (__ASSERT_ON == 1)
#warning "__ASSERT() statements are ENABLED"
#endif

#else /* __ASSERT_ON == 0 */

#define __ASSERT(test, fmt, ...) { }
#define __ASSERT_EVAL(expr1, expr2, test, fmt, ...) expr1
#define __ASSERT_NO_MSG(test) { }
#define __ASSERT_POST_ACTION() { }

#endif /* __ASSERT_ON */

#ifdef CONFIG_ASSERT_CUSTOM_HEADER
/* This include must always be at the end of __assert.h */
#include <zephyr_custom_assert.h>
#endif

#endif /* ZEPHYR_INCLUDE_SYS___ASSERT_H_ */
