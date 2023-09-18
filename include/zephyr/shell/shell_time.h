
/*
 * Copyright (c) 2023 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SHELL_TIME_H__
#define SHELL_TIME_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct shell;

#ifdef CONFIG_SHELL_TIME

bool z_shell_time_request(const char *str);

void z_shell_time_start(const struct shell *sh);

void z_shell_time_stop(const struct shell *sh);

#else /* CONFIG_SHELL_TIME */

#define z_shell_time_request(...) false

#define z_shell_time_start(...)

#define z_shell_time_stop(...)

#endif /* CONFIG_SHELL_TIME */

#ifdef __cplusplus
}
#endif

#endif /* SHELL_TIME_H__ */
