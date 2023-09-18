
/*
 * Copyright (c) 2023 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/shell/shell.h>
#include <zephyr/timing/timing.h>

#define SHELL_HELP_TIME		"Run commands and summarize duration."

static timing_t start_time;

bool z_shell_time_request(const char *str)
{
	return strcmp("time", str) == 0;
}

void z_shell_time_start(const struct shell *sh)
{
	ARG_UNUSED(sh);

	timing_init();
	timing_start();

	start_time = timing_counter_get();
}

void z_shell_time_stop(const struct shell *sh)
{
	timing_t end_time;
	uint64_t total_cycles;

	end_time = timing_counter_get();
	timing_stop();

	total_cycles = timing_cycles_get(&start_time, &end_time);

	shell_info(sh, "\nduration %lluns\n", timing_cycles_to_ns(total_cycles));
}

/* Empty shell command */
SHELL_CMD_REGISTER(time, NULL, SHELL_HELP_TIME, NULL);
