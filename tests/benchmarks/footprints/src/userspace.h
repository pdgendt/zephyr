/*
 * Copyright (c) 2020 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _USERSPACE_H_
#define _USERSPACE_H_

__syscall int dummy_syscall(void);
__syscall int validation_overhead_syscall(void);

#ifndef __ZPP__
#include <zephyr/syscalls/userspace.h>
#endif

#endif /* _USERSPACE_H_ */
