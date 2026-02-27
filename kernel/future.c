/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/kernel_structs.h>
#include <zephyr/toolchain.h>
#include <zephyr/sys/check.h>
#include <ksched.h>
#include <wait_q.h>

void k_future_init(struct k_future *f)
{
	__ASSERT_NO_MSG(!arch_is_in_isr());

	atomic_set(&f->state, K_FUTURE_PENDING);
	z_waitq_init(&f->wait_q);
	f->lock = (struct k_spinlock){};
	f->error = 0;
	f->value = NULL;
	f->out = NULL;

	k_object_init(f);
}

static int future_settle(struct k_future *f, int new_state, void *value, int error)
{
	struct k_thread *thread;
	k_spinlock_key_t key = k_spin_lock(&f->lock);

	if (atomic_get(&f->state) != K_FUTURE_PENDING) {
		k_spin_unlock(&f->lock, key);
		return -EALREADY;
	}

	f->value = value;
	f->error = error;
	atomic_set(&f->state, new_state);

	if (new_state == K_FUTURE_RESOLVED && f->out != NULL) {
		*f->out = value;
		f->out = NULL;
	} else {
		f->out = NULL;
	}

	while ((thread = z_unpend_first_thread(&f->wait_q)) != NULL) {
		arch_thread_return_value_set(thread, 0);
		z_ready_thread(thread);
	}

	z_reschedule(&f->lock, key);

	return 0;
}

int k_promise_resolve(struct k_promise *p, void *value)
{
	return future_settle(p->future, K_FUTURE_RESOLVED, value, 0);
}

int k_promise_reject(struct k_promise *p, int error)
{
	CHECKIF(error >= 0) {
		return -EINVAL;
	}

	return future_settle(p->future, K_FUTURE_REJECTED, NULL, error);
}

int k_future_cancel(struct k_future *f)
{
	__ASSERT_NO_MSG(!arch_is_in_isr());

	return future_settle(f, K_FUTURE_CANCELED, NULL, 0);
}

int k_future_wait(struct k_future *f, void **out, k_timeout_t timeout)
{
	__ASSERT_NO_MSG(!arch_is_in_isr());

	k_spinlock_key_t key = k_spin_lock(&f->lock);

	if (atomic_get(&f->state) != K_FUTURE_PENDING) {
		if (out != NULL && atomic_get(&f->state) == K_FUTURE_RESOLVED) {
			*out = f->value;
		}
		k_spin_unlock(&f->lock, key);
		return 0;
	}

	f->out = out;
	int ret = z_pend_curr(&f->lock, key, &f->wait_q, timeout);

	if (ret == -EAGAIN) {
		/*
		 * Timed out. Re-acquire the lock to safely inspect and clear
		 * f->out before this thread's stack slot becomes invalid.
		 *
		 * If the future settled concurrently with the timeout (i.e.
		 * future_settle already wrote *f->out and cleared f->out),
		 * the state is no longer PENDING and we treat the wait as
		 * successful rather than returning -EAGAIN.
		 */
		k_spinlock_key_t key2 = k_spin_lock(&f->lock);

		if (atomic_get(&f->state) != K_FUTURE_PENDING) {
			ret = 0;
		} else {
			f->out = NULL;
		}

		k_spin_unlock(&f->lock, key2);
	}

	return ret;
}
