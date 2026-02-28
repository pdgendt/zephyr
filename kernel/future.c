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

void k_promise_init(struct k_promise *p, void **out)
{
	__ASSERT_NO_MSG(!arch_is_in_isr());

	atomic_set(&p->state, K_FUTURE_PENDING);
	z_waitq_init(&p->wait_q);
	p->lock = (struct k_spinlock){};
	p->error = 0;
	p->value = NULL;
	p->out = out;

	k_object_init(p);
}

int k_promise_resolve(struct k_promise *p, void *value)
{
	struct k_thread *thread;
	k_spinlock_key_t key = k_spin_lock(&p->lock);
	int state = atomic_get(&p->state);

	if (state == K_FUTURE_CANCELED) {
		k_spin_unlock(&p->lock, key);
		return -ECANCELED;
	}

	if (state != K_FUTURE_PENDING) {
		k_spin_unlock(&p->lock, key);
		return -EALREADY;
	}

	p->value = value;
	atomic_set(&p->state, K_FUTURE_RESOLVED);

	if (p->out != NULL) {
		*p->out = value;
	}

	while ((thread = z_unpend_first_thread(&p->wait_q)) != NULL) {
		arch_thread_return_value_set(thread, 0);
		z_ready_thread(thread);
	}

	z_reschedule(&p->lock, key);

	return 0;
}

int k_promise_reject(struct k_promise *p, int error)
{
	struct k_thread *thread;

	CHECKIF(error >= 0) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&p->lock);
	int state = atomic_get(&p->state);

	if (state == K_FUTURE_CANCELED) {
		k_spin_unlock(&p->lock, key);
		return -ECANCELED;
	}

	if (state != K_FUTURE_PENDING) {
		k_spin_unlock(&p->lock, key);
		return -EALREADY;
	}

	p->error = error;
	atomic_set(&p->state, K_FUTURE_REJECTED);

	while ((thread = z_unpend_first_thread(&p->wait_q)) != NULL) {
		arch_thread_return_value_set(thread, 0);
		z_ready_thread(thread);
	}

	z_reschedule(&p->lock, key);

	return 0;
}

int k_future_cancel(struct k_future *f)
{
	__ASSERT_NO_MSG(!arch_is_in_isr());
	__ASSERT_NO_MSG(f->promise != NULL);

	struct k_promise *p = f->promise;
	struct k_thread *thread;
	k_spinlock_key_t key = k_spin_lock(&p->lock);

	if (atomic_get(&p->state) != K_FUTURE_PENDING) {
		k_spin_unlock(&p->lock, key);
		return -EALREADY;
	}

	atomic_set(&p->state, K_FUTURE_CANCELED);

	while ((thread = z_unpend_first_thread(&p->wait_q)) != NULL) {
		arch_thread_return_value_set(thread, 0);
		z_ready_thread(thread);
	}

	z_reschedule(&p->lock, key);

	return 0;
}

int k_future_wait(struct k_future *f, k_timeout_t timeout)
{
	__ASSERT_NO_MSG(!arch_is_in_isr());
	__ASSERT_NO_MSG(f->promise != NULL);

	struct k_promise *p = f->promise;
	k_spinlock_key_t key = k_spin_lock(&p->lock);

	if (atomic_get(&p->state) != K_FUTURE_PENDING) {
		k_spin_unlock(&p->lock, key);
		return 0;
	}

	int ret = z_pend_curr(&p->lock, key, &p->wait_q, timeout);

	if (ret == -EAGAIN) {
		/*
		 * Timed out. Re-acquire the lock to safely inspect the state.
		 * If the promise settled concurrently with the timeout, treat
		 * the wait as successful rather than returning -EAGAIN.
		 */
		k_spinlock_key_t key2 = k_spin_lock(&p->lock);

		if (atomic_get(&p->state) != K_FUTURE_PENDING) {
			ret = 0;
		}

		k_spin_unlock(&p->lock, key2);
	}

	return ret;
}
