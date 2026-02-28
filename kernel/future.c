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

void k_future_init(struct k_future *f, void **out)
{
	__ASSERT_NO_MSG(!arch_is_in_isr());

	atomic_set(&f->state, K_FUTURE_PENDING);
	z_waitq_init(&f->wait_q);
	f->lock = (struct k_spinlock){};
	f->error = 0;
	f->value = NULL;
	f->out = out;
	f->promise = NULL;

	k_object_init(f);
}

/*
 * Both resolve and reject acquire p->lock before f->lock (lock ordering).
 * cancel acquires p->lock then f->lock in the same order, preventing deadlock.
 *
 * p->future is NULLed under p->lock by all three paths so the producer never
 * dereferences a future that may have gone out of scope.
 */

int k_promise_resolve(struct k_promise *p, void *value)
{
	k_spinlock_key_t pk = k_spin_lock(&p->lock);
	struct k_future *f = p->future;

	if (f == NULL) {
		k_spin_unlock(&p->lock, pk);
		return atomic_get(&p->canceled) ? -ECANCELED : -EALREADY;
	}

	k_spinlock_key_t fk = k_spin_lock(&f->lock);

	/*
	 * We hold p->lock, so k_future_cancel() cannot be running concurrently
	 * (it acquires p->lock first).  The state must still be PENDING, but
	 * check for a double-resolve anyway.
	 */
	if (atomic_get(&f->state) != K_FUTURE_PENDING) {
		k_spin_unlock(&f->lock, fk);
		k_spin_unlock(&p->lock, pk);
		return -EALREADY;
	}

	f->value = value;
	atomic_set(&f->state, K_FUTURE_RESOLVED);
	if (f->out != NULL) {
		*f->out = value;
	}
	p->future = NULL; /* sever link — future may go out of scope after consumer wakes */

	struct k_thread *thread;

	while ((thread = z_unpend_first_thread(&f->wait_q)) != NULL) {
		arch_thread_return_value_set(thread, 0);
		z_ready_thread(thread);
	}

	k_spin_unlock(&f->lock, fk);
	z_reschedule(&p->lock, pk);

	return 0;
}

int k_promise_reject(struct k_promise *p, int error)
{
	CHECKIF(error >= 0) {
		return -EINVAL;
	}

	k_spinlock_key_t pk = k_spin_lock(&p->lock);
	struct k_future *f = p->future;

	if (f == NULL) {
		k_spin_unlock(&p->lock, pk);
		return atomic_get(&p->canceled) ? -ECANCELED : -EALREADY;
	}

	k_spinlock_key_t fk = k_spin_lock(&f->lock);

	if (atomic_get(&f->state) != K_FUTURE_PENDING) {
		k_spin_unlock(&f->lock, fk);
		k_spin_unlock(&p->lock, pk);
		return -EALREADY;
	}

	f->error = error;
	atomic_set(&f->state, K_FUTURE_REJECTED);
	p->future = NULL; /* sever link */

	struct k_thread *thread;

	while ((thread = z_unpend_first_thread(&f->wait_q)) != NULL) {
		arch_thread_return_value_set(thread, 0);
		z_ready_thread(thread);
	}

	k_spin_unlock(&f->lock, fk);
	z_reschedule(&p->lock, pk);

	return 0;
}

int k_future_cancel(struct k_future *f)
{
	__ASSERT_NO_MSG(!arch_is_in_isr());
	__ASSERT_NO_MSG(f->promise != NULL);

	struct k_promise *p = f->promise;

	/*
	 * Acquire p->lock first, then f->lock — same order as resolve/reject,
	 * preventing deadlock.  Nulling p->future under p->lock guarantees that
	 * once this function returns, any concurrent or future resolve/reject
	 * call will see NULL and return -ECANCELED without touching f.
	 */
	k_spinlock_key_t pk = k_spin_lock(&p->lock);
	k_spinlock_key_t fk = k_spin_lock(&f->lock);

	if (atomic_get(&f->state) != K_FUTURE_PENDING) {
		k_spin_unlock(&f->lock, fk);
		k_spin_unlock(&p->lock, pk);
		return -EALREADY;
	}

	atomic_set(&f->state, K_FUTURE_CANCELED);
	atomic_set(&p->canceled, 1);
	p->future = NULL; /* sever link — f may go out of scope after we return */

	struct k_thread *thread;

	while ((thread = z_unpend_first_thread(&f->wait_q)) != NULL) {
		arch_thread_return_value_set(thread, 0);
		z_ready_thread(thread);
	}

	k_spin_unlock(&f->lock, fk);
	z_reschedule(&p->lock, pk);

	return 0;
}

int k_future_wait(struct k_future *f, k_timeout_t timeout)
{
	__ASSERT_NO_MSG(!arch_is_in_isr());

	k_spinlock_key_t key = k_spin_lock(&f->lock);

	if (atomic_get(&f->state) != K_FUTURE_PENDING) {
		k_spin_unlock(&f->lock, key);
		return 0;
	}

	int ret = z_pend_curr(&f->lock, key, &f->wait_q, timeout);

	if (ret == -EAGAIN) {
		/*
		 * Timed out. Re-acquire the lock to safely inspect the state.
		 * If the future settled concurrently with the timeout, treat
		 * the wait as successful rather than returning -EAGAIN.
		 */
		k_spinlock_key_t key2 = k_spin_lock(&f->lock);

		if (atomic_get(&f->state) != K_FUTURE_PENDING) {
			ret = 0;
		}

		k_spin_unlock(&f->lock, key2);
	}

	return ret;
}
