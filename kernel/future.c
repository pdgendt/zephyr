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
	f->result = (struct k_future_result){0, NULL};
	f->promise = NULL;
	f->cont = NULL;

	k_object_init(f);
}

void k_promise_init(struct k_promise *p, struct k_future *f)
{
	__ASSERT_NO_MSG(!arch_is_in_isr());
	__ASSERT_NO_MSG(f != NULL);

	p->lock = (struct k_spinlock){};
	p->future = f;
	atomic_set(&p->canceled, 0);
	f->promise = p;
}

/* --- continuation helpers ------------------------------------------------ */

static uint8_t state_to_trigger(int state)
{
	switch (state) {
	case K_FUTURE_RESOLVED:
		return K_FUTURE_ON_RESOLVED;
	case K_FUTURE_REJECTED:
		return K_FUTURE_ON_REJECTED;
	case K_FUTURE_CANCELED:
		return K_FUTURE_ON_CANCELED;
	default:
		return 0;
	}
}

static void direct_fire(struct k_future_cont *cont, struct k_future *f)
{
	int state = atomic_get(&f->state);

	if (cont->cb != NULL && (cont->trigger & state_to_trigger(state))) {
		cont->cb(f, cont->ctx);
	}

	if (state == K_FUTURE_CANCELED && cont->cancel_propagate != NULL) {
		k_future_cancel(cont->cancel_propagate);
	}
}

/*
 * Common registration helper.  The caller must have already set cont->_fire.
 * If the future is already settled the continuation is fired immediately,
 * synchronously, in the caller's thread context.
 */
static void register_cont(struct k_future *f, struct k_future_cont *cont)
{
	k_spinlock_key_t key = k_spin_lock(&f->lock);
	int state = atomic_get(&f->state);

	if (state == K_FUTURE_PENDING) {
		__ASSERT_NO_MSG(f->cont == NULL);
		f->cont = cont;
		k_spin_unlock(&f->lock, key);
	} else {
		k_spin_unlock(&f->lock, key);
		cont->_fire(cont, f);
	}
}

void k_future_then(struct k_future *f, struct k_future_cont *cont)
{
	cont->_fire = direct_fire;
	register_cont(f, cont);
}

/* --- settle helpers ------------------------------------------------------- */

/*
 * All three settle paths (settle, reject, cancel) acquire p->lock before
 * f->lock when a promise is linked (lock ordering).  This prevents deadlock
 * between concurrent resolve and cancel.
 *
 * p->future is NULLed under p->lock by all three paths so the producer never
 * dereferences a future that may have gone out of scope.
 *
 * The continuation (if any) is extracted under f->lock so it fires exactly
 * once, then called after all locks are released so the callback can itself
 * call k_promise_settle() / k_future_cancel() on any other future without
 * re-entering any held lock.
 */

int k_promise_settle(struct k_promise *p, const struct k_future_result *result)
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
	 * guard against double-settle.
	 */
	if (atomic_get(&f->state) != K_FUTURE_PENDING) {
		k_spin_unlock(&f->lock, fk);
		k_spin_unlock(&p->lock, pk);
		return -EALREADY;
	}

	f->result = *result;
	atomic_set(&f->state, result->status < 0 ? K_FUTURE_REJECTED : K_FUTURE_RESOLVED);
	p->future = NULL; /* sever link */

	struct k_future_cont *cont = f->cont;

	f->cont = NULL;

	struct k_thread *thread;

	while ((thread = z_unpend_first_thread(&f->wait_q)) != NULL) {
		arch_thread_return_value_set(thread, 0);
		z_ready_thread(thread);
	}

	k_spin_unlock(&f->lock, fk);
	z_reschedule(&p->lock, pk);

	if (cont != NULL) {
		cont->_fire(cont, f);
	}

	return 0;
}

int k_promise_reject(struct k_promise *p, int error)
{
	CHECKIF(error >= 0) {
		return -EINVAL;
	}

	return k_promise_settle(p, &(struct k_future_result){.status = error});
}

int k_future_cancel(struct k_future *f)
{
	__ASSERT_NO_MSG(!arch_is_in_isr());

	struct k_promise *p = f->promise;

	if (p == NULL) {
		/*
		 * No promise linked — future was either never bound or has
		 * already been settled and the link severed.  Cancel directly
		 * under f->lock alone (no lock-ordering concern).
		 */
		k_spinlock_key_t fk = k_spin_lock(&f->lock);

		if (atomic_get(&f->state) != K_FUTURE_PENDING) {
			k_spin_unlock(&f->lock, fk);
			return -EALREADY;
		}

		atomic_set(&f->state, K_FUTURE_CANCELED);

		struct k_future_cont *cont = f->cont;

		f->cont = NULL;

		struct k_thread *thread;

		while ((thread = z_unpend_first_thread(&f->wait_q)) != NULL) {
			arch_thread_return_value_set(thread, 0);
			z_ready_thread(thread);
		}

		z_reschedule(&f->lock, fk);

		if (cont != NULL) {
			cont->_fire(cont, f);
		}

		return 0;
	}

	/*
	 * Acquire p->lock first, then f->lock — same order as settle,
	 * preventing deadlock.  Nulling p->future under p->lock guarantees
	 * that once this function returns, any concurrent or future settle
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

	struct k_future_cont *cont = f->cont;

	f->cont = NULL;

	struct k_thread *thread;

	while ((thread = z_unpend_first_thread(&f->wait_q)) != NULL) {
		arch_thread_return_value_set(thread, 0);
		z_ready_thread(thread);
	}

	k_spin_unlock(&f->lock, fk);
	z_reschedule(&p->lock, pk);

	if (cont != NULL) {
		cont->_fire(cont, f);
	}

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
