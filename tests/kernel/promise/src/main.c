/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>

#define STACK_SIZE     (512 + CONFIG_TEST_EXTRA_STACK_SIZE)
#define THREAD_PRIO    0                /* preemptive — runs when test thread pends */
#define SETTLE_DELAY   K_MSEC(20)       /* helper delay before settling */
#define WAIT_TIMEOUT   K_MSEC(500)      /* generous timeout for threaded waits */

K_THREAD_STACK_DEFINE(helper_stack, STACK_SIZE);
static struct k_thread helper_thread;

/* =========================================================================
 * Helper threads used by wait tests
 * ========================================================================= */

struct helper_arg {
	struct k_future  *f;
	struct k_promise *p;
	void             *value;
	int               error;
};

static void thread_resolve(void *a, void *b, void *c)
{
	struct helper_arg *arg = a;

	ARG_UNUSED(b);
	ARG_UNUSED(c);

	k_sleep(SETTLE_DELAY);
	k_promise_resolve(arg->p, arg->value);
}

static void thread_reject(void *a, void *b, void *c)
{
	struct helper_arg *arg = a;

	ARG_UNUSED(b);
	ARG_UNUSED(c);

	k_sleep(SETTLE_DELAY);
	k_promise_reject(arg->p, arg->error);
}

static void thread_cancel(void *a, void *b, void *c)
{
	struct helper_arg *arg = a;

	ARG_UNUSED(b);
	ARG_UNUSED(c);

	k_sleep(SETTLE_DELAY);
	k_future_cancel(arg->f);
}

/* =========================================================================
 * Continuation callbacks
 * ========================================================================= */

/* Increments the atomic_t pointed to by ctx. */
static void cb_count(struct k_future *f, void *ctx)
{
	ARG_UNUSED(f);
	atomic_inc((atomic_t *)ctx);
}

/* Stores the settled future pointer into the struct k_future ** pointed to by ctx. */
static void cb_capture(struct k_future *f, void *ctx)
{
	*(struct k_future **)ctx = f;
}

/* Resolves the next promise (ctx) with the source future's value — chain. */
static void cb_chain(struct k_future *f, void *ctx)
{
	k_promise_resolve((struct k_promise *)ctx, k_future_get_value(f));
}

/* Resolves the next promise (ctx) recovering from a rejection — catch. */
static void cb_catch(struct k_future *f, void *ctx)
{
	/* Recover: turn the negative error into a positive value. */
	k_promise_resolve((struct k_promise *)ctx,
			  (void *)(uintptr_t)(-k_future_get_error(f)));
}

/* =========================================================================
 * Suite declaration
 * ========================================================================= */

ZTEST_SUITE(promise, NULL, NULL, NULL, NULL, NULL);

/* =========================================================================
 * Basic state-machine tests (single-threaded)
 * ========================================================================= */

ZTEST(promise, test_resolve_basic)
{
	struct k_future f;
	struct k_promise p;
	void *val = (void *)0xdeadbeef;

	k_future_init(&f);
	k_promise_init(&p, &f);

	zassert_true(k_future_is_pending(&f));

	zassert_equal(k_promise_resolve(&p, val), 0);

	zassert_true(k_future_is_resolved(&f));
	zassert_false(k_future_is_pending(&f));
	zassert_false(k_future_is_rejected(&f));
	zassert_false(k_future_is_canceled(&f));
	zassert_equal(k_future_get_value(&f), val);
}

ZTEST(promise, test_reject_basic)
{
	struct k_future f;
	struct k_promise p;

	k_future_init(&f);
	k_promise_init(&p, &f);

	zassert_equal(k_promise_reject(&p, -ENODEV), 0);

	zassert_true(k_future_is_rejected(&f));
	zassert_false(k_future_is_resolved(&f));
	zassert_equal(k_future_get_error(&f), -ENODEV);
}

ZTEST(promise, test_cancel_basic)
{
	struct k_future f;
	struct k_promise p;

	k_future_init(&f);
	k_promise_init(&p, &f);

	zassert_equal(k_future_cancel(&f), 0);

	zassert_true(k_future_is_canceled(&f));
	zassert_false(k_future_is_pending(&f));
}

ZTEST(promise, test_resolve_twice)
{
	struct k_future f;
	struct k_promise p;

	k_future_init(&f);
	k_promise_init(&p, &f);

	zassert_equal(k_promise_resolve(&p, NULL), 0);
	zassert_equal(k_promise_resolve(&p, NULL), -EALREADY);
}

ZTEST(promise, test_reject_twice)
{
	struct k_future f;
	struct k_promise p;

	k_future_init(&f);
	k_promise_init(&p, &f);

	zassert_equal(k_promise_reject(&p, -EIO), 0);
	zassert_equal(k_promise_reject(&p, -EIO), -EALREADY);
}

ZTEST(promise, test_cancel_twice)
{
	struct k_future f;
	struct k_promise p;

	k_future_init(&f);
	k_promise_init(&p, &f);

	zassert_equal(k_future_cancel(&f), 0);
	zassert_equal(k_future_cancel(&f), -EALREADY);
}

ZTEST(promise, test_cancel_then_resolve)
{
	struct k_future f;
	struct k_promise p;

	k_future_init(&f);
	k_promise_init(&p, &f);

	zassert_equal(k_future_cancel(&f), 0);
	zassert_equal(k_promise_resolve(&p, NULL), -ECANCELED);

	/* State must still be CANCELED, not RESOLVED. */
	zassert_true(k_future_is_canceled(&f));
}

ZTEST(promise, test_cancel_then_reject)
{
	struct k_future f;
	struct k_promise p;

	k_future_init(&f);
	k_promise_init(&p, &f);

	zassert_equal(k_future_cancel(&f), 0);
	zassert_equal(k_promise_reject(&p, -EIO), -ECANCELED);

	zassert_true(k_future_is_canceled(&f));
}

ZTEST(promise, test_reject_nonnegative_error)
{
	struct k_future f;
	struct k_promise p;

	k_future_init(&f);
	k_promise_init(&p, &f);

	/* error must be strictly negative */
	zassert_equal(k_promise_reject(&p, 0), -EINVAL);
	zassert_equal(k_promise_reject(&p, 1), -EINVAL);

	/* Future must still be pending after the invalid calls. */
	zassert_true(k_future_is_pending(&f));
}

ZTEST(promise, test_promise_is_canceled)
{
	struct k_future f;
	struct k_promise p;

	k_future_init(&f);
	k_promise_init(&p, &f);

	zassert_false(k_promise_is_canceled(&p));

	k_future_cancel(&f);

	zassert_true(k_promise_is_canceled(&p));
}

/* =========================================================================
 * k_promise_settle and result macro tests
 * ========================================================================= */

ZTEST(promise, test_settle_resolves)
{
	struct k_future f;
	struct k_promise p;
	void *val = (void *)0x1234;
	struct k_future_result res = K_FUTURE_RESULT_RESOLVE(val);

	k_future_init(&f);
	k_promise_init(&p, &f);

	zassert_equal(k_promise_settle(&p, &res), 0);

	zassert_true(k_future_is_resolved(&f));
	zassert_equal(k_future_get_value(&f), val);
}

ZTEST(promise, test_settle_rejects)
{
	struct k_future f;
	struct k_promise p;
	struct k_future_result res = K_FUTURE_RESULT_REJECT(-ENODEV);

	k_future_init(&f);
	k_promise_init(&p, &f);

	zassert_equal(k_promise_settle(&p, &res), 0);

	zassert_true(k_future_is_rejected(&f));
	zassert_equal(k_future_get_error(&f), -ENODEV);
}

ZTEST(promise, test_result_macros)
{
	struct k_future_result resolve = K_FUTURE_RESULT_RESOLVE((void *)0xBEEF);
	struct k_future_result reject  = K_FUTURE_RESULT_REJECT(-ETIMEDOUT);

	zassert_equal(resolve.status, 0);
	zassert_equal(resolve.value, (void *)0xBEEF);

	zassert_equal(reject.status, -ETIMEDOUT);
}

ZTEST(promise, test_get_result)
{
	struct k_future f;
	struct k_promise p;
	void *val = (void *)0xABCD;

	k_future_init(&f);
	k_promise_init(&p, &f);
	k_promise_settle(&p, &(struct k_future_result)K_FUTURE_RESULT_RESOLVE(val));

	struct k_future_result got = k_future_get_result(&f);

	zassert_equal(got.status, 0);
	zassert_equal(got.value, val);
}

ZTEST(promise, test_cancel_without_promise)
{
	struct k_future f;

	/* Never bind a promise — cancel must still work. */
	k_future_init(&f);

	zassert_true(k_future_is_pending(&f));
	zassert_equal(k_future_cancel(&f), 0);
	zassert_true(k_future_is_canceled(&f));

	/* Second cancel must return -EALREADY. */
	zassert_equal(k_future_cancel(&f), -EALREADY);
}

/* =========================================================================
 * Static initializer
 * ========================================================================= */

K_FUTURE_DEFINE(static_future);

ZTEST(promise, test_static_define)
{
	struct k_promise p;

	/* Z_FUTURE_INITIALIZER sets state to PENDING. */
	zassert_true(k_future_is_pending(&static_future));

	k_promise_init(&p, &static_future);
	zassert_equal(k_promise_resolve(&p, (void *)0xCAFE), 0);
	zassert_true(k_future_is_resolved(&static_future));
	zassert_equal(k_future_get_value(&static_future), (void *)0xCAFE);
}

/* =========================================================================
 * Wait tests
 * ========================================================================= */

ZTEST(promise, test_wait_already_resolved)
{
	struct k_future f;
	struct k_promise p;

	k_future_init(&f);
	k_promise_init(&p, &f);
	k_promise_resolve(&p, (void *)0xAA);

	/* Already settled: must return immediately even with K_NO_WAIT. */
	zassert_equal(k_future_wait(&f, K_NO_WAIT), 0);
	zassert_true(k_future_is_resolved(&f));
}

ZTEST(promise, test_wait_already_canceled)
{
	struct k_future f;
	struct k_promise p;

	k_future_init(&f);
	k_promise_init(&p, &f);
	k_future_cancel(&f);

	zassert_equal(k_future_wait(&f, K_NO_WAIT), 0);
	zassert_true(k_future_is_canceled(&f));
}

ZTEST(promise, test_wait_nowait_pending)
{
	struct k_future f;
	struct k_promise p;

	k_future_init(&f);
	k_promise_init(&p, &f);

	/* Nothing settles it: K_NO_WAIT must return -EAGAIN. */
	zassert_equal(k_future_wait(&f, K_NO_WAIT), -EAGAIN);
	zassert_true(k_future_is_pending(&f));
}

ZTEST(promise, test_wait_thread_resolves)
{
	struct k_future f;
	struct k_promise p;
	struct helper_arg arg = { .f = &f, .p = &p, .value = (void *)0xBEEF };

	k_future_init(&f);
	k_promise_init(&p, &f);

	k_thread_create(&helper_thread, helper_stack, STACK_SIZE,
			thread_resolve, &arg, NULL, NULL,
			THREAD_PRIO, 0, K_NO_WAIT);

	zassert_equal(k_future_wait(&f, WAIT_TIMEOUT), 0);
	zassert_true(k_future_is_resolved(&f));
	zassert_equal(k_future_get_value(&f), (void *)0xBEEF);

	k_thread_join(&helper_thread, K_FOREVER);
}

ZTEST(promise, test_wait_thread_rejects)
{
	struct k_future f;
	struct k_promise p;
	struct helper_arg arg = { .f = &f, .p = &p, .error = -ENODEV };

	k_future_init(&f);
	k_promise_init(&p, &f);

	k_thread_create(&helper_thread, helper_stack, STACK_SIZE,
			thread_reject, &arg, NULL, NULL,
			THREAD_PRIO, 0, K_NO_WAIT);

	zassert_equal(k_future_wait(&f, WAIT_TIMEOUT), 0);
	zassert_true(k_future_is_rejected(&f));
	zassert_equal(k_future_get_error(&f), -ENODEV);

	k_thread_join(&helper_thread, K_FOREVER);
}

ZTEST(promise, test_wait_thread_cancels)
{
	struct k_future f;
	struct k_promise p;
	struct helper_arg arg = { .f = &f, .p = &p };

	k_future_init(&f);
	k_promise_init(&p, &f);

	k_thread_create(&helper_thread, helper_stack, STACK_SIZE,
			thread_cancel, &arg, NULL, NULL,
			THREAD_PRIO, 0, K_NO_WAIT);

	zassert_equal(k_future_wait(&f, WAIT_TIMEOUT), 0);
	zassert_true(k_future_is_canceled(&f));

	k_thread_join(&helper_thread, K_FOREVER);
}

ZTEST(promise, test_wait_timeout)
{
	struct k_future f;
	struct k_promise p;

	k_future_init(&f);
	k_promise_init(&p, &f);

	/* Nobody settles the future — wait must time out. */
	zassert_equal(k_future_wait(&f, K_MSEC(20)), -EAGAIN);
	zassert_true(k_future_is_pending(&f));
}

/* =========================================================================
 * Continuation tests
 * ========================================================================= */

ZTEST(promise, test_cont_fires_on_resolve)
{
	struct k_future f;
	struct k_promise p;
	atomic_t count;
	struct k_future_cont cont = {
		.cb      = cb_count,
		.ctx     = &count,
		.trigger = K_FUTURE_ON_RESOLVED,
	};

	atomic_set(&count, 0);
	k_future_init(&f);
	k_promise_init(&p, &f);
	k_future_then(&f, &cont);

	k_promise_resolve(&p, NULL);

	zassert_equal(atomic_get(&count), 1);
}

ZTEST(promise, test_cont_fires_on_reject)
{
	struct k_future f;
	struct k_promise p;
	atomic_t count;
	struct k_future_cont cont = {
		.cb      = cb_count,
		.ctx     = &count,
		.trigger = K_FUTURE_ON_REJECTED,
	};

	atomic_set(&count, 0);
	k_future_init(&f);
	k_promise_init(&p, &f);
	k_future_then(&f, &cont);

	k_promise_reject(&p, -ENODEV);

	zassert_equal(atomic_get(&count), 1);
}

ZTEST(promise, test_cont_fires_on_cancel)
{
	struct k_future f;
	struct k_promise p;
	atomic_t count;
	struct k_future_cont cont = {
		.cb      = cb_count,
		.ctx     = &count,
		.trigger = K_FUTURE_ON_CANCELED,
	};

	atomic_set(&count, 0);
	k_future_init(&f);
	k_promise_init(&p, &f);
	k_future_then(&f, &cont);

	k_future_cancel(&f);

	zassert_equal(atomic_get(&count), 1);
}

ZTEST(promise, test_cont_settled_fires_on_resolve_and_reject)
{
	struct k_future f1, f2;
	struct k_promise p1, p2;
	atomic_t c1, c2;
	struct k_future_cont cont1 = {
		.cb = cb_count, .ctx = &c1, .trigger = K_FUTURE_ON_SETTLED,
	};
	struct k_future_cont cont2 = {
		.cb = cb_count, .ctx = &c2, .trigger = K_FUTURE_ON_SETTLED,
	};

	atomic_set(&c1, 0);
	atomic_set(&c2, 0);

	k_future_init(&f1);
	k_promise_init(&p1, &f1);
	k_future_then(&f1, &cont1);

	k_future_init(&f2);
	k_promise_init(&p2, &f2);
	k_future_then(&f2, &cont2);

	k_promise_resolve(&p1, NULL);
	k_promise_reject(&p2, -EIO);

	zassert_equal(atomic_get(&c1), 1);
	zassert_equal(atomic_get(&c2), 1);
}

ZTEST(promise, test_cont_trigger_filter)
{
	struct k_future f;
	struct k_promise p;
	atomic_t count;
	struct k_future_cont cont = {
		/* Only fires on resolve; future is rejected below. */
		.cb      = cb_count,
		.ctx     = &count,
		.trigger = K_FUTURE_ON_RESOLVED,
	};

	atomic_set(&count, 0);
	k_future_init(&f);
	k_promise_init(&p, &f);
	k_future_then(&f, &cont);

	k_promise_reject(&p, -EIO);

	/* Trigger did not match: callback must not have been called. */
	zassert_equal(atomic_get(&count), 0);
}

ZTEST(promise, test_cont_receives_correct_future)
{
	struct k_future f;
	struct k_promise p;
	struct k_future *captured = NULL;
	struct k_future_cont cont = {
		.cb      = cb_capture,
		.ctx     = &captured,
		.trigger = K_FUTURE_ON_ANY,
	};

	k_future_init(&f);
	k_promise_init(&p, &f);
	k_future_then(&f, &cont);

	k_promise_resolve(&p, NULL);

	zassert_equal(captured, &f);
}

ZTEST(promise, test_cont_fires_immediately_when_already_resolved)
{
	struct k_future f;
	struct k_promise p;
	atomic_t count;
	struct k_future_cont cont = {
		.cb      = cb_count,
		.ctx     = &count,
		.trigger = K_FUTURE_ON_RESOLVED,
	};

	atomic_set(&count, 0);
	k_future_init(&f);
	k_promise_init(&p, &f);

	k_promise_resolve(&p, NULL);

	/* Register after settle: must fire synchronously before returning. */
	k_future_then(&f, &cont);

	zassert_equal(atomic_get(&count), 1);
}

ZTEST(promise, test_cont_fires_immediately_when_already_canceled)
{
	struct k_future f;
	struct k_promise p;
	atomic_t count;
	struct k_future_cont cont = {
		.cb      = cb_count,
		.ctx     = &count,
		.trigger = K_FUTURE_ON_CANCELED,
	};

	atomic_set(&count, 0);
	k_future_init(&f);
	k_promise_init(&p, &f);
	k_future_cancel(&f);

	k_future_then(&f, &cont);

	zassert_equal(atomic_get(&count), 1);
}

/* =========================================================================
 * k_future_catch / k_future_finally
 * ========================================================================= */

ZTEST(promise, test_catch_fires_on_reject)
{
	struct k_future f;
	struct k_promise p;
	atomic_t count;
	struct k_future_cont cont = { .cb = cb_count, .ctx = &count };

	atomic_set(&count, 0);
	k_future_init(&f);
	k_promise_init(&p, &f);

	/* k_future_catch sets trigger = ON_REJECTED */
	k_future_catch(&f, &cont);
	k_promise_reject(&p, -EIO);

	zassert_equal(atomic_get(&count), 1);
}

ZTEST(promise, test_catch_not_fired_on_resolve)
{
	struct k_future f;
	struct k_promise p;
	atomic_t count;
	struct k_future_cont cont = { .cb = cb_count, .ctx = &count };

	atomic_set(&count, 0);
	k_future_init(&f);
	k_promise_init(&p, &f);
	k_future_catch(&f, &cont);

	k_promise_resolve(&p, NULL);

	zassert_equal(atomic_get(&count), 0);
}

ZTEST(promise, test_finally_fires_on_resolve)
{
	struct k_future f;
	struct k_promise p;
	atomic_t count;
	struct k_future_cont cont = { .cb = cb_count, .ctx = &count };

	atomic_set(&count, 0);
	k_future_init(&f);
	k_promise_init(&p, &f);

	/* k_future_finally sets trigger = ON_ANY */
	k_future_finally(&f, &cont);
	k_promise_resolve(&p, NULL);

	zassert_equal(atomic_get(&count), 1);
}

ZTEST(promise, test_finally_fires_on_reject)
{
	struct k_future f;
	struct k_promise p;
	atomic_t count;
	struct k_future_cont cont = { .cb = cb_count, .ctx = &count };

	atomic_set(&count, 0);
	k_future_init(&f);
	k_promise_init(&p, &f);
	k_future_finally(&f, &cont);

	k_promise_reject(&p, -EIO);

	zassert_equal(atomic_get(&count), 1);
}

ZTEST(promise, test_finally_fires_on_cancel)
{
	struct k_future f;
	struct k_promise p;
	atomic_t count;
	struct k_future_cont cont = { .cb = cb_count, .ctx = &count };

	atomic_set(&count, 0);
	k_future_init(&f);
	k_promise_init(&p, &f);
	k_future_finally(&f, &cont);

	k_future_cancel(&f);

	zassert_equal(atomic_get(&count), 1);
}

/* =========================================================================
 * Cancel propagation via k_future_link_cancel
 * ========================================================================= */

ZTEST(promise, test_cancel_propagate)
{
	struct k_future fA, fB;
	struct k_promise pA, pB;
	struct k_future_cancel_link link;

	k_future_init(&fA);
	k_promise_init(&pA, &fA);
	k_future_init(&fB);
	k_promise_init(&pB, &fB);

	k_future_link_cancel(&fA, &fB, &link);

	zassert_false(k_future_is_canceled(&fB));

	k_future_cancel(&fA);

	zassert_true(k_future_is_canceled(&fA));
	zassert_true(k_future_is_canceled(&fB));
}

ZTEST(promise, test_cancel_propagate_not_triggered_on_resolve)
{
	struct k_future fA, fB;
	struct k_promise pA, pB;
	struct k_future_cancel_link link;

	k_future_init(&fA);
	k_promise_init(&pA, &fA);
	k_future_init(&fB);
	k_promise_init(&pB, &fB);

	k_future_link_cancel(&fA, &fB, &link);

	k_promise_resolve(&pA, NULL);

	/* fA resolved, not canceled: fB must remain pending. */
	zassert_true(k_future_is_pending(&fB));
}

ZTEST(promise, test_cancel_propagate_chain)
{
	struct k_future fA, fB, fC;
	struct k_promise pA, pB, pC;
	struct k_future_cancel_link linkAB, linkBC;

	k_future_init(&fA);
	k_promise_init(&pA, &fA);
	k_future_init(&fB);
	k_promise_init(&pB, &fB);
	k_future_init(&fC);
	k_promise_init(&pC, &fC);

	k_future_link_cancel(&fA, &fB, &linkAB);
	k_future_link_cancel(&fB, &fC, &linkBC);

	k_future_cancel(&fA);

	/* Canceling fA must cascade to fB and then to fC. */
	zassert_true(k_future_is_canceled(&fA));
	zassert_true(k_future_is_canceled(&fB));
	zassert_true(k_future_is_canceled(&fC));
}

ZTEST(promise, test_cancel_link_already_canceled_fires_immediately)
{
	struct k_future fA, fB;
	struct k_promise pA, pB;
	struct k_future_cancel_link link;

	k_future_init(&fA);
	k_promise_init(&pA, &fA);
	k_future_init(&fB);
	k_promise_init(&pB, &fB);

	/* Cancel fA first, then register the link — must fire immediately. */
	k_future_cancel(&fA);

	zassert_false(k_future_is_canceled(&fB));

	k_future_link_cancel(&fA, &fB, &link);

	zassert_true(k_future_is_canceled(&fB));
}

/* =========================================================================
 * Chaining and catch
 * ========================================================================= */

ZTEST(promise, test_chain_resolve)
{
	struct k_future fA, fB;
	struct k_promise pA, pB;
	void *val = (void *)0x5678;
	struct k_future_cont cont = {
		.cb      = cb_chain,
		.ctx     = &pB,
		.trigger = K_FUTURE_ON_RESOLVED,
	};

	k_future_init(&fA);
	k_promise_init(&pA, &fA);
	k_future_init(&fB);
	k_promise_init(&pB, &fB);

	k_future_then(&fA, &cont);

	k_promise_resolve(&pA, val);

	/* Continuation must have resolved fB with the same value. */
	zassert_true(k_future_is_resolved(&fB));
	zassert_equal(k_future_get_value(&fB), val);
}

ZTEST(promise, test_catch_reject)
{
	struct k_future fA, fB;
	struct k_promise pA, pB;
	struct k_future_cont cont = {
		.cb      = cb_catch,
		.ctx     = &pB,
		.trigger = K_FUTURE_ON_REJECTED,
	};

	k_future_init(&fA);
	k_promise_init(&pA, &fA);
	k_future_init(&fB);
	k_promise_init(&pB, &fB);

	k_future_then(&fA, &cont);

	k_promise_reject(&pA, -ENODEV);

	/* cb_catch recovers from rejection by resolving fB with ENODEV. */
	zassert_true(k_future_is_resolved(&fB));
	zassert_equal(k_future_get_value(&fB), (void *)(uintptr_t)ENODEV);
}

ZTEST(promise, test_chain_reject_not_caught)
{
	struct k_future fA, fB;
	struct k_promise pA, pB;
	struct k_future_cont cont = {
		/* Only handles resolve; rejected fA must leave fB pending. */
		.cb      = cb_chain,
		.ctx     = &pB,
		.trigger = K_FUTURE_ON_RESOLVED,
	};

	k_future_init(&fA);
	k_promise_init(&pA, &fA);
	k_future_init(&fB);
	k_promise_init(&pB, &fB);

	k_future_then(&fA, &cont);

	k_promise_reject(&pA, -EIO);

	zassert_true(k_future_is_pending(&fB));
}
