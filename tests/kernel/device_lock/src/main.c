/*
 * SPDX-FileCopyrightText: 2026 Basalte bv
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/device_lock.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

/* Two fake device instances defined in app.overlay. */
#define LOCKED_DEV   DEVICE_DT_GET(DT_NODELABEL(fakelockdriver_locked))
#define UNLOCKED_DEV DEVICE_DT_GET(DT_NODELABEL(fakelockdriver_unlocked))

DEVICE_DT_DEFINE(DT_NODELABEL(fakelockdriver_locked), NULL, NULL, NULL, NULL,
		 POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);
DEVICE_DT_DEFINE(DT_NODELABEL(fakelockdriver_unlocked), NULL, NULL, NULL, NULL,
		 POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);

/* -------------------------------------------------------------------------- */
/* Helpers for the mutual-exclusion test                                       */
/* -------------------------------------------------------------------------- */

#define STACK_SIZE 1024

static K_THREAD_STACK_DEFINE(thread_a_stack, STACK_SIZE);
static K_THREAD_STACK_DEFINE(thread_b_stack, STACK_SIZE);
static struct k_thread thread_a_data;
static struct k_thread thread_b_data;

/*
 * Semaphore used to synchronise the two threads: A posts it after acquiring
 * the device lock so that B knows it is safe to attempt its own acquisition.
 */
static K_SEM_DEFINE(sync_sem, 0, 1);

/*
 * Monotonically increasing counter written under the device lock. After both
 * threads complete it must equal 2.
 */
static atomic_t lock_sequence;

/*
 * thread_a acquires the device lock, records sequence step 1, then yields
 * long enough for thread_b to attempt its own acquisition.  It records
 * sequence step 2 *before* releasing the lock so that thread_b can verify the
 * ordering upon entry.
 */
static void thread_a_fn(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;

	zassert_ok(device_take(dev, K_FOREVER));

	atomic_set(&lock_sequence, 1);
	k_sem_give(&sync_sem); /* tell B that A now owns the lock */

	/* Hold the lock long enough that B is guaranteed to be waiting. */
	k_sleep(K_MSEC(20));

	/* Set step 2 while still holding the lock. */
	atomic_set(&lock_sequence, 2);

	zassert_ok(device_give(dev));
}

/*
 * thread_b waits until A has acquired the lock, then tries to acquire it
 * itself.  The acquisition must block until A releases.  Once B enters its
 * critical section the sequence counter must already be 2.
 */
static void thread_b_fn(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;

	/* Wait until thread_a holds the lock. */
	zassert_ok(k_sem_take(&sync_sem, K_FOREVER));

	/* sequence is 1 here — A is inside its critical section. */
	zassert_equal(atomic_get(&lock_sequence), 1);

	/* Validate that we can't take the device right away */
	zassert_equal(device_take(dev, K_NO_WAIT), -EBUSY);

	/* This call blocks until thread_a releases the lock. */
	zassert_ok(device_take(dev, K_FOREVER));

	/* A set sequence to 2 before releasing, so we must see 2. */
	zassert_equal(atomic_get(&lock_sequence), 2);

	zassert_ok(device_give(dev));
}


ZTEST(device_lock, test_has_api_lock)
{
	zassert_equal(device_has_api_lock(LOCKED_DEV), IS_ENABLED(CONFIG_DEVICE_API_LOCK));
	zassert_false(device_has_api_lock(UNLOCKED_DEV));
}

ZTEST(device_lock, test_take_give_locked)
{
	const struct device *dev = LOCKED_DEV;

	zassert_ok(device_take(dev, K_FOREVER));
	zassert_ok(device_give(dev));
}

ZTEST(device_lock, test_take_give_unlocked)
{
	const struct device *dev = UNLOCKED_DEV;

	zassert_ok(device_take(dev, K_FOREVER));
	zassert_ok(device_give(dev));
}

ZTEST(device_lock, test_recursive_lock)
{
	const struct device *dev = LOCKED_DEV;

	zassert_ok(device_take(dev, K_FOREVER));
	zassert_ok(device_take(dev, K_FOREVER)); /* recursive — must not deadlock */

	zassert_ok(device_give(dev));
	zassert_ok(device_give(dev));
}

ZTEST(device_lock, test_try_take)
{
	const struct device *dev = LOCKED_DEV;

	/* Acquire normally, then try again non-blocking (same thread → ok). */
	zassert_ok(device_take(dev, K_FOREVER));
	zassert_ok(device_take(dev, K_NO_WAIT));
	zassert_ok(device_give(dev));
	zassert_ok(device_give(dev));
}

ZTEST(device_lock, test_try_take_unlocked)
{
	zassert_ok(device_take(UNLOCKED_DEV, K_NO_WAIT));
	zassert_ok(device_give(UNLOCKED_DEV));
}

/**
 * Mutual exclusion: thread B cannot enter the critical section while thread A
 * holds the device lock.
 */
ZTEST(device_lock, test_mutual_exclusion)
{
	const struct device *dev = LOCKED_DEV;

	if (!IS_ENABLED(CONFIG_DEVICE_API_LOCK)) {
		ztest_test_skip();
	}

	atomic_set(&lock_sequence, 0);

	k_thread_create(&thread_a_data, thread_a_stack, STACK_SIZE,
			thread_a_fn, (void *)dev, NULL, NULL,
			K_PRIO_PREEMPT(1), 0, K_NO_WAIT);

	k_thread_create(&thread_b_data, thread_b_stack, STACK_SIZE,
			thread_b_fn, (void *)dev, NULL, NULL,
			K_PRIO_PREEMPT(1), 0, K_NO_WAIT);

	k_thread_join(&thread_a_data, K_FOREVER);
	k_thread_join(&thread_b_data, K_FOREVER);

	/* Both threads completed their critical sections. */
	zassert_equal(atomic_get(&lock_sequence), 2);
}

ZTEST_SUITE(device_lock, NULL, NULL, NULL, NULL, NULL);
