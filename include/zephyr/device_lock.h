/*
 * SPDX-FileCopyrightText: 2026 Basalte bv
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Device API lock — thread-safe serialization of driver API calls.
 *
 * @defgroup device_lock_api Device API Lock
 * @ingroup device_model
 * @{
 *
 * Devices whose DTS node carries the ``zephyr,api-lock`` boolean property
 * are assigned a statically-allocated @ref k_mutex. Driver class headers
 * (e.g. flash) that opt in wrap their @c z_impl_* stubs with
 * device_take() and device_give(), serializing concurrent callers without
 * per-driver boilerplate.
 *
 * Callers may also hold the lock across multiple API calls to form atomic
 * compound operations:
 *
 * @code
 *   device_take(flash_dev);
 *   flash_erase(flash_dev, offset, size);
 *   flash_write(flash_dev, offset, buf, size);
 *   device_give(flash_dev);
 * @endcode
 *
 * @}
 */

#ifndef ZEPHYR_INCLUDE_DEVICE_LOCK_H_
#define ZEPHYR_INCLUDE_DEVICE_LOCK_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup device_lock_api
 * @{
 */

/**
 * @brief Acquire the device API lock.
 *
 * Serializes access to a device whose DTS node carries the
 * ``zephyr,api-lock`` property. The caller may hold this lock across
 * multiple API calls to compose atomic compound operations. The underlying
 * mutex is recursive so that driver-internal re-entry does not deadlock.
 *
 * @note May not be called in ISRs.
 *
 * When @kconfig{CONFIG_DEVICE_API_LOCK} is disabled, or the device has no
 * lock configured, this function compiles away to a no-op.
 *
 * @param dev     Device instance.
 * @param timeout Maximum time to wait, or @ref K_NO_WAIT / @ref K_FOREVER.
 *
 * @retval 0       Lock acquired (or no lock configured).
 * @retval -EBUSY  Lock is held and @p timeout is @ref K_NO_WAIT.
 * @retval -EAGAIN Timed out waiting for the lock.
 * @retval -errno  Other mutex error.
 */
static inline int device_take(const struct device *dev, k_timeout_t timeout)
{
#if defined(CONFIG_DEVICE_API_LOCK)
	if (dev->api_lock != NULL) {
		return k_mutex_lock(dev->api_lock, timeout);
	}
#else
	ARG_UNUSED(dev);
	ARG_UNUSED(timeout);
#endif
	return 0;
}

/**
 * @brief Release the device API lock.
 *
 * Counterpart to device_take(). Must be called once for each successful
 * device_take() call on the same device by the same thread.
 *
 * When @kconfig{CONFIG_DEVICE_API_LOCK} is disabled, or the device has no
 * lock configured, this function compiles away to a no-op.
 *
 * @param dev Device instance.
 *
 * @retval 0 Lock acquired (or no lock configured).
 * @retval -errno On mutex error.
 */
static inline int device_give(const struct device *dev)
{
#if defined(CONFIG_DEVICE_API_LOCK)
	if (dev->api_lock != NULL) {
		return k_mutex_unlock(dev->api_lock);
	}
#else
	ARG_UNUSED(dev);
#endif
	return 0;
}

/**
 * @brief Check whether a device has an API lock configured.
 *
 * Returns true when @kconfig{CONFIG_DEVICE_API_LOCK} is enabled and the
 * device was created from a DTS node that carries the ``zephyr,api-lock``
 * property. Returns false in all other cases, including when the feature is
 * entirely disabled at build time.
 *
 * @param dev Device instance.
 *
 * @retval true  Device has an API lock.
 * @retval false Device has no API lock, or the feature is disabled.
 */
static inline bool device_has_api_lock(const struct device *dev)
{
#if defined(CONFIG_DEVICE_API_LOCK)
	return dev->api_lock != NULL;
#else
	ARG_UNUSED(dev);
	return false;
#endif
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DEVICE_LOCK_H_ */
