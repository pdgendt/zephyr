/*
 * Copyright (c) 2025 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_imx_snvs_rtc

#include <zephyr/drivers/rtc.h>
#include <zephyr/logging/log.h>

#include "rtc_utils.h"

LOG_MODULE_REGISTER(nxp_imx_snvs, CONFIG_RTC_LOG_LEVEL);

#include <fsl_snvs_hp.h>
#include <fsl_snvs_lp.h>

enum {
	IMX_SNVS_RTC_ID_LP = 0,
	IMX_SNVS_RTC_ID_HP,
	IMX_SNVS_RTC_NUM,
};

struct rtc_imx_snvs_config {
	SNVS_Type *base;
#ifdef CONFIG_RTC_ALARM
	void (*irq_config_func)(void);
#endif
	bool wakeup_source;
};

struct rtc_imx_snvs_data {
#ifdef CONFIG_RTC_ALARM
	rtc_alarm_callback alarm_callback[IMX_SNVS_RTC_NUM];
	void *alarm_user_data[IMX_SNVS_RTC_NUM];
#endif
};

static int status2errno(status_t status)
{
	switch (status) {
	case kStatus_Success:
		return 0;
	case kStatus_InvalidArgument:
		return -EINVAL;
	case kStatus_Timeout:
		return -EAGAIN;
	case kStatus_OutOfRange:
		return -ERANGE;
	case kStatus_ReadOnly:
		return -EROFS;
	case kStatus_Fail:
		return -EFAULT;
	case kStatus_Busy:
		return -EBUSY;
	case kStatus_NoData:
		return -ENODATA;
	case kStatus_NoTransferInProgress:
		return -ENOSR;
	default:
		break;
	}

	return -ENXIO;
}

static inline void rtc2datetime(const struct rtc_time *timeptr, snvs_lp_srtc_datetime_t *datetime)
{
	datetime->year = timeptr->tm_year + 1900;
	datetime->month = timeptr->tm_mon + 1;
	datetime->day = timeptr->tm_mday;
	datetime->hour = timeptr->tm_hour;
	datetime->minute = timeptr->tm_min;
	datetime->second = timeptr->tm_sec;
}

static inline void datetime2rtc(const snvs_lp_srtc_datetime_t *datetime, struct rtc_time *timeptr)
{
	timeptr->tm_year = datetime->year - 1900;
	timeptr->tm_mon = datetime->month - 1;
	timeptr->tm_mday = datetime->day;
	timeptr->tm_hour = datetime->hour;
	timeptr->tm_min = datetime->minute;
	timeptr->tm_sec = datetime->second;
	timeptr->tm_nsec = 0;
	timeptr->tm_yday = -1;
	timeptr->tm_wday = -1;
	timeptr->tm_isdst = -1;
}

static int rtc_imx_snvs_set_time(const struct device *dev, const struct rtc_time *timeptr)
{
	const struct rtc_imx_snvs_config *config = dev->config;
	snvs_lp_srtc_datetime_t datetime;
	int ret;

	if (!rtc_utils_validate_rtc_time(timeptr, 0)) {
		return -EINVAL;
	}

	rtc2datetime(timeptr, &datetime);

	ret = status2errno(SNVS_LP_SRTC_SetDatetime(config->base, &datetime));
	if (ret < 0) {
		return ret;
	}

	/* Sync to our high power RTC */
	SNVS_HP_RTC_TimeSynchronize(config->base);

	return 0;
}

static int rtc_imx_snvs_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	const struct rtc_imx_snvs_config *config = dev->config;
	snvs_lp_srtc_datetime_t datetime;

	SNVS_LP_SRTC_GetDatetime(config->base, &datetime);

	datetime2rtc(&datetime, timeptr);

	return 0;
}

#ifdef CONFIG_RTC_ALARM
static int rtc_imx_snvs_alarm_get_supported_fields(const struct device *dev, uint16_t id,
						   uint16_t *mask)
{
	ARG_UNUSED(dev);

	if (id >= IMX_SNVS_RTC_NUM) {
		return -EINVAL;
	}

	*mask = (RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE |
		 RTC_ALARM_TIME_MASK_HOUR | RTC_ALARM_TIME_MASK_MONTHDAY |
		 RTC_ALARM_TIME_MASK_MONTH | RTC_ALARM_TIME_MASK_YEAR);

	return 0;
}

static int rtc_imx_snvs_alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask,
				       const struct rtc_time *timeptr)
{
	const struct rtc_imx_snvs_config *config = dev->config;
	snvs_lp_srtc_datetime_t datetime;

	ARG_UNUSED(mask);

	if (id >= IMX_SNVS_RTC_NUM) {
		return -EINVAL;
	}

	rtc2datetime(timeptr, &datetime);

	if (id == IMX_SNVS_RTC_ID_LP) {
		return status2errno(SNVS_LP_SRTC_SetAlarm(config->base, &datetime));
	}

	return status2errno(
		SNVS_HP_RTC_SetAlarm(config->base, (snvs_hp_rtc_datetime_t *)&datetime));
}

static int rtc_imx_snvs_alarm_get_time(const struct device *dev, uint16_t id, uint16_t *mask,
				       struct rtc_time *timeptr)
{
	const struct rtc_imx_snvs_config *config = dev->config;
	snvs_lp_srtc_datetime_t datetime;

	ARG_UNUSED(mask);

	if (id >= IMX_SNVS_RTC_NUM) {
		return -EINVAL;
	}

	if (id == IMX_SNVS_RTC_ID_LP) {
		SNVS_LP_SRTC_GetAlarm(config->base, &datetime);
	} else {
		SNVS_HP_RTC_GetAlarm(config->base, (snvs_hp_rtc_datetime_t *)&datetime);
	}

	datetime2rtc(&datetime, timeptr);

	return 0;
}

static int rtc_imx_snvs_alarm_is_pending(const struct device *dev, uint16_t id)
{
	const struct rtc_imx_snvs_config *config = dev->config;

	switch (id) {
	case IMX_SNVS_RTC_ID_LP:
		if ((SNVS_LP_SRTC_GetStatusFlags(config->base) & kSNVS_SRTC_AlarmInterruptFlag) !=
		    0U) {
			SNVS_LP_SRTC_ClearStatusFlags(config->base, kSNVS_SRTC_AlarmInterruptFlag);
			return 1;
		}
		return 0;
	case IMX_SNVS_RTC_ID_HP:
		if ((SNVS_HP_RTC_GetStatusFlags(config->base) & kSNVS_RTC_AlarmInterruptFlag) !=
		    0U) {
			SNVS_HP_RTC_ClearStatusFlags(config->base, kSNVS_RTC_AlarmInterruptFlag);
			return 1;
		}
		return 0;
	default:
		break;
	}

	return -EINVAL;
}

static int rtc_imx_snvs_alarm_set_callback(const struct device *dev, uint16_t id,
					   rtc_alarm_callback callback, void *user_data)
{
	const struct rtc_imx_snvs_config *config = dev->config;
	struct rtc_imx_snvs_data *data = dev->data;
	uint32_t key;

	if (id >= IMX_SNVS_RTC_NUM) {
		return -EINVAL;
	}

	key = irq_lock();
	switch (id) {
	case IMX_SNVS_RTC_ID_LP:
		if (callback != NULL) {
			SNVS_LP_SRTC_EnableInterrupts(config->base, kSNVS_SRTC_AlarmInterrupt);
		} else {
			SNVS_LP_SRTC_DisableInterrupts(config->base, kSNVS_SRTC_AlarmInterrupt);
		}
		break;
	case IMX_SNVS_RTC_ID_HP:
		if (callback != NULL) {
			SNVS_HP_RTC_EnableInterrupts(config->base, kSNVS_RTC_AlarmInterrupt);
		} else {
			SNVS_HP_RTC_DisableInterrupts(config->base, kSNVS_RTC_AlarmInterrupt);
		}
		break;
	}
	data->alarm_callback[id] = callback;
	data->alarm_user_data[id] = user_data;
	irq_unlock(key);

	return 0;
}

static void rtc_imx_snvs_isr(const struct device *dev)
{
	const struct rtc_imx_snvs_config *config = dev->config;
	struct rtc_imx_snvs_data *data = dev->data;

	if (data->alarm_callback[IMX_SNVS_RTC_ID_LP] != NULL &&
	    (SNVS_LP_SRTC_GetStatusFlags(config->base) & kSNVS_SRTC_AlarmInterruptFlag) != 0U) {
		SNVS_LP_SRTC_ClearStatusFlags(config->base, kSNVS_SRTC_AlarmInterruptFlag);
		data->alarm_callback[IMX_SNVS_RTC_ID_LP](dev, IMX_SNVS_RTC_ID_LP,
							 data->alarm_user_data[IMX_SNVS_RTC_ID_LP]);
	}

	if (data->alarm_callback[IMX_SNVS_RTC_ID_HP] != NULL &&
	    (SNVS_HP_RTC_GetStatusFlags(config->base) & kSNVS_RTC_AlarmInterruptFlag) != 0U) {
		SNVS_HP_RTC_ClearStatusFlags(config->base, kSNVS_RTC_AlarmInterruptFlag);
		data->alarm_callback[IMX_SNVS_RTC_ID_HP](dev, IMX_SNVS_RTC_ID_HP,
							 data->alarm_user_data[IMX_SNVS_RTC_ID_HP]);
	}
}
#endif /* CONFIG_RTC_ALARM */

static DEVICE_API(rtc, rtc_imx_snvs_driver_api) = {
	.set_time = rtc_imx_snvs_set_time,
	.get_time = rtc_imx_snvs_get_time,
#ifdef CONFIG_RTC_ALARM
	.alarm_get_supported_fields = rtc_imx_snvs_alarm_get_supported_fields,
	.alarm_set_time = rtc_imx_snvs_alarm_set_time,
	.alarm_get_time = rtc_imx_snvs_alarm_get_time,
	.alarm_is_pending = rtc_imx_snvs_alarm_is_pending,
	.alarm_set_callback = rtc_imx_snvs_alarm_set_callback,
#endif /* CONFIG_RTC_ALARM */
};

static int rtc_imx_snvs_init(const struct device *dev)
{
	const struct rtc_imx_snvs_config *config = dev->config;
	snvs_hp_rtc_config_t hp_rtc_config;
	snvs_lp_srtc_config_t lp_srtc_config;

	SNVS_HP_RTC_GetDefaultConfig(&hp_rtc_config);
	SNVS_HP_RTC_Init(config->base, &hp_rtc_config);

	/* Reset power glitch detector */
	SNVS_LP_Init(config->base);

	/* Init SRTC to default config */
	SNVS_LP_SRTC_GetDefaultConfig(&lp_srtc_config);
	SNVS_LP_SRTC_Init(config->base, &lp_srtc_config);

	if (config->wakeup_source) {
		config->base->LPCR |= SNVS_LPCR_LPWUI_EN_MASK;
	}

	/* RTC should always run */
	SNVS_LP_SRTC_StartTimer(config->base);
	SNVS_HP_RTC_TimeSynchronize(config->base);
	SNVS_HP_RTC_StartTimer(config->base);

#ifdef CONFIG_RTC_ALARM
	config->irq_config_func();
#endif /* CONFIG_RTC_ALARM */

	return 0;
}

#ifdef CONFIG_RTC_ALARM
#define RTC_NXP_IMX_SNVS_IRQ_DEFINE(inst)                                                          \
	static void rtc_imx_snvs_irq_config_##inst(void)                                           \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), rtc_imx_snvs_isr,     \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}
#else
#define RTC_NXP_IMX_SNVS_IRQ_DEFINE(...)
#endif /* CONFIG_RTC_ALARM */

#define RTC_NXP_IMX_SNVS_DEFINE(inst)                                                              \
	RTC_NXP_IMX_SNVS_IRQ_DEFINE(inst)                                                          \
	static const struct rtc_imx_snvs_config rtc_imx_snvs_config_##inst = {                     \
		IF_ENABLED(CONFIG_RTC_ALARM, (.irq_config_func = rtc_imx_snvs_irq_config_##inst,)) \
		.base = (SNVS_Type *)DT_REG_ADDR(DT_INST_PARENT(inst)),                            \
		.wakeup_source = DT_INST_PROP(inst, wakeup_source),                                \
	};                                                                                         \
	static struct rtc_imx_snvs_data rtc_imx_snvs_data_##inst;                                  \
	DEVICE_DT_INST_DEFINE(inst, rtc_imx_snvs_init, NULL, &rtc_imx_snvs_data_##inst,            \
			      &rtc_imx_snvs_config_##inst, PRE_KERNEL_1, CONFIG_RTC_INIT_PRIORITY, \
			      &rtc_imx_snvs_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RTC_NXP_IMX_SNVS_DEFINE)
