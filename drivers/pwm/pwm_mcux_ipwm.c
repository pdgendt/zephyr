/*
 * Copyright (c) 2019, Linaro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_imx_ipwm

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(imx_ipwm, CONFIG_PWM_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>

#include <soc.h>

#define MCUX_IPWM_SWR_LOOP 10

struct mcux_ipwm_config {
	PWM_Type *base;

	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;

	const struct pinctrl_dev_config *pincfg;

	uint16_t prescaler;
	uint8_t run_debug;
	uint8_t run_wait;
};

struct mcux_ipwm_data {
	uint16_t period_cycles;
};

static int mcux_ipwm_wait_fifo_slot(const struct device *dev)
{
	const struct mcux_ipwm_config *cfg = dev->config;
	uint32_t clock_rate;
	int ret;

	if (PWM_PWMSR_FIFOAV(cfg->base->PWMSR) < 4) {
		return 0;
	}

	ret = clock_control_get_rate(cfg->clock_dev, cfg->clock_subsys, &clock_rate);
	if (ret < 0) {
		return ret;
	}

	clock_rate /= cfg->prescaler;

	k_sleep(K_NSEC(DIV_ROUND_UP(NSEC_PER_SEC, clock_rate)));

	if (PWM_PWMSR_FIFOAV(cfg->base->PWMSR) >= 4) {
		return -EAGAIN;
	}

	return 0;
}

static int mcux_ipwm_reset(const struct device *dev)
{
	const struct mcux_ipwm_config *cfg = dev->config;
	size_t wait_count = 0;
	uint32_t pwmcr;

	cfg->base->PWMCR = PWM_PWMCR_SWR_MASK;

	do {
		k_sleep(K_MSEC(1));
		pwmcr = cfg->base->PWMCR;
	} while (PWM_PWMCR_SWR(pwmcr) && (++wait_count < MCUX_IPWM_SWR_LOOP));

	if (PWM_PWMCR_SWR(pwmcr)) {
		return -EAGAIN;
	}

	return 0;
}

static int mcux_ipwm_set_cycles(const struct device *dev, uint32_t channel, uint32_t period_cycles,
				uint32_t pulse_cycles, pwm_flags_t flags)
{
	const struct mcux_ipwm_config *cfg = dev->config;
	struct mcux_ipwm_data *data = dev->data;
	uint32_t pwmcr;
	int ret;

	ARG_UNUSED(channel);

	if (pulse_cycles > period_cycles || pulse_cycles > UINT16_MAX ||
	    period_cycles > UINT16_MAX) {
		return -EINVAL;
	}

	pwmcr = cfg->base->PWMCR;

	if (pwmcr & PWM_PWMCR_EN_MASK) {
		ret = mcux_ipwm_wait_fifo_slot(dev);
	} else {
		ret = mcux_ipwm_reset(dev);
	}

	if (ret < 0) {
		LOG_ERR("Failed to wait/reset (%d)", ret);
		return ret;
	}

	/*
	 * according to imx pwm RM, the real period value should be
	 * PERIOD value in PWMPR plus 2.
	 */
	if (period_cycles >= 2U) {
		period_cycles -= 2U;
	} else {
		return -EINVAL;
	}

	cfg->base->PWMSAR = PWM_PWMSAR_SAMPLE(pulse_cycles);
	if (data->period_cycles != period_cycles) {
		cfg->base->PWMPR = PWM_PWMPR_PERIOD(period_cycles);
		data->period_cycles = period_cycles;
	}

	/* Use CCM clock source */
	pwmcr = PWM_PWMCR_EN_MASK | PWM_PWMCR_PRESCALER(cfg->prescaler) | PWM_PWMCR_CLKSRC(2) |
		PWM_PWMCR_WAITEN(cfg->run_wait) | PWM_PWMCR_DBGEN(cfg->run_debug) |
		PWM_PWMCR_POUTC(flags & PWM_POLARITY_INVERTED);

	cfg->base->PWMCR = pwmcr;

	return 0;
}

static int mcux_ipwm_get_cycles_per_sec(const struct device *dev, uint32_t channel,
					uint64_t *cycles)
{
	const struct mcux_ipwm_config *cfg = dev->config;
	uint32_t clock_rate;
	int ret;

	ARG_UNUSED(channel);

	ret = clock_control_get_rate(cfg->clock_dev, cfg->clock_subsys, &clock_rate);
	if (ret < 0) {
		return ret;
	}

	*cycles = clock_rate / cfg->prescaler;

	return 0;
}

static int mcux_ipwm_init(const struct device *dev)
{
	const struct mcux_ipwm_config *cfg = dev->config;
	int ret;

	if (!device_is_ready(cfg->clock_dev)) {
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (ret < 0) {
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		(void)clock_control_off(cfg->clock_dev, cfg->clock_subsys);
		return ret;
	}

	return 0;
}

static const struct pwm_driver_api mcux_ipwm_driver_api = {
	.set_cycles = mcux_ipwm_set_cycles,
	.get_cycles_per_sec = mcux_ipwm_get_cycles_per_sec,
};

#define MCUX_IPWM_DEFINE(n)                                                                        \
	BUILD_ASSERT(1 <= DT_INST_PROP(n, prescaler) && DT_INST_PROP(n, prescaler) <= 4096,        \
		     "Invalid prescaler [1-4096]");                                                \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct mcux_ipwm_config mcux_ipwm_cfg##n = {                                  \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),              \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
		.prescaler = DT_INST_PROP(n, prescaler),                                           \
		.run_debug = DT_INST_PROP(n, run_in_debug),                                        \
		.run_wait = DT_INST_PROP(n, run_in_wait),                                          \
	};                                                                                         \
	static struct mcux_ipwm_data mcux_ipwm_data##n;                                            \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, mcux_ipwm_init, NULL, &mcux_ipwm_data##n, &mcux_ipwm_cfg##n,      \
			      POST_KERNEL, CONFIG_PWM_INIT_PRIORITY, &mcux_ipwm_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MCUX_IPWM_DEFINE)
