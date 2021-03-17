/*
 * Copyright (c) 2021 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(net_otPlat_iface, CONFIG_OPENTHREAD_LOG_LEVEL);

#include <kernel.h>
#include <string.h>

#include <net/ieee802154_radio.h>
#include <net/openthread.h>


static int dummy_init(const struct device *dev)
{
	return 0;
}

static enum ieee802154_hw_caps dummy_get_capabilities(const struct device *dev)
{
	return 0;
}

static int dummy_cca(const struct device *dev)
{
	return 0;
}

static int dummy_set_channel(const struct device *dev, uint16_t channel)
{
	return 0;
}

static int dummy_filter(const struct device *dev,
			bool set,
			enum ieee802154_filter_type type,
			const struct ieee802154_filter *filter)
{
	return -ENOTSUP;
}

static int dummy_set_txpower(const struct device *dev, int16_t dbm)
{
	return 0;
}

static int dummy_tx(const struct device *dev,
		    enum ieee802154_tx_mode mode,
		    struct net_pkt *pkt,
		    struct net_buf *frag)
{
	return 0;
}

static int dummy_start(const struct device *dev)
{
	return 0;
}

static int dummy_stop(const struct device *dev)
{
	return 0;
}

static void dummy_iface_init(struct net_if *iface)
{
	ieee802154_init(iface);
}

static struct ieee802154_radio_api dummy_radio_api = {
	.iface_api.init = dummy_iface_init,

	.get_capabilities = dummy_get_capabilities,
	.cca = dummy_cca,
	.set_channel = dummy_set_channel,
	.filter = dummy_filter,
	.set_txpower = dummy_set_txpower,
	.start = dummy_start,
	.stop = dummy_stop,
	.tx = dummy_tx,
};

NET_DEVICE_INIT(ot_dummy_radio, "OT-DUMMY",
		dummy_init, device_pm_control_nop, NULL, NULL,
		CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		&dummy_radio_api, OPENTHREAD_L2,
		NET_L2_GET_CTX_TYPE(OPENTHREAD_L2), 1280);
