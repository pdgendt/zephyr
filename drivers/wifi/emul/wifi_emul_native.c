/*
 * SPDX-FileCopyrightText: 2026 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <cmdline.h>
#include <posix_native_task.h>

#include <zephyr/device.h>
#include <zephyr/drivers/wifi/wifi_emul.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/wifi.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(wifi_emul_native, CONFIG_WIFI_LOG_LEVEL);

#define WIFI_EMUL_SPEC_MAX 256

/* Raw --wifi-emul-ap specs captured at command-line parsing (pre-boot), applied
 * once the device is initialized.
 */
static char cmdline_specs[CONFIG_WIFI_EMUL_AP_COUNT_MAX][WIFI_EMUL_SPEC_MAX];
static size_t cmdline_spec_count;
static size_t cmdline_dropped;

static void wifi_emul_ap_opt_found(char *argv, int offset)
{
	if (cmdline_spec_count >= ARRAY_SIZE(cmdline_specs)) {
		cmdline_dropped++;
		return;
	}

	strncpy(cmdline_specs[cmdline_spec_count], &argv[offset], WIFI_EMUL_SPEC_MAX - 1);
	cmdline_specs[cmdline_spec_count][WIFI_EMUL_SPEC_MAX - 1] = '\0';
	cmdline_spec_count++;
}

static void wifi_emul_native_add_options(void)
{
	static struct args_struct_t wifi_emul_options[] = {
		{
			.option = "wifi-emul-ap",
			.name = "spec",
			.type = 's',
			.call_when_found = wifi_emul_ap_opt_found,
			.descript =
				"Register an emulated Wi-Fi access point at boot (repeatable). "
				"spec is a comma-separated list of key=value fields: "
				"ssid=<name>,bssid=<xx:xx:xx:xx:xx:xx>,channel=<n>,band=<2|5|6>,"
				"security=<n> and optionally psk=<key>,rssi=<dBm>,hidden",
		},
		ARG_TABLE_ENDMARKER,
	};

	native_add_command_line_opts(wifi_emul_options);
}

NATIVE_TASK(wifi_emul_native_add_options, PRE_BOOT_1, 11);

static int wifi_emul_parse_band(const char *val, enum wifi_frequency_bands *band)
{
	if (strcmp(val, "2") == 0) {
		*band = WIFI_FREQ_BAND_2_4_GHZ;
	} else if (strcmp(val, "5") == 0) {
		*band = WIFI_FREQ_BAND_5_GHZ;
	} else if (strcmp(val, "6") == 0) {
		*band = WIFI_FREQ_BAND_6_GHZ;
	} else {
		return -EINVAL;
	}

	return 0;
}

static int wifi_emul_parse_long(const char *val, long min, long max, long *out)
{
	char *end;
	long v;

	if (*val == '\0') {
		return -EINVAL;
	}

	v = strtol(val, &end, 10);
	if (*end != '\0' || v < min || v > max) {
		return -EINVAL;
	}

	*out = v;

	return 0;
}

static int wifi_emul_parse_spec(const char *spec, struct wifi_emul_ap *ap)
{
	char buf[WIFI_EMUL_SPEC_MAX];
	char *cur;
	bool have_ssid = false;
	bool have_bssid = false;
	bool have_channel = false;
	bool have_band = false;
	bool have_security = false;

	memset(ap, 0, sizeof(*ap));

	strncpy(buf, spec, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';

	cur = buf;
	while (*cur != '\0') {
		char *tok = cur;
		char *comma = strchr(cur, ',');
		char *val;
		long num;

		if (comma != NULL) {
			*comma = '\0';
			cur = comma + 1;
		} else {
			cur += strlen(cur);
		}

		if (*tok == '\0') {
			continue;
		}

		val = strchr(tok, '=');
		if (val != NULL) {
			*val++ = '\0';
		}

		if (strcmp(tok, "hidden") == 0) {
			ap->hidden = true;
			continue;
		}

		if (val == NULL) {
			return -EINVAL;
		}

		if (strcmp(tok, "ssid") == 0) {
			size_t len = strlen(val);

			if (len == 0 || len > WIFI_SSID_MAX_LEN) {
				return -EINVAL;
			}
			memcpy(ap->ssid, val, len);
			ap->ssid_length = (uint8_t)len;
			have_ssid = true;
		} else if (strcmp(tok, "bssid") == 0) {
			if (net_bytes_from_str(ap->bssid, sizeof(ap->bssid), val) < 0) {
				return -EINVAL;
			}
			have_bssid = true;
		} else if (strcmp(tok, "channel") == 0) {
			if (wifi_emul_parse_long(val, 1, UINT8_MAX, &num) < 0) {
				return -EINVAL;
			}
			ap->channel = (uint8_t)num;
			have_channel = true;
		} else if (strcmp(tok, "band") == 0) {
			if (wifi_emul_parse_band(val, &ap->band) < 0) {
				return -EINVAL;
			}
			have_band = true;
		} else if (strcmp(tok, "security") == 0) {
			if (wifi_emul_parse_long(val, 0, __WIFI_SECURITY_TYPE_AFTER_LAST - 1,
						 &num) < 0) {
				return -EINVAL;
			}
			ap->security = (enum wifi_security_type)num;
			have_security = true;
		} else if (strcmp(tok, "psk") == 0) {
			size_t len = strlen(val);

			if (len > WIFI_PSK_MAX_LEN) {
				return -EINVAL;
			}
			memcpy(ap->psk, val, len);
		} else if (strcmp(tok, "rssi") == 0) {
			if (wifi_emul_parse_long(val, INT8_MIN, INT8_MAX, &num) < 0) {
				return -EINVAL;
			}
			ap->rssi = (int8_t)num;
		} else {
			return -EINVAL;
		}
	}

	if (!have_ssid || !have_bssid || !have_channel || !have_band || !have_security) {
		return -EINVAL;
	}

	return 0;
}

void wifi_emul_native_load_aps(const struct device *dev)
{
	if (cmdline_dropped > 0) {
		LOG_WRN("Dropped %zu --wifi-emul-ap option(s), increase "
			"CONFIG_WIFI_EMUL_AP_COUNT_MAX",
			cmdline_dropped);
	}

	for (size_t i = 0; i < cmdline_spec_count; i++) {
		struct wifi_emul_ap ap;
		int ret;

		if (wifi_emul_parse_spec(cmdline_specs[i], &ap) < 0) {
			LOG_ERR("Invalid --wifi-emul-ap spec: \"%s\"", cmdline_specs[i]);
			continue;
		}

		ret = wifi_emul_ap_add(dev, &ap);
		if (ret < 0) {
			LOG_WRN("Could not add AP \"%.*s\" (%d)", ap.ssid_length, ap.ssid, ret);
		} else {
			LOG_INF("Added AP \"%.*s\" from command line", ap.ssid_length, ap.ssid);
		}
	}
}
