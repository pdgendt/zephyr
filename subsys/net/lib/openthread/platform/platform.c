/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief
 *   This file includes the platform-specific initializers.
 */

#include <kernel.h>
#include <openthread/instance.h>
#include <openthread/tasklet.h>
#include <lib/platform/exit_code.h>

#include "platform-zephyr.h"

void otSysInit(int argc, char *argv[])
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	platformAlarmInit();
	platformRadioInit();
}

void otSysProcessDrivers(otInstance *aInstance)
{
	platformRadioProcess(aInstance);
	platformAlarmProcess(aInstance);

	if (IS_ENABLED(CONFIG_OPENTHREAD_COPROCESSOR)) {
		platformUartProcess(aInstance);
	}
}

/**
 * openthread has this function but uses strerror which isn't
 * supported by zephyr
 */
const char *otExitCodeToString(uint8_t aExitCode)
{
	const char *retval = NULL;

	switch (aExitCode)
	{
	case OT_EXIT_SUCCESS:
		retval = "Success";
		break;

	case OT_EXIT_FAILURE:
		retval = "Failure";
		break;

	case OT_EXIT_INVALID_ARGUMENTS:
		retval = "InvalidArgument";
		break;

	case OT_EXIT_RADIO_SPINEL_INCOMPATIBLE:
		retval = "RadioSpinelIncompatible";
		break;

	case OT_EXIT_RADIO_SPINEL_RESET:
		retval = "RadioSpinelReset";
		break;

	case OT_EXIT_RADIO_SPINEL_NO_RESPONSE:
		retval = "RadioSpinelNoResponse";
		break;

	case OT_EXIT_ERROR_ERRNO:
		retval = "ErrorNo";
		break;

	default:
		retval = "UnknownExitCode";
		break;
	}

	return retval;
}
