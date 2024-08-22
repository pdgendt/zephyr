# Copyright (c) 2024 Basalte bv
# SPDX-License-Identifier: Apache-2.0

import logging
import os
from pathlib import Path
from west import util

logger = logging.getLogger("twister")
logger.setLevel(logging.DEBUG)


def setup_sca(testplan, env):
    logger.info(f"Using SCA tool: {env.options.sca_variant}")

    topdir = util.west_topdir(start=Path.cwd(), fall_back=True)

    for ts in testplan.instances.values():
        if env.options.sca_platform and ts.platform.name not in env.options.sca_platform:
            continue

        ts.extra_args.append(f"ZEPHYR_SCA_VARIANT={env.options.sca_variant}")
        logger.debug(f"SCA enabled for {ts.platform.name}:{ts.testsuite.name}")

        if env.options.sca_variant == "codechecker":
            if "CODECHECKER_NAME" not in os.environ:
                ts.extra_args.append(f"CODECHECKER_NAME={ts.platform.name}:{ts.testsuite.name}")
            if "CODECHECKER_ANALYZE_JOBS" not in os.environ:
                ts.extra_args.append("CODECHECKER_ANALYZE_JOBS=1")
            if topdir and "CODECHECKER_TRIM_PATH_PREFIX" not in os.environ:
                ts.extra_args.append(f"CODECHECKER_TRIM_PATH_PREFIX={topdir}")
            if env.version and "CODECHECKER_STORE_TAG" not in os.environ:
                ts.extra_args.append(f"CODECHECKER_STORE_TAG={env.version}")
