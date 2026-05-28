# Copyright (c) 2026 Basalte bv
#
# SPDX-License-Identifier: Apache-2.0

import logging
import re

import pytest

import zcmake


@pytest.fixture(autouse=True)
def _reset_zcmake_logger():
    logger = logging.getLogger('zcmake')
    saved_handlers = logger.handlers[:]
    saved_level = logger.level
    saved_propagate = logger.propagate
    yield
    logger.handlers[:] = saved_handlers
    logger.setLevel(saved_level)
    logger.propagate = saved_propagate


def _zcmake_info_messages(caplog):
    return [
        r.getMessage() for r in caplog.records if r.name == 'zcmake' and r.levelno == logging.INFO
    ]


def test_run_cmake_dry_run_logs_full_command(caplog):
    '''run_cmake(dry_run=True) emits the full cmake command via _logger.info.'''
    caplog.set_level(logging.INFO, logger='zcmake')
    assert zcmake.run_cmake(['-Bbuild', '-Ssrc'], dry_run=True) is None
    msgs = _zcmake_info_messages(caplog)
    # First message is from _ensure_min_version, second from run_cmake.
    assert len(msgs) == 2
    assert 'Dry run' in msgs[1]
    assert '-Bbuild' in msgs[1]
    assert '-Ssrc' in msgs[1]


def test_run_cmake_dry_run_with_cwd_includes_cwd(caplog):
    '''Dry-run output annotates the cwd as "(in <path>)" when one is given.'''
    caplog.set_level(logging.INFO, logger='zcmake')
    zcmake.run_cmake(['-Bbuild'], cwd='/tmp/build', dry_run=True)
    assert any('(in /tmp/build)' in m for m in _zcmake_info_messages(caplog))


def test_ensure_min_version_dry_run_logs_version_command(caplog):
    '''_ensure_min_version(dry_run=True) logs the "cmake --version" probe via _logger.info.'''
    caplog.set_level(logging.INFO, logger='zcmake')
    zcmake._ensure_min_version('cmake', dry_run=True)
    msgs = _zcmake_info_messages(caplog)
    assert len(msgs) == 1
    assert re.match(r'^Dry run: .*--version$', msgs[0])
