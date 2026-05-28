# Copyright (c) 2026 Basalte bv
#
# SPDX-License-Identifier: Apache-2.0

import logging
import uuid

import pytest
from west.commands import Verbosity, WestCommand

from build_helpers import WestLogHandler, forward_logging_to_west


class _DummyCommand(WestCommand):
    # color_ui short-circuits the config-driven property that would
    # otherwise require a real west workspace to be available.
    color_ui = False

    def do_add_parser(self, parser_adder):
        pass

    def do_run(self, args, unknown_args):
        pass


def _make_command(verbosity=Verbosity.DBG_EXTREME):
    return _DummyCommand('dummy', 'help', 'desc', verbosity=verbosity)


@pytest.fixture
def unique_logger():
    # propagate=False keeps hasHandlers() from seeing pytest's root handler.
    name = f'test_bridge_{uuid.uuid4().hex}'
    logger = logging.getLogger(name)
    logger.propagate = False
    yield logger
    logger.handlers.clear()
    logger.setLevel(logging.NOTSET)
    logger.propagate = True


def _make_record(name, level, msg):
    return logging.LogRecord(
        name=name,
        level=level,
        pathname='',
        lineno=0,
        msg=msg,
        args=(),
        exc_info=None,
    )


@pytest.mark.parametrize(
    'level,stream',
    [
        (logging.ERROR, 'err'),
        (logging.WARNING, 'err'),
        (logging.INFO, 'out'),
        (logging.DEBUG, 'out'),
    ],
)
def test_handler_routes_through_west_command(level, stream, capsys):
    '''Bridged records reach the correct WestCommand output stream.'''
    command = _make_command()
    handler = WestLogHandler(command)
    handler.emit(_make_record('mod', level, 'hello'))
    captured = capsys.readouterr()
    assert 'hello' in getattr(captured, stream)
    assert 'hello' not in getattr(captured, 'out' if stream == 'err' else 'err')


def test_handler_critical_routes_to_die(capsys):
    '''CRITICAL+1 records trigger die(), which prints to stderr and exits.'''
    command = _make_command(verbosity=Verbosity.INF)
    handler = WestLogHandler(command)
    with pytest.raises(SystemExit):
        handler.emit(_make_record('mod', logging.CRITICAL + 1, 'boom'))
    assert 'boom' in capsys.readouterr().err


def test_handler_subdebug_gated_by_extreme_verbosity(capsys):
    '''Sub-DEBUG records only reach output when command verbosity is at least DBG_EXTREME.'''
    quiet = _make_command(verbosity=Verbosity.DBG)
    handler = WestLogHandler(quiet)
    handler.emit(_make_record('mod', 1, 'deep'))
    assert 'deep' not in capsys.readouterr().out

    loud = _make_command(verbosity=Verbosity.DBG_EXTREME)
    handler = WestLogHandler(loud)
    handler.emit(_make_record('mod', 1, 'deep'))
    assert 'deep' in capsys.readouterr().out


def test_handler_formats_with_name_and_level(capsys):
    '''Bridged messages are prefixed with the logger name and level.'''
    command = _make_command()
    handler = WestLogHandler(command)
    handler.emit(_make_record('foo', logging.WARNING, 'hello'))
    assert 'foo: WARNING: hello' in capsys.readouterr().err


@pytest.mark.parametrize(
    'verbosity,expected_level',
    [
        (Verbosity.DBG, 1),
        (Verbosity.INF, logging.INFO),
    ],
)
def test_forward_logging_sets_level(unique_logger, verbosity, expected_level):
    '''Logger level follows the command verbosity: sub-DEBUG when verbose, INFO otherwise.'''
    command = _make_command(verbosity=verbosity)
    forward_logging_to_west(command, unique_logger.name)
    assert unique_logger.level == expected_level


def test_forward_logging_accepts_str(unique_logger):
    '''A single logger name (str) attaches one WestLogHandler to that logger.'''
    forward_logging_to_west(_make_command(), unique_logger.name)
    handlers = [h for h in unique_logger.handlers if isinstance(h, WestLogHandler)]
    assert len(handlers) == 1


def test_forward_logging_accepts_iterable():
    '''An iterable of names attaches one WestLogHandler to each named logger.'''
    names = [f'test_bridge_{uuid.uuid4().hex}' for _ in range(2)]
    loggers = [logging.getLogger(n) for n in names]
    for logger in loggers:
        logger.propagate = False
    try:
        forward_logging_to_west(_make_command(), names)
        for logger in loggers:
            handlers = [h for h in logger.handlers if isinstance(h, WestLogHandler)]
            assert len(handlers) == 1, f'{logger.name} missing handler'
    finally:
        for logger in loggers:
            logger.handlers.clear()
            logger.setLevel(logging.NOTSET)
            logger.propagate = True


def test_forward_logging_is_idempotent(unique_logger):
    '''Calling forward_logging_to_west twice on the same logger does not duplicate the handler.'''
    command = _make_command()
    forward_logging_to_west(command, unique_logger.name)
    forward_logging_to_west(command, unique_logger.name)
    handlers = [h for h in unique_logger.handlers if isinstance(h, WestLogHandler)]
    assert len(handlers) == 1


def test_forward_logging_skips_loggers_with_existing_handlers(unique_logger):
    '''A logger that already has a handler is left alone (hasHandlers() guard).'''
    pre_existing = logging.NullHandler()
    unique_logger.addHandler(pre_existing)
    forward_logging_to_west(_make_command(), unique_logger.name)
    west_handlers = [h for h in unique_logger.handlers if isinstance(h, WestLogHandler)]
    assert west_handlers == []
    assert pre_existing in unique_logger.handlers
