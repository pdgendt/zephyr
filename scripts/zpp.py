#!/usr/bin/env python3

# Copyright (c) 2024 Basalte bv
# SPDX-License-Identifier: Apache-2.0

import argparse
import asyncio
import contextlib
import dataclasses
import json
import os
import re
import shlex
import sys
import tempfile
from collections.abc import AsyncIterator
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

# Match C23 attributes with the zephyr namespace, for example:
# [[zephyr::func("syscall", "WEST_TOPDIR/zephyr/include/zephyr/kernel.h", 2174)]]
ANNOTATION = re.compile(r'\[\[\s*zephyr::(?P<attr>[a-zA-Z0-9_]+)(?:\((?P<args>.+)\))?\s*\]\]')
ANNOTATION_ARG = re.compile(r'\s*(?:"(?P<s>[^"]*)")|(?P<i>\d+)\s*,?')

ANNOTATION_FUNC = re.compile(r'\s*(?P<type>.*?)\s*(?P<name>[a-zA-Z0-9_]+)\s*[(](?P<args>[^)]*)[)]')
ANNOTATION_STRUCT = re.compile(r'\s*struct\s+(?P<name>[a-zA-Z0-9_]+)')

ZPP_ARGS = [
    "-E",  # Only run preprocessor
    "-P",  # No comment directives
    "-D__ZPP__",  # ZPP define
    f'-I{Path(__file__).parent / "zpp" / "include"}',  # Add include stubs
]


class AnnotationType(str, Enum):
    FUNCTION = "func"
    STRUCT = "struct"

    @classmethod
    def has_value(cls, value: str) -> bool:
        try:
            cls(value)
        except ValueError:
            return False
        return True


@dataclass
class Annotation:
    attr: AnnotationType
    args: list[str | int]
    data: dict[str, str] = field(default_factory=dict)

    def __hash__(self) -> int:
        return json.dumps(dataclasses.asdict(self)).__hash__()


@dataclass
class CompileCommand:
    command: str
    directory: Path
    file: Path


def dbg(*elems, **kwargs):
    if args.verbosity > 1:
        print(*elems, **kwargs)


def inf(*elems, **kwargs):
    if args.verbosity > 0:
        print(*elems, **kwargs)


async def tee(cmd: list[str], file: Path | None) -> AsyncIterator[str]:
    """
    Execute a command and yield the output for each line.
    Optionally write the output to a file as well.
    """
    with open(file, "wb") if file else contextlib.nullcontext() as out:
        process = await asyncio.create_subprocess_shell(
            " ".join(cmd), stdout=asyncio.subprocess.PIPE
        )
        assert process.stdout is not None
        while True:
            if process.stdout.at_eof():
                break
            line = await process.stdout.readline()
            if out:
                out.write(line)
            yield line.decode()


async def search_chainable(
    pattern: str | re.Pattern[str], input: str, gen: AsyncIterator[str]
) -> re.Match[str] | None:
    """
    Helper function to search a regex pattern from input, if nothing is
    found we add data from the generator and try again.

    This can be used if a pattern is split over multiple lines.
    """
    while True:
        match = re.search(pattern, input)
        if match is not None:
            return match

        add = await anext(gen)
        if add is None:
            return None

        input += add


async def process_command(cmd: CompileCommand) -> tuple[set[Annotation], list[str]] | None:
    # Only parse c files
    if cmd.file.suffix != ".c":
        inf(f"SKIP(filetype): {cmd.file}")
        return None
    if not cmd.file.exists():
        inf(f"SKIP(missing): {cmd.file}")
        return None

    if os.name == "nt":
        cmd.command = cmd.command.replace("\\", "\\\\")  # Fix windows paths

    # Use the argument parser to drop the output argument and keep everything else
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument("-o", "--output")
    parser.add_argument("-DZPP", action="store_true", dest="zpp")
    command_args, command_remaining = parser.parse_known_args(shlex.split(cmd.command))

    if not command_args.zpp:
        inf(f"SKIP(nozpp): {cmd.file}")
        return None

    result = set()
    inf(f"ZPP: {cmd.file}")

    # Capture the dependencies
    tmpdir = tempfile.TemporaryDirectory()
    deps_file = (
        cmd.directory if args.intermediate else Path(tmpdir.name)
    ) / f"{command_args.output}.zpp.d"
    deps_file.parent.mkdir(parents=True, exist_ok=True)

    output = tee(
        command_remaining + ZPP_ARGS + ["-MMD", "-MF", str(deps_file)],
        cmd.directory / f"{command_args.output}.zpp.i" if args.intermediate else None,
    )

    async for line in output:
        match = re.search(ANNOTATION, line)
        if match is None:
            continue

        if not AnnotationType.has_value(match.group("attr")):
            continue

        # remove the annotation from the line itself
        line = line.replace(match.group(0), "")

        match_args = []
        if match.group("args") is not None:
            # split into string or int arguments
            for m in re.finditer(ANNOTATION_ARG, match.group("args")):
                match_args.append(m.group("s") or int(m.group("i")))

        annotation = Annotation(
            attr=AnnotationType(match.group("attr")),
            args=match_args,
        )

        match annotation.attr:
            case AnnotationType.FUNCTION:
                match_func = await search_chainable(ANNOTATION_FUNC, line, output)
                assert match_func is not None

                annotation.data["type"] = match_func.group("type")
                annotation.data["name"] = match_func.group("name")
                annotation.data["args"] = match_func.group("args")
            case AnnotationType.STRUCT:
                match_struct = await search_chainable(ANNOTATION_STRUCT, line, output)
                assert match_struct is not None

                annotation.data["name"] = match_struct.group("name")

        result.add(annotation)

    with open(deps_file) as df:
        deps = [line.split(":", 1)[-1].strip(' \t\n\r\\') for line in df]

    tmpdir.cleanup()

    dbg("annotations", result)
    dbg("dependencies", deps)

    return result, deps


async def process_command_with_sem(
    cmd: CompileCommand, sem: asyncio.Semaphore
) -> tuple[set[Annotation], list[str]] | None:
    async with sem:
        return await process_command(cmd)


def parse_args():
    global args
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        allow_abbrev=False,
    )

    parser.add_argument(
        "-v",
        "--verbose",
        dest="verbosity",
        action="count",
        help="print more diagnostic messages (option can be given multiple times)",
        default=0,
    )
    parser.add_argument(
        "-i",
        "--intermediate",
        action="store_true",
        help="Store intermediate files",
    )
    parser.add_argument(
        '-j',
        '--jobs',
        nargs='?',
        const=-1,
        default=1,
        type=int,
        action='store',
        help='''Use multiple jobs to parallelize commands.
                Pass no number or -1 to run commands on all cores.''',
    )
    parser.add_argument("-d", "--depfile", action="store", help="The depfile output file")
    parser.add_argument("-o", "--output", action="store", help="The json output file")
    parser.add_argument(
        "database",
        help="Database file, typically compile_commands.json in the build directory.",
    )

    args = parser.parse_args()


async def main():
    parse_args()

    inf(f'ZPP_ARGS: {" ".join(ZPP_ARGS)}')

    with open(args.database) as fp:
        db_json = json.load(fp)

    jobs = args.jobs if args.jobs > 0 else os.cpu_count() or sys.maxsize
    sem = asyncio.Semaphore(jobs)

    # Collect all targets the we depend on
    deps = set()
    result = set()

    cmds = [
        CompileCommand(
            command=item.get("command"),
            directory=Path(item.get("directory")),
            file=Path(item.get("file")),
        )
        for item in db_json
    ]

    annotations = await asyncio.gather(*[process_command_with_sem(cmd, sem) for cmd in cmds])

    for a in annotations:
        if a is None:
            continue

        result.update(a[0])
        deps.update(a[1])

    dbg(result)
    if args.output:
        with open(args.output, "w") as out:
            json.dump([dataclasses.asdict(obj) for obj in result], out, indent=2)

        if args.depfile:
            with open(args.depfile, "w") as out:
                out.write(f"{args.output}: \\\n ")
                out.write(" \\\n ".join(filter(None, sorted(deps))))


if __name__ == "__main__":
    asyncio.run(main())
