#!/usr/bin/env python3

# Copyright (c) 2024 Basalte bv
# SPDX-License-Identifier: Apache-2.0

import argparse
import json
import os
import re
import shlex
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

# Match C23 attributes with the zephyr namespace, for example:
# [[zephyr::func("syscall")]]
ANNOTATION = re.compile(r'\[\[\s*zephyr::(?P<attr>[a-zA-Z0-9_]+)(?:\((?P<args>.+)\))?\s*\]\]')

ZPP_ARGS = [
    "-E",  # Only run preprocessor
    "-P",  # No comment directives
    "-D__ZPP__",  # ZPP define
    f"-I{Path(__file__).parent / "zpp" / "include"}",  # Add include stubs
]


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


def process_command(cmd: CompileCommand):
    # Only parse c files
    if cmd.file.suffix != ".c":
        inf(f"SKIP(filetype): {cmd.file}")
        return
    if not cmd.file.exists():
        inf(f"SKIP(missing): {cmd.file}")
        return

    if os.name == "nt":
        cmd.command = cmd.command.replace("\\", "\\\\")  # Fix windows paths

    # Use the argument parser to drop the output argument and keep everything else
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output")
    parser.add_argument("-DZPP", action="store_true", dest="zpp")
    command_args, command_remaining = parser.parse_known_args(shlex.split(cmd.command))

    if not command_args.zpp:
        inf(f"SKIP(nozpp): {cmd.file}")
        return

    inf(f"ZPP: {cmd.file}")

    # Generate the source code as produced by the preprocessor
    src = subprocess.check_output(command_remaining + ZPP_ARGS).decode()
    for line in src.splitlines():
        match = re.search(ANNOTATION, line)
        if match is None:
            continue

        if match.group("attr") == "struct":
            pass

        dbg(f'{match.group("attr")}, {match.group("args")}')

    if args.intermediate:
        with open(cmd.directory / f"{command_args.output}.i", "w") as fp:
            fp.write(src)


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
        "database",
        help="Database file, typically compile_commands.json in the build directory.",
    )

    args = parser.parse_args()


def main():
    parse_args()

    inf(f"ZPP_ARGS: {" ".join(ZPP_ARGS)}")

    with open(args.database) as fp:
        db_json = json.load(fp)

    for item in db_json:
        process_command(
            CompileCommand(
                command=item.get("command"),
                directory=Path(item.get("directory")),
                file=Path(item.get("file")),
            ),
        )


if __name__ == "__main__":
    main()
