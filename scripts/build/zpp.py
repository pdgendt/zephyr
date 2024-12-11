#!/usr/bin/env python3

# Copyright (c) 2024 Basalte bv
# SPDX-License-Identifier: Apache-2.0

import argparse
import json
import os
import re
import shlex
import subprocess
from dataclasses import dataclass
from pathlib import Path

ANNOTATION = re.compile(
    r'__attribute__\s*\(\(annotate\(\s*"(.*)"\s*\)\)\)\s+struct\s+([a-zA-Z0-9_]+)'
)


@dataclass(frozen=True)
class CompileCommand:
    command: str
    directory: Path
    file: Path


def process_command(cmd: CompileCommand, dbg: bool = False):
    parser = argparse.ArgumentParser()

    # Only parse c files
    if cmd.file.suffix != ".c" or not cmd.file.exists():
        return

    if os.name == "nt":
        cmd = cmd.__replace__(command=cmd.command.replace("\\", "\\\\")) # Fix windows paths

    # Use the argument parser to drop the output argument and keep everything else
    parser.add_argument("-o", "--output")
    parser.add_argument("-DZPP", action="store_true", dest="zpp")
    args, command_remaining = parser.parse_known_args(shlex.split(cmd.command))

    if not args.zpp:
        return


    # Generate the source code as produced by the preprocessor
    src = subprocess.check_output(command_remaining + ["-E", "-P", "-D__ZPP__"]).decode()

    # debug only
    with open(cmd.directory / f"{args.output}.i", "w") as fp:
        fp.write(src)


def parse_args():
    global args
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        allow_abbrev=False,
    )


    parser.add_argument("-d", "--debug", action="store_true")
    parser.add_argument(
        "database",
        help="Database file, typically compile_commands.json in the build directory.",
    )

    args = parser.parse_args()


def main():
    parse_args()

    with open(args.database) as fp:
        db_json = json.load(fp)

    for item in db_json:
        process_command(
            CompileCommand(
                command=item.get("commacsnd"),
                directory=Path(item.get("directory")),
                file=Path(item.get("file")),
            ),
            args.debug,
        )


if __name__ == "__main__":
    main()
