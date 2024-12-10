#!/usr/bin/env python3

# Copyright (c) 2024 Basalte bv
# SPDX-License-Identifier: Apache-2.0

import argparse
import json
import os
import re
import shlex
import subprocess
from pathlib import Path

ANNOTATION = re.compile(
    r'__attribute__\s*\(\(annotate\(\s*"(.*)"\s*\)\)\)\s+struct\s+([a-zA-Z0-9_]+)'
)

def process_command(command: str, file: Path, directory: Path):
    parser = argparse.ArgumentParser()

    # Only parse c files
    if file.suffix != ".c":
        return

    if os.name == "nt":
        command = command.replace("\\", "\\\\") # Fix windows paths

    # Use the argument parser to drop the output argument and keep everything else
    parser.add_argument("-o", "--output")
    parser.add_argument("-DZPP", action="store_true", dest="zpp")
    args, command_remaining = parser.parse_known_args(shlex.split(command))

    if not args.zpp:
        return

    print(f"zpp {file}", directory / f"{args.output}.i")

    # Generate the source code as produced by the preprocessor
    src = subprocess.check_output(command_remaining + ["-E", "-P", "-D__ZPP__"]).decode()

    # debug only
    with open(directory / f"{args.output}.i", "w") as fp:
        fp.write(src)


def parse_args():
    global args
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        allow_abbrev=False,
    )

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
        if "command" in item:
            process_command(item["command"], Path(item["file"]), Path(item["directory"]))


if __name__ == "__main__":
    main()
