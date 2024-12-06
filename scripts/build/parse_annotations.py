#!/usr/bin/env python3

# Copyright (c) 2024 Basalte bv
# SPDX-License-Identifier: Apache-2.0

import argparse
import json
import os
import re
import shlex
import subprocess

ANNOTATION = re.compile(
    r'__attribute__\s*\(\(annotate\(\s*"(.*)"\s*\)\)\)\s+struct\s+([a-zA-Z0-9_]+)'
)


def process_command(command: str):
    parser = argparse.ArgumentParser()

    if os.name == "nt":
        command = command.replace("\\", "\\\\") # Fix windows paths

    # Use the argument parser to drop the output argument and keep everything else
    parser.add_argument("-o", "--output")
    _, command_remaining = parser.parse_known_args(shlex.split(command))

    # Generate the source code as produced by the preprocessor
    ir_output = subprocess.check_output(command_remaining + ['-E']).decode()

    for line in ir_output.splitlines():
        match = ANNOTATION.search(line)
        if match is None:
            continue

        print(f"{match.group(1)},{match.group(2)}")


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
            process_command(item["command"])


if __name__ == "__main__":
    main()
