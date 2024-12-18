#!/usr/bin/env python3
#
# Copyright (c) 2023 Bjarki Arge Andreasen
#
# SPDX-License-Identifier: Apache-2.0
"""
Script to generate iterable sections from JSON encoded dictionary containing a list of annotations.
"""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1]))
from zpp import Annotation, AnnotationType


def get_tagged_items(filepath: str, tags: list[str]) -> list[str]:
    with open(filepath) as fp:
        annotations = [Annotation(**a) for a in json.load(fp)]

    result = [
        a.data.get("name")
        for a in annotations
        if a.attr == AnnotationType.STRUCT and len(a.args) > 0 and a.args[0] in tags
    ]
    # Remove empty strings or annotations missing a name
    return [i for i in result if i]


def gen_ld(filepath: str, items: list, args):
    with open(filepath, "w") as fp:
        for item in items:
            fp.write(f"ITERABLE_SECTION_{args.mem}({item}, {args.align})\n")


def gen_cmake(filepath: str, items: list, args):
    with open(filepath, "w") as fp:
        for item in items:
            if args.mem == "ROM":
                fp.write(
                    f'list(APPEND sections "{{NAME\\;{item}_area\\;'
                    + 'GROUP\\;RODATA_REGION\\;'
                    + f'SUBALIGN\\;{args.align}\\;'
                    + 'NOINPUT\\;TRUE}")\n'
                )
            else:
                fp.write(
                    f'list(APPEND sections "{{NAME\\;{item}_area\\;'
                    + 'GROUP\\;DATA_REGION\\;'
                    + f'SUBALIGN\\;{args.align}\\;'
                    + f'{"ALIGN_WITH_INPUT\\;TRUE\\;" if args.xip else ""}'
                    + 'NOINPUT\\;TRUE}")\n'
                )
            fp.write(
                f'list(APPEND section_settings "{{SECTION\\;{item}_area\\;'
                + 'SORT\\;NAME\\;'
                + 'KEEP\\;TRUE\\;'
                + f'INPUT\\;._{item}.static.*\\;'
                + f'SYMBOLS\\;_{item}_list_start\\;_{item}_list_end}}")\n'
            )
        fp.write(f'set({args.prefix}_SECTIONS         "${{sections}}" CACHE INTERNAL "")\n')
        fp.write(f'set({args.prefix}_SECTION_SETTINGS "${{section_settings}}" CACHE INTERNAL "")\n')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        allow_abbrev=False,
    )

    parser.add_argument("-i", "--input", required=True, help="Path to input list of tags")
    parser.add_argument(
        "-a", "--alignment", dest="align", required=True, help="Iterable section alignment"
    )
    parser.add_argument("-x", "--xip", action="store_true", help="Add if XIP is enabled")
    parser.add_argument(
        "-m",
        "--memory-type",
        dest="mem",
        default="ROM",
        choices=["ROM", "RAM"],
        action="store",
        help="The iterable section's memory type",
    )
    parser.add_argument(
        "-t",
        "--tag",
        required=True,
        dest="tags",
        action="append",
        help="Tag(s) to generate iterable sections for",
    )
    parser.add_argument("-l", "--ld-output", required=True, help="Path to output linker file")
    parser.add_argument(
        "-c", "--cmake-output", required=True, help="Path to CMake linker script inclusion file"
    )
    parser.add_argument(
        "-p", "--cmake-prefix", dest="prefix", required=True, help="Section name prefix"
    )

    return parser.parse_args()


def main():
    args = parse_args()

    items = get_tagged_items(args.input, args.tags)

    gen_ld(args.ld_output, items, args)
    gen_cmake(args.cmake_output, items, args)


if __name__ == "__main__":
    main()
