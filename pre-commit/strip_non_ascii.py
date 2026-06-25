#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
"""strip_non_ascii.py

Pre-commit hook that removes non-ASCII / non-printable characters from text files.

Motivation:
    Prevent accidental inclusion of characters like Chinese/Japanese glyphs or other
    non-ASCII symbols in committed sources.

Behavior:
    - Operates on the files passed by pre-commit.
    - Skips likely-binary files (detected via NUL byte in the first chunk).
    - Keeps: TAB (\t), LF (\n), CR (\r), printable ASCII 0x20..0x7E, and selected
        exception characters.
    - Removes: everything else.

Usage (called by pre-commit):
    python pre-commit/strip_non_ascii.py <file1> [file2 ...]
"""

from __future__ import annotations
import sys

_EXCEPTION_CHARACTERS = {"ä", "ö", "ü", "Ä", "Ö", "Ü"}
_PRINTABLE_ASCII = {chr(codepoint) for codepoint in range(0x20, 0x7F)}
_ALLOWED_WHITESPACE = {"\t", "\n", "\r"}
_ALLOWED_CHARACTERS = _PRINTABLE_ASCII | _ALLOWED_WHITESPACE | _EXCEPTION_CHARACTERS


def _is_probably_binary(data: bytes) -> bool:
    return b"\x00" in data


def _filter_bytes(data: bytes) -> tuple[bytes, int]:
    text = data.decode("utf-8", errors="surrogateescape")
    removed = 0
    out = []
    for character in text:
        if character in _ALLOWED_CHARACTERS:
            out.append(character)
        else:
            removed += 1
    return "".join(out).encode("utf-8"), removed


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: strip_non_ascii.py <file> [file ...]", file=sys.stderr)
        return 1

    changed: list[tuple[str, int]] = []
    skipped: list[str] = []
    errors: list[str] = []

    for path in sys.argv[1:]:
        try:
            with open(path, "rb") as fh:
                original = fh.read()
        except OSError as exc:
            errors.append(f"Cannot read {path}: {exc}")
            continue

        if _is_probably_binary(original[:8192]):
            skipped.append(path)
            continue

        filtered, removed = _filter_bytes(original)
        if removed == 0:
            continue

        try:
            with open(path, "wb") as fh:
                fh.write(filtered)
            changed.append((path, removed))
        except OSError as exc:
            errors.append(f"Cannot write {path}: {exc}")

    if changed:
        print("strip_non_ascii: removed non-ASCII / non-printable characters from:")
        for path, removed in changed:
            print(f"  {path} ({removed} character(s) removed)")

    if skipped:
        print("strip_non_ascii: skipped likely-binary files:")
        for path in skipped:
            print(f"  {path}")

    if errors:
        for e in errors:
            print(f"ERROR: {e}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
