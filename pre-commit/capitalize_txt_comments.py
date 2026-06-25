#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
"""capitalize_txt_comments.py

Pre-commit hook that capitalizes comment lines in *CMakeLists.txt* only
and ensures exactly one space between the # marker and the comment text.

This repository includes a number of hooks that operate on "*.txt"-suffixed
files. However, "CMakeLists.txt" is a special case: it ends with ".txt" but is
not a plain text document. This hook exists purely to format *CMakeLists.txt*
comment lines without ever touching CMake code.

Behavior:
    - Only files whose basename is exactly "CMakeLists.txt" are processed.
    - All other files (including other ".txt" files) are ignored completely.

Rules (CMakeLists.txt mode):
    - Only full-line comments (start with "#" after optional whitespace).
        - Only comments with whitespace after the leading "#" (or "###") are
            eligible (e.g. "# foo"); lines like "#foo" are treated as commented-out
            code / directives and left untouched.
    - Commented-out code / identifiers (e.g. containing "_", "::", "(") are left
      untouched to avoid breaking builds.
    - Exactly one space is enforced between the final "#" and the comment text.

Usage (called by pre-commit):
    python capitalize_txt_comments.py <file1> [file2 ...]
"""

import re
import sys
from pathlib import Path
from typing import Optional

# Patterns that indicate a line should NOT be auto-capitalized:
#   - Lines starting with a URL scheme
#   - Lines that are purely numeric / bullet / list markers
#   - Lines with leading underscores (variable-like tokens)
_SKIP_RE = re.compile(
    r"""
    ^[\s]*           # Optional leading whitespace
    (?:
        https?://    # URL
      | \d+\.        # Ordered list  "1. item"
      | [-*+]\s      # Unordered bullet
      | \[           # Markdown-style checkbox / link
      | [A-Z_]{2,}   # ALL_CAPS constant or header
    )
    """,
    re.VERBOSE,
)


_SKIP_COMMENT_TEXT_RE = re.compile(
    r"""
    ^(?:
        https?://    # URL
      | \d+\.        # Ordered list  "1. item"
      | [-*+]\s      # Unordered bullet
      | \[           # Markdown-style checkbox / link
      | [A-Z_]{2,}   # ALL_CAPS constant or header
    )
    """,
    re.VERBOSE,
)


def _is_target_file(path: str) -> bool:
    """Return True if *path* should be processed by this hook."""
    return Path(path).name == "CMakeLists.txt"


def _capitalize_first_alpha(text: str) -> str:
    """Capitalize the first alphabetic character in *text* (preserving prefix).

    This is intentionally more general than "capitalize first non-whitespace".
    For separator-style comments like "--- header ---", we still want to
    capitalise the first *alphabetic* character ("--- Header ---").
    """
    m = re.search(r"[a-zA-Z_]\w*", text)
    if m and len(m.group(0)) <= 2:
        return text

    for i, ch in enumerate(text):
        if ch.isalpha():
            return text[:i] + ch.upper() + text[i + 1 :]
    return text


def _looks_like_code_or_identifier(text: str) -> bool:
    """Heuristic: avoid touching commented-out code / targets / identifiers."""
    stripped = text.lstrip(" \t")
    if not stripped:
        return False
    if stripped.startswith("${"):
        return True
    if "::" in stripped or "(" in stripped or ")" in stripped:
        return True
    m = re.match(r"([A-Za-z_][\w:]*)", stripped)
    if not m:
        return False
    token = m.group(1)
    return "_" in token


def _capitalize_line(line: str) -> str:
    """Capitalize the first alphabetic character on the line."""
    stripped = line.lstrip(" \t")
    if not stripped:
        return line
    # If the first non-whitespace character is not alphabetic (e.g. '<', '.',
    # '(', '-', '1'), do not attempt capitalization.
    if not stripped[0].isalpha():
        return line

    # Check for leading word with underscore (variable name / code), function calls, or paths
    m = re.search(r"([a-zA-Z_]\w*)([/\(]?)", line)
    if m:
        if "_" in m.group(1) or m.group(2):
            return line

    # First non-whitespace character is alphabetic, so capitalize that one.
    idx = len(line) - len(stripped)
    return line[:idx] + stripped[0].upper() + stripped[1:]


def process_cmake_source(source: str) -> str:
    """Return *source* with eligible CMake comment lines capitalised.

    Only lines that start with '#' after optional whitespace are considered.
    Exactly one space is enforced between the final '#' and the comment text.
    """
    lines = source.splitlines(keepends=True)
    result: list[str] = []

    for line in lines:
        # Preserve blank/whitespace-only lines
        if not line.strip():
            result.append(line)
            continue

        m = re.match(r"^(\s*#+)(\s*)(.*?)(\r?\n)?$", line)
        if not m:
            result.append(line)
            continue

        hashes, spaces, comment_text, newline = m.groups()
        newline = newline or ""

        # Treat lines like "#add_library(...)" (no whitespace after '#') as
        # commented-out code / directives and do not touch them.
        if spaces == "":
            result.append(line)
            continue

        comment_stripped = comment_text.lstrip(" \t")
        if not comment_stripped:
            result.append(line)
            continue

        if _SKIP_COMMENT_TEXT_RE.match(comment_stripped):
            result.append(line)
            continue

        if _looks_like_code_or_identifier(comment_text):
            result.append(line)
            continue

        updated_comment = _capitalize_first_alpha(comment_text)
        # Enforce exactly one space between the hashes and the comment text.
        result.append(f"{hashes} {updated_comment.lstrip()}{newline}")

    return "".join(result)


def process_source(source: str, *, file_path: Optional[str] = None) -> str:
    """Return processed *source*.

    Only *CMakeLists.txt* is modified. All other files are returned unchanged.
    """
    if not (file_path and _is_target_file(file_path)):
        return source

    return process_cmake_source(source)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: capitalize_txt_comments.py <file> [file ...]", file=sys.stderr)
        return 1

    changed_files = []
    errors = []

    for path in sys.argv[1:]:
        if not _is_target_file(path):
            continue

        try:
            with open(path, "r", encoding="utf-8") as fh:
                original = fh.read()
        except OSError as exc:
            errors.append(f"Cannot read {path}: {exc}")
            continue

        processed = process_source(original, file_path=path)

        if processed != original:
            try:
                with open(path, "w", encoding="utf-8") as fh:
                    fh.write(processed)
                changed_files.append(path)
            except OSError as exc:
                errors.append(f"Cannot write {path}: {exc}")

    if changed_files:
        print("capitalize_comments: capitalised first lines of blocks in:")
        for f in changed_files:
            print(f"  {f}")

    if errors:
        for e in errors:
            print(f"ERROR: {e}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
