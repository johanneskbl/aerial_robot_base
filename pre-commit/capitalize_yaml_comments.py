#!/usr/bin/env python3
"""
capitalize_yaml_comments.py

Pre-commit hook that capitalizes the first letter of every YAML comment.

Handles:
  - Full-line comments  : # some text  ->  # Some text
  - Inline comments     : key: value  # note  ->  key: value  # Note

YAML string values (quoted or block scalars) are never modified.

Usage (called by pre-commit):
    python capitalize_yaml_comments.py <file1> [file2 ...]
"""

import re
import sys


# ---------------------------------------------------------------------------
# Tokeniser
# ---------------------------------------------------------------------------

# We tokenise enough YAML syntax to skip over string literals safely.
# YAML is complex; we cover the common cases: double-quoted, single-quoted,
# and block/flow scalars are skipped by not matching them as comments.

_TOKEN_RE = re.compile(
    r"""
    # --- double-quoted YAML string ---
    (?P<dquote>"(?:[^"\\]|\\.)*")
    |
    # --- single-quoted YAML string ('' is an escaped single quote) ---
    (?P<squote>'(?:[^']|'')*')
    |
    # --- YAML comment (# to end of line, NOT consuming the newline) ---
    # Only match # that is preceded by start-of-line or whitespace, to avoid
    # matching # inside unquoted values like anchor names (&foo).
    (?P<comment>(?:^|(?<=\s))\#[^\n]*)
    """,
    re.VERBOSE | re.MULTILINE,
)


def _capitalize_comment_text(text: str) -> str:
    """Uppercase the very first alphabetic character in *text*."""
    stripped = text.lstrip(" \t")
    if not stripped:
        return text
    # If the first non-whitespace character is not alphabetic (e.g. '<', '.',
    # '(', '-', '1'), do not attempt capitalization.
    if not stripped[0].isalpha():
        return text

    m = re.search(r"[a-zA-Z_]\w*", text)
    if m:
        if "_" in m.group(0):
            return text
        if m.group(0).lower() == "ros":
            return text[: m.start()] + "ROS" + text[m.end() :]

    for i, ch in enumerate(text):
        if ch.isalpha():
            return text[:i] + ch.upper() + text[i + 1 :]
    return text


def _process_comment(raw: str) -> str:
    """Capitalise a # comment, preserving the # prefix and any leading spaces."""
    # Raw may start with whitespace captured by the lookbehind workaround -
    # find the actual '#' character.
    hash_idx = raw.index("#")
    before_hash = raw[:hash_idx]  # Any captured leading whitespace (empty or space)
    rest = raw[hash_idx + 1 :]  # Everything after '#'

    stripped = rest.lstrip(" \t")
    leading = rest[: len(rest) - len(stripped)]

    # Special directives - leave alone.
    lower = stripped.lower()
    if lower.startswith(("noqa", "type:", "fmt:", "yaml-language-server")):
        return raw

    return before_hash + "#" + leading + _capitalize_comment_text(stripped)


def process_source(source: str) -> str:
    """Return *source* with all # comments capitalised."""
    result = []
    prev_end = 0
    last_was_comment = False

    for m in _TOKEN_RE.finditer(source):
        gap = source[prev_end : m.start()]
        result.append(gap)
        prev_end = m.end()

        kind = m.lastgroup
        raw = m.group()

        if kind == "comment":
            is_continuation = False
            if last_was_comment and (not gap or gap.isspace()):
                if gap.count("\n") <= 1:
                    is_continuation = True

            if is_continuation:
                result.append(raw)
            else:
                result.append(_process_comment(raw))
            last_was_comment = True
        else:
            result.append(raw)
            last_was_comment = False

    result.append(source[prev_end:])
    return "".join(result)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: capitalize_yaml_comments.py <file> [file ...]", file=sys.stderr)
        return 1

    changed_files = []
    errors = []

    for path in sys.argv[1:]:
        try:
            with open(path, "r", encoding="utf-8") as fh:
                original = fh.read()
        except OSError as exc:
            errors.append(f"Cannot read {path}: {exc}")
            continue

        processed = process_source(original)

        if processed != original:
            try:
                with open(path, "w", encoding="utf-8") as fh:
                    fh.write(processed)
                changed_files.append(path)
            except OSError as exc:
                errors.append(f"Cannot write {path}: {exc}")

    if changed_files:
        print("capitalize_comments: capitalised comments in:")
        for f in changed_files:
            print(f"  {f}")

    if errors:
        for e in errors:
            print(f"ERROR: {e}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
