#!/usr/bin/env python3
"""
capitalize_python_comments.py

Pre-commit hook that capitalizes the first letter of every Python comment.

Handles:
  - Single-line comments  : # some text  ->  # Some text
  - Inline comments       : x = 1  # note  ->  x = 1  # Note
  - Block strings used as docstrings / multi-line comments are NOT touched
    (they are string literals, not comments).

Usage (called by pre-commit):
    python capitalize_python_comments.py <file1> [file2 ...]
"""

import re
import sys


# ---------------------------------------------------------------------------
# Tokeniser
# ---------------------------------------------------------------------------

# Split source into alternating non-comment / comment chunks so we never
# accidentally capitalise string literals that look like comments.

_TOKEN_RE = re.compile(
    r"""
    # --- triple-quoted strings (must come before single-quoted) ---
    (?P<triple>
        \"\"\"(?:[^"\\]|\\.|"(?!""))*\"\"\"   # triple double-quoted
      | \'\'\'(?:[^'\\]|\\.|'(?!''))*\'\'\'   # triple single-quoted
    )
    |
    # --- single / double quoted string literals ---
    (?P<string>
        "(?:[^"\\]|\\.)*"
      | '(?:[^'\\]|\\.)*'
    )
    |
    # --- Python comment (# to end of line, NOT consuming the newline) ---
    (?P<comment>\#[^\n]*)
    """,
    re.VERBOSE | re.DOTALL,
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

    # Do not capitalize if the first word contains an underscore
    # (likely a variable name or commented-out code).
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
    prefix = "#"
    rest = raw[len(prefix) :]
    # Preserve leading whitespace after #
    stripped = rest.lstrip(" \t")
    leading = rest[: len(rest) - len(stripped)]

    # Shebangs, type-ignore, noqa, pylint, mypy, fmt directives - leave alone.
    lower = stripped.lower()
    if stripped.startswith("!") or lower.startswith(("type:", "noqa", "pylint", "mypy:", "fmt:")):
        return raw

    return prefix + leading + _capitalize_comment_text(stripped)


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

            # Section-header comments (# ---...) are never continuations;
            # they always start a new block and should be capitalised.
            comment_body = raw[1:].lstrip(" \t")
            if comment_body.startswith("---"):
                is_continuation = False

            if is_continuation:
                result.append(raw)
            else:
                result.append(_process_comment(raw))
            last_was_comment = True
        else:
            # Triple, string – emit verbatim
            result.append(raw)
            last_was_comment = False

    result.append(source[prev_end:])
    return "".join(result)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: capitalize_python_comments.py <file> [file ...]", file=sys.stderr)
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
