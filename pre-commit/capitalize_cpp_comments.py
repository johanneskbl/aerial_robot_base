#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
"""
capitalize_comments.py

Pre-commit hook that capitalizes the first letter of every C++ comment
and ensures exactly one whitespace before the first letter.

Handles:
  - Single-line comments  : // some text  ->  // Some text
  - Inline comments       : int x; // note  ->  int x; // Note
  - Block comments        : /* text */  ->  /* Text */
  - Multi-line block comments (each line's leading text is capitalized)

Usage (called by pre-commit):
    python capitalize_comments.py <file1> [file2 ...]
"""

import re
import sys

# ---------------------------------------------------------------------------
# Tokeniser
# ---------------------------------------------------------------------------

# We split the source into alternating "non-comment" and "comment" chunks so
# that we never accidentally capitalise string literals that happen to contain
# comment-like sequences.

_TOKEN_RE = re.compile(
    r"""
    # --- String / char literals (skip entirely) ---
    (?P<string>
        "(?:[^"\\]|\\.)*"       # Double-quoted string
      | '(?:[^'\\]|\\.)*'       # Single-quoted char
    )
    |
    # --- Raw string literal R"delim(...)delim" ---
    (?P<rawstring>
        R"(?P<delim>[^()\\ \t\v\f\n]*)\(
        .*?
        \)(?P=delim)"
    )
    |
    # --- Block comment ---
    (?P<block>/\*.*?\*/)
    |
    # --- Line comment (to end of line, NOT consuming the newline) ---
    (?P<line>//[^\n]*)
    """,
    re.VERBOSE | re.DOTALL,
)


# If a comment's first word is one of these, we assume it is a code-ish comment
# (often commented-out code) and we leave it untouched.
_NO_CAPITALIZE_FIRST_WORDS = frozenset(
    {
        "namespace",
        "clang",
        "include",
        "define",
        "ifdef",
        "ifndef",
        "endif",
        "pragma",
        "using",
        "typedef",
        "struct",
        "class",
        "template",
        "enum",
        "union",
        "return",
        "if",
        "else",
        "for",
        "while",
        "switch",
        "case",
        "break",
        "continue",
        "try",
        "catch",
        "throw",
        "public",
        "private",
        "protected",
        "nullptr",
        "true",
        "false",
        "const",
        "static",
        "virtual",
        "override",
        "final",
        "auto",
        "int",
        "float",
        "double",
        "bool",
        "char",
        "void",
        "std",
        "https",
        "http",
        "www",
        "inline",
        "x",
        "y",
        "z",
    }
)


def _capitalize_comment_text(text: str) -> str:
    """Uppercase the very first alphabetic character in *text* and enforce exactly one space."""
    stripped = text.lstrip(" \t")
    if not stripped:
        return text
    # If the first non-whitespace character is not alphabetic (e.g. '<', '.',
    # '(', '-', '1'), do not attempt capitalization or spacing adjustments.
    if not stripped[0].isalpha():
        return text

    # Do not capitalize if the very first word in the comment looks like code.
    first_word_match = re.match(r"[a-zA-Z_]\w*", stripped)
    if first_word_match:
        first_word = first_word_match.group(0)
        first_word_lower = first_word.lower()
        if "_" in first_word:
            return text
        if len(first_word) <= 2:
            return text
        if stripped[len(first_word) :].startswith("."):
            return text
        if first_word_lower == "ros":
            return " ROS" + stripped[len(first_word) :]
        if first_word_lower in _NO_CAPITALIZE_FIRST_WORDS:
            return text

    for i, ch in enumerate(stripped):
        if ch.isalpha():
            return " " + stripped[:i] + ch.upper() + stripped[i + 1 :]
    return text


def _process_line_comment(raw: str) -> str:
    """Capitalise a // comment, ensuring exactly one space prefix after //."""
    prefix = "//"
    rest = raw[len(prefix) :]
    return prefix + _capitalize_comment_text(rest)


def _process_block_comment(raw: str) -> str:
    """Capitalise text inside a /* ... */ block comment.

    Strategy: capitalise the first alphabetic character in the entire block
    """
    assert raw.startswith("/*") and raw.endswith("*/")
    inner = raw[2:-2]  # Everything between /* and */

    lines = inner.split("\n")
    result_lines = []
    capitalized = False

    for line in lines:
        if not capitalized:
            # Decorative prefix patterns like " * ", " *", leading spaces
            m = re.match(r"^([ \t]*\*?)(.*?)(\s*)$", line, re.DOTALL)
            if m:
                lead, body, trail = m.group(1), m.group(2), m.group(3)
                new_body = _capitalize_comment_text(body)
                # Only strip trailing spaces from lead when _capitalize_comment_text
                # added its own leading space, to avoid double-spacing.
                if body and body.lstrip(" \t") and body.lstrip(" \t")[0].isalpha():
                    if new_body.startswith(" "):
                        lead = lead.rstrip(" \t")
                result_lines.append(lead + new_body + trail)
                if any(c.isalpha() for c in body):
                    capitalized = True
            else:
                result_lines.append(line)
        else:
            result_lines.append(line)

    return "/*" + "\n".join(result_lines) + "*/"


def process_source(source: str) -> str:
    """Return *source* with all comments capitalised."""
    result = []
    prev_end = 0
    last_was_line_comment = False

    for m in _TOKEN_RE.finditer(source):
        gap = source[prev_end : m.start()]
        result.append(gap)
        prev_end = m.end()

        kind = m.lastgroup
        raw = m.group()

        if kind == "line":
            is_continuation = False
            if last_was_line_comment and (not gap or gap.isspace()):
                if gap.count("\n") <= 1:
                    is_continuation = True

            if is_continuation:
                result.append(raw)
            else:
                result.append(_process_line_comment(raw))
            last_was_line_comment = True
        elif kind == "block":
            result.append(_process_block_comment(raw))
            last_was_line_comment = False
        else:
            result.append(raw)
            last_was_line_comment = False

    result.append(source[prev_end:])
    return "".join(result)


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: capitalize_comments.py <file> [file ...]", file=sys.stderr)
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
