#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
"""
capitalize_python_comments.py

Pre-commit hook that capitalizes the first letter of every Python comment
and ensures exactly one space between the # marker and the comment text.

Handles:
  - Single-line comments  : # some text  ->  # Some text
  - Inline comments       : x = 1  # note  ->  x = 1  # Note
    - Comments that start with code-ish keywords (e.g. "if", "for", "def") are
        left untouched to avoid rewriting commented-out code.
    - Regex verbose comments inside triple-quoted patterns passed to
        re.compile(..., re.VERBOSE|re.X, ...) are also capitalised.
  - Block strings used as docstrings / multi-line comments are NOT touched
    (they are string literals, not comments).

Exceptions (spacing and capitalization are NOT enforced):
  - Shebangs (#!)
  - Directives: type:, noqa, pylint, mypy:, fmt:

Usage (called by pre-commit):
    python capitalize_python_comments.py <file1> [file2 ...]
"""

import re
import sys
from typing import List, Optional

# ---------------------------------------------------------------------------
# Tokeniser
# ---------------------------------------------------------------------------

# Split source into alternating non-comment / comment chunks so we never
# accidentally capitalise string literals that look like comments.

_TOKEN_RE = re.compile(
    r"""
    # --- Triple-quoted strings (must come before single-quoted) ---
    (?P<triple>
        \"\"\"(?:[^"\\]|\\.|"(?!""))*\"\"\"   # Triple double-quoted
      | \'\'\'(?:[^'\\]|\\.|'(?!''))*\'\'\'   # Triple single-quoted
    )
    |
    # --- Single / double quoted string literals ---
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


# If a comment's first word is one of these, we assume it is a code-ish comment
# (often commented-out code) and we leave it untouched.
_NO_CAPITALIZE_FIRST_WORDS = frozenset(
    {
        # Python keywords / common statement starters
        "def",
        "class",
        "import",
        "from",
        "as",
        "return",
        "if",
        "elif",
        "else",
        "for",
        "while",
        "try",
        "except",
        "finally",
        "with",
        "lambda",
        "yield",
        "pass",
        "break",
        "continue",
        "raise",
        "assert",
        "del",
        "global",
        "nonlocal",
        "async",
        "await",
        "match",
        "case",
        # Common builtins / literals often seen in commented-out code
        "true",
        "false",
        "none",
        "self",
        "int",
        "float",
        "bool",
        "str",
        "list",
        "dict",
        "tuple",
        "set",
        # Common URL starters (avoid touching links)
        "https",
        "http",
        "www",
        "x",
        "y",
        "z",
        "fmt",
    }
)


def _capitalize_comment_text(text: str) -> str:
    """Uppercase the very first alphabetic character in *text* and enforce exactly one space."""
    leading_len = len(text) - len(text.lstrip(" \t"))
    stripped = text[leading_len:]
    if not stripped:
        return text

    # Do not capitalize if the very first word in the comment looks like code.
    first_word_match = re.match(r"[a-zA-Z_]\w*", stripped)
    if first_word_match:
        first_word = first_word_match.group(0)
        first_word_lower = first_word.lower()
        if "_" in first_word:
            return text
        if stripped[len(first_word) :].startswith("."):
            return text
        if first_word_lower == "ros":
            return " ROS" + stripped[len(first_word) :]
        if first_word_lower in _NO_CAPITALIZE_FIRST_WORDS:
            return text

    # Find the first alphabetic character anywhere in the string. We allow
    # leading punctuation (e.g. "--- header") but avoid changing patterns like
    # "1st" where a leading digit is semantically meaningful.
    first_alpha_idx = next((i for i, ch in enumerate(stripped) if ch.isalpha()), None)
    if first_alpha_idx is None:
        return text
    if any(ch.isdigit() for ch in stripped[:first_alpha_idx]):
        return text

    # Enforce exactly one space before the text (prefix before first alpha),
    # then capitalize.
    pre_alpha = stripped[:first_alpha_idx]
    capitalized_char = stripped[first_alpha_idx].upper()
    rest = stripped[first_alpha_idx + 1 :]
    return " " + pre_alpha + capitalized_char + rest


def _process_comment(raw: str) -> str:
    """Capitalise a # comment, enforcing exactly one space after # and before text."""
    prefix = "#"
    rest = raw[len(prefix) :]
    # Preserve leading whitespace after #
    stripped = rest.lstrip(" \t")
    leading = rest[: len(rest) - len(stripped)]

    # Shebangs, type-ignore, noqa, pylint, mypy, fmt directives - leave alone.
    lower = stripped.lower()
    if stripped.startswith("!") or lower.startswith(("type:", "noqa", "pylint", "mypy:", "fmt:")):
        return raw

    new_text = _capitalize_comment_text(stripped)
    # _capitalize_comment_text returns a string starting with exactly one space
    # when it makes a change (or returns the original). We discard 'leading'
    # so the result is always "# <one space><text>".
    if new_text != stripped:
        return prefix + new_text
    # No capitalization was applied; still enforce the single space.
    if not stripped:
        # Empty comment body - leave as-is.
        return raw
    return prefix + " " + stripped


def _looks_like_verbose_re_compile(preceding: str, following: str) -> bool:
    """Heuristic: detect re.compile(<string>, ..., re.VERBOSE|re.X, ...)."""
    # Search within a small tail window for performance on large files.
    window_start = max(0, len(preceding) - 5000)
    idx = preceding.rfind("re.compile", window_start)
    if idx < 0:
        return False

    # Ensure we are inside the argument list for this re.compile call.
    after = preceding[idx:]
    open_paren = after.find("(")
    if open_paren < 0:
        return False
    # If we can already see a ')' before the current token, then this call is
    # closed and the triple string is unrelated.
    if ")" in after[open_paren:]:
        return False

    args_before_token = after[open_paren + 1 :]

    post = following[:800]
    close_paren = post.find(")")
    if close_paren == -1:
        close_paren = len(post)
    flags_chunk = post[:close_paren]
    return (
        ("re.VERBOSE" in args_before_token)
        or ("re.X" in args_before_token)
        or ("re.VERBOSE" in flags_chunk)
        or ("re.X" in flags_chunk)
    )


def _is_escaped(text: str, idx: int) -> bool:
    """Return True if text[idx] is escaped by an odd number of backslashes."""
    backslashes = 0
    j = idx - 1
    while j >= 0 and text[j] == "\\":
        backslashes += 1
        j -= 1
    return (backslashes % 2) == 1


def _find_verbose_regex_comment_start(line: str) -> Optional[int]:
    """Find start index of a verbose-regex comment (#...), or None.

    Approximates Python's re.VERBOSE rules:
      - '#' starts a comment unless escaped \\# or inside a character class.
    """
    in_class = False
    for i, ch in enumerate(line):
        if ch == "[" and not _is_escaped(line, i) and not in_class:
            in_class = True
            continue
        if ch == "]" and in_class and not _is_escaped(line, i):
            in_class = False
            continue
        if ch == "#" and not in_class and not _is_escaped(line, i):
            return i
    return None


def _process_verbose_regex_comments_in_triple_string(raw: str) -> str:
    """Capitalise verbose-regex comments inside a triple-quoted string token."""
    quote = raw[:3]
    if quote not in ('"""', "'''") or not raw.endswith(quote):
        return raw

    body = raw[3:-3]
    out_lines: List[str] = []

    for line in body.splitlines(keepends=True):
        core = line.rstrip("\r\n")
        eol = line[len(core) :]

        comment_start = _find_verbose_regex_comment_start(core)
        if comment_start is None:
            out_lines.append(core + eol)
            continue

        before = core[:comment_start]
        comment_raw = core[comment_start:]
        out_lines.append(before + _process_comment(comment_raw) + eol)

    return quote + "".join(out_lines) + quote


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
            if kind == "triple" and _looks_like_verbose_re_compile(source[: m.start()], source[m.end() :]):
                result.append(_process_verbose_regex_comments_in_triple_string(raw))
            else:
                # Triple, string  emit verbatim
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
