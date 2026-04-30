#!/usr/bin/env python3
"""
capitalize_comments.py

Pre-commit hook that capitalizes the first letter of every C++ comment.

Handles:
  - Single-line comments  : // some text  →  // Some text
  - Inline comments       : int x; // note  →  int x; // Note
  - Block comments        : /* text */  →  /* Text */
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
    # --- string / char literals (skip entirely) ---
    (?P<string>
        "(?:[^"\\]|\\.)*"       # double-quoted string
      | '(?:[^'\\]|\\.)*'       # single-quoted char
    )
    |
    # --- raw string literal R"delim(...)delim" ---
    (?P<rawstring>
        R"(?P<delim>[^()\\ \t\v\f\n]*)\(
        .*?
        \)(?P=delim)"
    )
    |
    # --- block comment ---
    (?P<block>/\*.*?\*/)
    |
    # --- line comment (to end of line, NOT consuming the newline) ---
    (?P<line>//[^\n]*)
    """,
    re.VERBOSE | re.DOTALL,
)


def _capitalize_comment_text(text: str) -> str:
    """Uppercase the very first alphabetic character in *text*."""
    # Do not capitalize if the very first word in the comment contains an
    # underscore, as it is likely a variable name or commented-out code.
    m = re.search(r'[a-zA-Z_]\w*', text)
    if m:
        if '_' in m.group(0):
            return text
        if m.group(0).lower() == 'ros':
            return text[:m.start()] + 'ROS' + text[m.end():]

    for i, ch in enumerate(text):
        if ch.isalpha():
            return text[:i] + ch.upper() + text[i + 1 :]
    return text


def _process_line_comment(raw: str) -> str:
    """Capitalise a // comment, preserving the // prefix and any leading spaces."""
    # raw starts with '//'
    prefix = "//"
    rest = raw[len(prefix) :]
    # preserve leading whitespace after //
    stripped = rest.lstrip(" \t")
    leading = rest[: len(rest) - len(stripped)]
    return prefix + leading + _capitalize_comment_text(stripped)


def _process_block_comment(raw: str) -> str:
    """Capitalise text inside a /* … */ block comment.

    Strategy: capitalise the first alphabetic character in the entire block
    """
    # Split into /* , body , */
    assert raw.startswith("/*") and raw.endswith("*/")
    inner = raw[2:-2]  # everything between /* and */

    lines = inner.split("\n")
    result_lines = []
    capitalized = False

    for line in lines:
        if not capitalized:
            # Decorative prefix patterns like " * ", " *", leading spaces
            m = re.match(r"^([ \t]*\*?[ \t]*)(.*?)(\s*)$", line, re.DOTALL)
            if m:
                lead, body, trail = m.group(1), m.group(2), m.group(3)
                result_lines.append(lead + _capitalize_comment_text(body) + trail)
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

    for m in _TOKEN_RE.finditer(source):
        # Emit the literal text between the previous token and this one
        result.append(source[prev_end : m.start()])
        prev_end = m.end()

        kind = m.lastgroup
        raw = m.group()

        if kind == "line":
            result.append(_process_line_comment(raw))
        elif kind == "block":
            result.append(_process_block_comment(raw))
        else:
            # string, rawstring – emit verbatim
            result.append(raw)

    # Tail after the last token
    result.append(source[prev_end:])
    return "".join(result)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

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