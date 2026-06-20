#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
"""
capitalize_markdown_comments.py

Pre-commit hook that capitalizes the first letter of HTML comments embedded
in Markdown files. These are the only "comment" syntax available in Markdown.

Handles:
  - Single-line HTML comments  : <!-- some text -->  ->  <!-- Some text -->
  - Multi-line HTML comments   : the first alphabetic character in the block
                                  is capitalised; subsequent lines are untouched.

Inline code spans, fenced code blocks, and link/image URLs are NOT modified.

Usage (called by pre-commit):
    python capitalize_markdown_comments.py <file1> [file2 ...]
"""

import re
import sys

# ---------------------------------------------------------------------------
# Tokeniser
# ---------------------------------------------------------------------------

_TOKEN_RE = re.compile(
    r"""
    # --- Fenced code block (``` or ~~~, skip entirely) ---
    (?P<fence>
        ^[ \t]*(?P<fence_char>`{3,}|~{3,})[^\n]*\n   # Opening fence line
        .*?                                          # Content
        ^[ \t]*(?P=fence_char)[^\n]*$                # Closing fence
    )
    |
    # --- Indented code block (4-space or tab indent) ---
    (?P<indented>
        (?:^(?:    |\t)[^\n]*\n)+
    )
    |
    # --- Inline code span ---
    (?P<inline>`+[^`\n]+?`+)
    |
    # --- HTML comment <!-- ... --> ---
    (?P<comment><!--.*?-->)
    """,
    re.VERBOSE | re.DOTALL | re.MULTILINE,
)


def _capitalize_comment_text(text: str) -> str:
    """Uppercase the very first alphabetic character in *text* and enforce exactly one space."""
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
        if len(m.group(0)) <= 2:
            return text
        if m.group(0).lower() == "ros":
            return " ROS" + text[m.end() :]

    for i, ch in enumerate(stripped):
        if ch.isalpha():
            # Enforce exactly one space before the text, then capitalize.
            return " " + stripped[:i] + ch.upper() + stripped[i + 1 :]
    return text


def _process_html_comment(raw: str) -> str:
    """Capitalise text inside an <!-- ... --> comment."""
    assert raw.startswith("<!--") and raw.endswith("-->")
    inner = raw[4:-3]

    lines = inner.split("\n")
    result_lines = []
    capitalized = False

    for line in lines:
        if not capitalized:
            m = re.match(r"^([ \t\-]*)(.*?)(\s*)$", line, re.DOTALL)
            if m:
                lead, body, trail = m.group(1), m.group(2), m.group(3)
                new_body = _capitalize_comment_text(body)
                # _capitalize_comment_text prepends exactly one space; strip
                # any trailing space from lead to avoid double-spacing.
                if new_body != body:
                    lead = lead.rstrip(" \t")
                result_lines.append(lead + new_body + trail)
                if any(c.isalpha() for c in body):
                    capitalized = True
            else:
                result_lines.append(line)
        else:
            result_lines.append(line)

    return "<!--" + "\n".join(result_lines) + "-->"


def process_source(source: str) -> str:
    """Return *source* with all HTML comments capitalised."""
    result = []
    prev_end = 0

    for m in _TOKEN_RE.finditer(source):
        gap = source[prev_end : m.start()]
        result.append(gap)
        prev_end = m.end()

        kind = m.lastgroup
        raw = m.group()

        if kind == "comment":
            result.append(_process_html_comment(raw))
        else:
            # Fence, indented, inline - emit verbatim
            result.append(raw)

    result.append(source[prev_end:])
    return "".join(result)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: capitalize_markdown_comments.py <file> [file ...]", file=sys.stderr)
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
