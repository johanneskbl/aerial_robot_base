#!/usr/bin/env python3
"""
capitalize_xml_comments.py

Pre-commit hook that capitalizes the first letter of every XML/Xacro comment.

Handles:
  - Single-line XML comments  : <!-- some text -->  ->  <!-- Some text -->
  - Multi-line XML comments   : the first alphabetic character in the block
                                 is capitalised; subsequent lines are untouched.

XML CDATA sections, attribute values, and element text content are NOT modified.

Works for both .xml and .xacro files.

Usage (called by pre-commit):
    python capitalize_xml_comments.py <file1> [file2 ...]
"""

import re
import sys


# ---------------------------------------------------------------------------
# Tokeniser
# ---------------------------------------------------------------------------

_TOKEN_RE = re.compile(
    r"""
    # --- CDATA section (skip entirely) ---
    (?P<cdata><!\[CDATA\[.*?\]\]>)
    |
    # --- XML comment <!-- ... --> ---
    (?P<comment><!--.*?-->)
    |
    # --- quoted attribute value (skip) ---
    (?P<dquote>"[^"]*")
    |
    (?P<squote>'[^']*')
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


def _process_xml_comment(raw: str) -> str:
    """Capitalise text inside an <!-- ... --> comment."""
    assert raw.startswith("<!--") and raw.endswith("-->")
    inner = raw[4:-3]  # Everything between <!-- and -->

    lines = inner.split("\n")
    result_lines = []
    capitalized = False

    for line in lines:
        if not capitalized:
            # Strip decorative leading dashes or spaces
            m = re.match(r"^([ \t\-]*)(.*?)(\s*)$", line, re.DOTALL)
            if m:
                lead, body, trail = m.group(1), m.group(2), m.group(3)
                new_body = _capitalize_comment_text(body)
                result_lines.append(lead + new_body + trail)
                if any(c.isalpha() for c in body):
                    capitalized = True
            else:
                result_lines.append(line)
        else:
            result_lines.append(line)

    return "<!--" + "\n".join(result_lines) + "-->"


def process_source(source: str) -> str:
    """Return *source* with all XML comments capitalised."""
    result = []
    prev_end = 0

    for m in _TOKEN_RE.finditer(source):
        gap = source[prev_end : m.start()]
        result.append(gap)
        prev_end = m.end()

        kind = m.lastgroup
        raw = m.group()

        if kind == "comment":
            result.append(_process_xml_comment(raw))
        else:
            # Cdata, dquote, squote – emit verbatim
            result.append(raw)

    result.append(source[prev_end:])
    return "".join(result)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: capitalize_xml_comments.py <file> [file ...]", file=sys.stderr)
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
