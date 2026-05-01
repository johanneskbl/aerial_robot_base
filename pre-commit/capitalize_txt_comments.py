#!/usr/bin/env python3
"""
capitalize_txt_comments.py

Pre-commit hook that capitalizes the first letter of every line in plain
text (.txt) files, treating each non-empty line as a "comment" / sentence
that should start with a capital letter.

Rules:
  - Empty or whitespace-only lines are left untouched.
  - Lines that start with a special marker (e.g. URLs, list bullets, numbers)
    are left untouched.
  - The first alphabetic character on each qualifying line is uppercased.

Usage (called by pre-commit):
    python capitalize_txt_comments.py <file1> [file2 ...]
"""

import re
import sys


# Patterns that indicate a line should NOT be auto-capitalized:
#   - Lines starting with a URL scheme
#   - Lines that are purely numeric / bullet / list markers
#   - Lines with leading underscores (variable-like tokens)
_SKIP_RE = re.compile(
    r"""
    ^[\s]*           # optional leading whitespace
    (?:
        https?://    # URL
      | \d+\.        # ordered list  "1. item"
      | [-*+]\s      # unordered bullet
      | \[           # markdown-style checkbox / link
      | [A-Z_]{2,}   # ALL_CAPS constant or header
    )
    """,
    re.VERBOSE,
)


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


def process_source(source: str) -> str:
    """Return *source* with the first letter of each line capitalised."""
    lines = source.splitlines(keepends=True)
    result = []
    last_was_processed = False

    for line in lines:
        stripped = line.strip()
        if not stripped:
            result.append(line)
            last_was_processed = False
            continue

        if _SKIP_RE.match(line):
            result.append(line)
            last_was_processed = False
            continue

        # Continuation lines: if the previous line was processed and this
        # line does not look like the start of a new sentence (no leading
        # whitespace reset, line doesn't end a sentence) - leave it.
        # For plain text we apply capitalization only to the first line of
        # each "block" (paragraph).  A blank line separates blocks.
        if last_was_processed:
            result.append(line)
        else:
            result.append(_capitalize_line(line))
            last_was_processed = True

    return "".join(result)


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
