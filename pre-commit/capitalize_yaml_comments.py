#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
"""
capitalize_yaml_comments.py

Pre-commit hook that capitalizes the first letter of every YAML comment
and ensures exactly one space between the # marker and the comment text.

Handles:
  - Full-line comments  : # some text  ->  # Some text
  - Inline comments     : key: value  # note  ->  key: value  # Note

YAML string values (quoted or block scalars) are never modified.

Exceptions (spacing and capitalization are NOT enforced):
  - Directives: noqa, type:, fmt:, yaml-language-server
  - Command-like comments (sudo, apt, git, etc.)

Usage (called by pre-commit):
    python capitalize_yaml_comments.py <file1> [file2 ...]
"""

import re
import sys

# ---------------------------------------------------------------------------
# Heuristics
# ---------------------------------------------------------------------------

# If a comment starts with one of these tokens, it's likely a shell command
# snippet that should remain lowercase for copy/paste.
_COMMAND_WORDS = frozenset(
    {
        "sudo",
        "apt",
        "apt-get",
        "dnf",
        "yum",
        "pacman",
        "brew",
        "pip",
        "pip3",
        "python",
        "python3",
        "ros2",
        "rosdep",
        "colcon",
        "git",
        "cmake",
        "make",
        "ninja",
        "docker",
        "podman",
        "systemctl",
        "journalctl",
        "ls",
        "cd",
        "cp",
        "mv",
        "rm",
        "mkdir",
        "ln",
        "chmod",
        "chown",
        "cat",
        "echo",
        "grep",
        "sed",
        "awk",
        "find",
        "xargs",
        "curl",
        "wget",
        "tar",
        "unzip",
    }
)


def _looks_like_command_comment(text: str) -> bool:
    stripped = text.lstrip(" \t")
    if not stripped:
        return False

    first = stripped.split(None, 1)[0]
    first = first.strip("`")
    first = first.rstrip(",:;")
    first_lower = first.lower()

    if first_lower in _COMMAND_WORDS:
        return True

    # Relative/absolute script invocation like: ./script.sh --flag
    if first_lower.startswith("./") or first_lower.startswith("/") or first_lower.startswith("~/"):
        return True

    return False


# ---------------------------------------------------------------------------
# Tokeniser
# ---------------------------------------------------------------------------

# We tokenise enough YAML syntax to skip over string literals safely.
# YAML is complex; we cover the common cases: double-quoted, single-quoted,
# and block/flow scalars are skipped by not matching them as comments.

_TOKEN_RE = re.compile(
    r"""
    # --- Double-quoted YAML string ---
    (?P<dquote>"(?:[^"\\]|\\.)*")
    |
    # --- Single-quoted YAML string ('' is an escaped single quote) ---
    (?P<squote>'(?:[^']|'')*')
    |
    # --- YAML comment (# to end of line, NOT consuming the newline) ---
    # Only match # that is preceded by start-of-line or whitespace, to avoid
    # Matching # inside unquoted values like anchor names (&foo).
    (?P<comment>(?:^|(?<=\s))\#[^\n]*)
    """,
    re.VERBOSE | re.MULTILINE,
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
        if m.group(0).lower() == "ros":
            return " ROS" + text[m.end() :]

    for i, ch in enumerate(stripped):
        if ch.isalpha():
            # Enforce exactly one space before the text, then capitalize.
            return " " + stripped[:i] + ch.upper() + stripped[i + 1 :]
    return text


def _process_comment(raw: str) -> str:
    """Capitalise a # comment, enforcing exactly one space after # and before text."""
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

    # Likely copy/pastable command snippet (e.g. "sudo apt ") - leave alone.
    if _looks_like_command_comment(stripped):
        return raw

    new_text = _capitalize_comment_text(stripped)
    # _capitalize_comment_text returns a string starting with exactly one space
    # when it makes a change. We discard the original 'leading' whitespace
    # so the result is always "# <one space><text>".
    if new_text != stripped:
        return before_hash + "#" + new_text
    # No capitalization was applied; still enforce the single space.
    if not stripped:
        # Empty comment body  leave as-is.
        return raw
    return before_hash + "#" + " " + stripped


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
