#!/usr/bin/env python3
"""license_python.py

Pre-commit hook that ensures the project LICENSE text is present as a header in
Python sources.

Targets (via pre-commit config): .py

Behavior:
  - Reads LICENSE from the project root (sibling of the pre-commit/ directory)
  - Inserts it as a block of `# ...` comments
  - Preserves a shebang and PEP-263 encoding cookie (if present) at the top
  - Idempotent: does nothing if the header already exists

Usage (called by pre-commit):
	python pre-commit/license_python.py <file1> [file2 ...]
"""

from __future__ import annotations

import re
import sys
from pathlib import Path


_SIGNATURE_NEEDLES = (
    "Software License Agreement (BSD-3 License)",
    "DRAGON Laboratory",
    "All rights reserved.",
    "THIS SOFTWARE IS PROVIDED BY",
)

_ENCODING_RE = re.compile(r"^#.*coding[:=][ \t]*([-\w.]+)")


def _repo_root() -> Path:
    # Pre-commit/license_python.py -> pre-commit -> repo root
    return Path(__file__).resolve().parent.parent


def _read_license_text() -> str:
    license_path = _repo_root() / "LICENSE"
    text = license_path.read_text(encoding="utf-8")
    text = text.replace("\r\n", "\n").replace("\r", "\n")
    return text.rstrip("\n") + "\n"


def _has_license_header(source: str) -> bool:
    head = source[:8000]
    return all(needle in head for needle in _SIGNATURE_NEEDLES)


def _format_license_comment_block(license_text: str) -> str:
    lines = license_text.splitlines()
    out: list[str] = []
    for line in lines:
        if line.strip() == "":
            out.append("#")
        else:
            out.append(f"# {line}")
    out.append("")
    return "\n".join(out)


def _split_python_preamble(lines: list[str]) -> tuple[list[str], list[str]]:
    """Return (preamble_lines, rest_lines).

    Preamble includes:
      - optional shebang on first line
      - optional encoding cookie on first or second line
      - a single blank line after the preamble is preserved as part of preamble
    """

    idx = 0
    preamble: list[str] = []

    if idx < len(lines) and lines[idx].startswith("#!"):
        preamble.append(lines[idx])
        idx += 1

    if idx < len(lines) and _ENCODING_RE.match(lines[idx]):
        preamble.append(lines[idx])
        idx += 1
    elif idx == 1 and idx < len(lines) and _ENCODING_RE.match(lines[idx]):
        # (covered by branch above, but kept for clarity)
        preamble.append(lines[idx])
        idx += 1

    # Keep a single blank line after shebang/encoding if it already exists.
    if idx < len(lines) and lines[idx].strip() == "":
        preamble.append(lines[idx])
        idx += 1

    return preamble, lines[idx:]


def _apply_to_source(source: str, license_text: str) -> str:
    bom = ""
    if source.startswith("\ufeff"):
        bom = "\ufeff"
        source = source.lstrip("\ufeff")

    if _has_license_header(source):
        return bom + source

    source = source.replace("\r\n", "\n").replace("\r", "\n")
    lines = source.split("\n")

    preamble, rest = _split_python_preamble(lines)
    header = _format_license_comment_block(license_text)

    # If file is empty (or only preamble), ensure we don't create extra leading
    # blank lines beyond the standard header separator.
    rebuilt = "\n".join(preamble)
    if rebuilt and not rebuilt.endswith("\n"):
        rebuilt += "\n"

    rebuilt += header
    rebuilt += "\n".join(rest)

    # `split("\n")` drops the final newline information; normalize to one.
    return bom + rebuilt.rstrip("\n") + "\n"


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: license_python.py <file> [file ...]", file=sys.stderr)
        return 1

    try:
        license_text = _read_license_text()
    except OSError as exc:
        print(f"ERROR: Cannot read LICENSE: {exc}", file=sys.stderr)
        return 1

    changed_files: list[str] = []
    errors: list[str] = []

    for path_str in sys.argv[1:]:
        path = Path(path_str)
        try:
            original = path.read_text(encoding="utf-8")
        except OSError as exc:
            errors.append(f"Cannot read {path_str}: {exc}")
            continue

        processed = _apply_to_source(original, license_text)

        if processed != original:
            try:
                path.write_text(processed, encoding="utf-8")
                changed_files.append(path_str)
            except OSError as exc:
                errors.append(f"Cannot write {path_str}: {exc}")

    if changed_files:
        print("license_python: added LICENSE header to:")
        for f in changed_files:
            print(f"  {f}")

    if errors:
        for e in errors:
            print(f"ERROR: {e}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
