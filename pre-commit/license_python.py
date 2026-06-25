#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
"""license_python.py

Pre-commit hook that ensures a short SPDX license header is present in Python
sources.

Targets (via pre-commit config): .py

Behavior:
    - Reads LICENSE from the project root (sibling of the pre-commit/ directory)
    - Parses license type and copyright year from LICENSE
    - Prepends only:
            # SPDX-License-Identifier: <id>
            # Copyright (c) <year>, DRAGON Laboratory, The University of Tokyo
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
    "SPDX-License-Identifier:",
    "DRAGON Laboratory, The University of Tokyo",
)


# Files under these repo-relative directories will be ignored by this hook.
#
# Use this to keep vendored / copied sources unchanged.
# Example (paths are relative to the LICENSE location / this hook's repo root):
#   _EXCLUDE_DIRS = ("third_party", "vendor")
_EXCLUDE_DIRS: tuple[str, ...] = ("aerial_robot_nerve/spinal/mcu_project/lib/My_Lib",)

_ENCODING_RE = re.compile(r"^#.*coding[:=][ \t]*([-\w.]+)")


def _repo_root() -> Path:
    # Pre-commit/license_python.py -> pre-commit -> repo root
    return Path(__file__).resolve().parent.parent


def _excluded_entries() -> tuple[str, ...]:
    """Return normalized excluded entries.

    Keeps the configuration ergonomic and tolerant if `_EXCLUDE_DIRS` is
    accidentally set to a plain string.
    """

    if isinstance(_EXCLUDE_DIRS, str):
        return (_EXCLUDE_DIRS,)
    return tuple(_EXCLUDE_DIRS)


def _is_excluded_path(path: Path) -> bool:
    """Return True if path is under any excluded directory."""

    repo_root = _repo_root()
    try:
        resolved = (repo_root / path).resolve() if not path.is_absolute() else path.resolve()
    except OSError:
        # If resolution fails (broken symlink, permission, etc.), do not exclude.
        resolved = (repo_root / path) if not path.is_absolute() else path

    for rel in _excluded_entries():
        excluded_root = (repo_root / rel).resolve()
        try:
            if resolved.is_relative_to(excluded_root):
                return True
        except AttributeError:
            # Python < 3.9 fallback
            try:
                resolved.relative_to(excluded_root)
                return True
            except ValueError:
                pass

    return False


def _read_license_file() -> str:
    license_path = _repo_root() / "LICENSE"
    text = license_path.read_text(encoding="utf-8")
    return text.replace("\r\n", "\n").replace("\r", "\n")


def _has_license_header(source: str) -> bool:
    head = source[:8000]
    return all(needle in head for needle in _SIGNATURE_NEEDLES)


def _is_spdx_identifier_line(line: str) -> bool:
    return line.startswith("# SPDX-License-Identifier:")


def _is_dragon_copyright_line(line: str) -> bool:
    # Keep this narrow so we don't rewrite unrelated copyright notices.
    return line.startswith("# Copyright") and ("DRAGON Laboratory, The University of Tokyo" in line)


def _find_spdx_header_in_rest(rest_lines: list[str]) -> tuple[int, int] | None:
    """Return (start_idx, end_idx_exclusive) for an SPDX header at top of rest.

    Searches only near the top to avoid false positives deeper in the file.
    Allows a few leading blank lines before the header.
    """

    i = 0
    while i < len(rest_lines) and rest_lines[i].strip() == "":
        i += 1

    # Look only at the first handful of lines after initial blanks.
    limit = min(len(rest_lines), i + 20)
    for j in range(i, limit):
        if _is_spdx_identifier_line(rest_lines[j]):
            start = j
            end = j + 1
            if end < len(rest_lines) and _is_dragon_copyright_line(rest_lines[end]):
                end += 1
            return start, end

    return None


def _parse_spdx_license_identifier(license_text: str) -> str:
    # Prefer explicit strings from the LICENSE; keep a small, conservative
    # mapping rather than guessing.
    upper = license_text.upper()
    if "BSD-3" in upper or "BSD 3" in upper:
        return "BSD-3-Clause"
    if "BSD-2" in upper or "BSD 2" in upper:
        return "BSD-2-Clause"
    raise ValueError("Unsupported or unrecognized license type in LICENSE")


_COPYRIGHT_YEAR_RE = re.compile(
    r"copyright\s*\(\s*c\s*\)\s*(\d{4})(?:\s*-\s*(\d{4}))?",
    flags=re.IGNORECASE,
)


def _parse_copyright_year(license_text: str) -> str:
    match = _COPYRIGHT_YEAR_RE.search(license_text)
    if not match:
        raise ValueError("Cannot find a copyright year in LICENSE")
    start_year, end_year = match.group(1), match.group(2)
    return end_year or start_year


def _format_spdx_header(spdx_id: str, year: str) -> str:
    # Must match the exact desired header lines.
    return (
        f"# SPDX-License-Identifier: {spdx_id}\n"
        f"# Copyright (c) {year}, DRAGON Laboratory, The University of Tokyo\n"
    )


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
        # (Covered by branch above, but kept for clarity)
        preamble.append(lines[idx])
        idx += 1

    # Keep a single blank line after shebang/encoding if it already exists.
    if idx < len(lines) and lines[idx].strip() == "":
        preamble.append(lines[idx])
        idx += 1

    return preamble, lines[idx:]


def _apply_to_source(source: str, header_text: str) -> str:
    bom = ""
    if source.startswith("\ufeff"):
        bom = "\ufeff"
        source = source[1:]

    source = source.replace("\r\n", "\n").replace("\r", "\n")
    lines = source.split("\n")

    preamble, rest = _split_python_preamble(lines)
    header_lines = header_text.replace("\r\n", "\n").replace("\r", "\n").split("\n")
    # Drop any trailing empty lines; we'll control blank-line spacing explicitly.
    while header_lines and header_lines[-1] == "":
        header_lines.pop()
    header_lines = [ln for ln in header_lines if ln != ""]
    if not header_lines:
        # Should never happen, but fail safe.
        return bom + source.rstrip("\n") + "\n"

    # Update in-place if an SPDX header already exists near the top of the file,
    # otherwise insert it right after the preamble.
    header_span = _find_spdx_header_in_rest(rest)
    if header_span is not None:
        start, end = header_span
        rest = rest[:start] + header_lines + rest[end:]
        # Ensure exactly one blank line after the header block.
        k = start + len(header_lines)
        while k < len(rest) and rest[k].strip() == "":
            del rest[k]
    else:
        # Strip leading blanks so we don't accumulate spacing before/after header.
        while rest and rest[0].strip() == "":
            rest.pop(0)
        rest = header_lines + rest

    # If file is empty (or only preamble), ensure we don't create extra leading
    # blank lines beyond the standard header separator.
    rebuilt = "\n".join(preamble)
    if rebuilt and not rebuilt.endswith("\n"):
        rebuilt += "\n"

    rebuilt += "\n".join(rest)

    # `Split("\n")` drops the final newline information; normalize to one.
    return bom + rebuilt.rstrip("\n") + "\n"


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: license_python.py <file> [file ...]", file=sys.stderr)
        return 1

    try:
        license_text = _read_license_file()
        spdx_id = _parse_spdx_license_identifier(license_text)
        year = _parse_copyright_year(license_text)
        header_text = _format_spdx_header(spdx_id, year)
    except OSError as exc:
        print(f"ERROR: Cannot read LICENSE: {exc}", file=sys.stderr)
        return 1
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    changed_files: list[str] = []
    errors: list[str] = []

    for path_str in sys.argv[1:]:
        path = Path(path_str)

        if _is_excluded_path(path):
            continue

        try:
            original = path.read_text(encoding="utf-8")
        except OSError as exc:
            errors.append(f"Cannot read {path_str}: {exc}")
            continue

        processed = _apply_to_source(original, header_text)

        if processed != original:
            try:
                path.write_text(processed, encoding="utf-8")
                changed_files.append(path_str)
            except OSError as exc:
                errors.append(f"Cannot write {path_str}: {exc}")

    if changed_files:
        print("license_python: added SPDX header to:")
        for f in changed_files:
            print(f"  {f}")

    if errors:
        for e in errors:
            print(f"ERROR: {e}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
