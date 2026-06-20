#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
"""license_cpp.py

Pre-commit hook that ensures the project LICENSE text is present as a header in
C / C++ sources.

Targets (via pre-commit config): .c, .cc, .cpp, .h, .hpp

Behavior:
    - Reads LICENSE from the project root (sibling of the pre-commit/ directory)
    - Prepends it as a /* ... */ block comment
    - Preserves the repo's C++ modeline ("// -*- mode: c++ -*-") above the license
    - Idempotent: does nothing if the header already exists and matches LICENSE
    - If a top-of-file license statement exists but differs from LICENSE,
            replaces it with the canonical LICENSE header

Usage (called by pre-commit):
        python pre-commit/license_cpp.py <file1> [file2 ...]
"""

from __future__ import annotations
import sys
from pathlib import Path

_SIGNATURE_NEEDLES = (
    "Software License Agreement (BSD-3 License)",
    "DRAGON Laboratory",
    "All rights reserved.",
    "THIS SOFTWARE IS PROVIDED BY",
)


# Files under these repo-relative directories will be ignored by this hook.
#
# Use this to keep vendored / copied sources unchanged.
# Example (paths are relative to the LICENSE location / this hook's repo root):
#   _EXCLUDE_DIRS = ("third_party", "vendor")
_EXCLUDE_DIRS: tuple[str, ...] = ("aerial_robot_nerve/spinal/mcu_project/lib/My_Lib",)


# These needles are used to *detect* an existing license header that should be
# replaced. They are intentionally broader than _SIGNATURE_NEEDLES so we can
# upgrade older variants (e.g., "BSD License" without "-3").
_LICENSE_DETECTION_NEEDLES = (
    "Software License Agreement",
    "Redistribution and use",
    "THIS SOFTWARE IS PROVIDED BY",
)


_MODELINE = "// -*- mode: c++ -*-"


def _strip_one_prefix(text: str, prefix: str) -> tuple[bool, str]:
    if text.startswith(prefix):
        return True, text[len(prefix) :]
    return False, text


def _extract_comment_text(comment: str) -> str:
    """Best-effort extraction of plain text from a top-of-file comment.

    Supports:
      - C-style block comments: /* ... */ and /** ... */
      - C++ single-line comments: // ... (consecutive)

    Returned text is normalized to '\n' newlines and does not include a trailing
    newline.
    """
    comment = comment.replace("\r\n", "\n").replace("\r", "\n")
    comment = comment.strip("\n")

    # /* ... */ Comment
    if comment.lstrip().startswith("/*"):
        left = comment.find("/*")
        right = comment.rfind("*/")
        if left == -1 or right == -1 or right < left + 2:
            return ""
        inner = comment[left + 2 : right]
        lines: list[str] = []
        for line in inner.split("\n"):
            line = line.rstrip("\n")
            # Ignore indentation used to align the '*' decoration, but preserve any
            # indentation that is part of the license text itself.
            decorated = line.lstrip("\t ")
            has_star, decorated = _strip_one_prefix(decorated, "*")
            if has_star:
                _, decorated = _strip_one_prefix(decorated, " ")
                lines.append(decorated.rstrip())
            else:
                lines.append(decorated.rstrip())
        return "\n".join(lines).strip("\n")

    # // Comment
    lines_out: list[str] = []
    for raw_line in comment.split("\n"):
        line = raw_line.lstrip("\t ")
        has_slashes, rest = _strip_one_prefix(line, "//")
        if not has_slashes:
            # Not a pure //-only block
            return ""
        rest = rest.lstrip(" ")
        lines_out.append(rest.rstrip())
    return "\n".join(lines_out).strip("\n")


def _looks_like_license_comment(comment: str) -> bool:
    extracted = _extract_comment_text(comment)
    if not extracted:
        return False
    # Only look at the beginning to avoid matching e.g. a large file banner.
    head = extracted[:8000]
    return all(needle in head for needle in _LICENSE_DETECTION_NEEDLES)


def _find_top_license_comment_span(source: str) -> tuple[int, int, str] | None:
    """Find an existing license comment at the top of the file.

    Only considers the first non-whitespace token in the file to avoid replacing
    license text that appears later (e.g., in docs).

    Returns (start_index, end_index, comment_text).
    """
    # Skip leading whitespace (including blank lines). Keep it in the output when
    # replacing, but don't let it affect detection.
    idx = 0
    while idx < len(source) and source[idx] in " \t\n":
        idx += 1

    if source.startswith("/*", idx):
        end = source.find("*/", idx + 2)
        if end == -1:
            return None
        end += 2
        comment = source[idx:end]
        # Use a strong/unique trigger to avoid overwriting unrelated comments.
        # Detect both the canonical BSD-3 header and older variants.
        if _looks_like_license_comment(comment):
            return idx, end, comment
        return None

    if source.startswith("//", idx):
        end = idx
        lines: list[str] = []
        while end < len(source):
            line_end = source.find("\n", end)
            if line_end == -1:
                line_end = len(source)
            line = source[end:line_end]
            if not line.lstrip(" \t").startswith("//"):
                break
            lines.append(line)
            end = line_end
            if end < len(source) and source[end] == "\n":
                end += 1
        comment = "\n".join(lines)
        # Use a strong/unique trigger to avoid overwriting unrelated comments.
        # Detect both the canonical BSD-3 header and older variants.
        if _looks_like_license_comment(comment):
            return idx, end, comment
        return None

    return None


def _repo_root() -> Path:
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


def _read_license_text() -> str:
    license_path = _repo_root() / "LICENSE"
    text = license_path.read_text(encoding="utf-8")
    # Normalize newlines for stable output.
    text = text.replace("\r\n", "\n").replace("\r", "\n")
    return text.rstrip("\n") + "\n"


def _has_license_header(source: str) -> bool:
    # Look only at the beginning of the file to avoid false positives in docs
    head = source[:8000]
    return all(needle in head for needle in _SIGNATURE_NEEDLES)


def _format_license_block(license_text: str) -> str:
    lines = license_text.splitlines()

    # Typical C/C++ header style
    out = ["/*"]
    for line in lines:
        if line.strip() == "":
            out.append(" *")
        else:
            out.append(f" * {line}")
    out.append(" */")
    out.append("")
    return "\n".join(out)


def _apply_to_source(source: str, license_text: str) -> str:
    bom = ""
    if source.startswith("\ufeff"):
        bom = "\ufeff"
        source = source.lstrip("\ufeff")

    # Preserve the Emacs/Vim modeline above the license
    modeline_prefix = ""
    if source.lower().startswith(_MODELINE):
        line_end = source.find("\n")
        if line_end == -1:
            modeline_prefix = source + "\n"
            source = ""
        else:
            modeline_prefix = source[: line_end + 1]
            source = source[line_end + 1 :]

    # If a license header exists at the top of the file, replace it only if its
    # content differs from LICENSE
    existing = _find_top_license_comment_span(source)
    if existing is not None:
        start, end, comment = existing
        existing_text = _extract_comment_text(comment)
        wanted_text = license_text.strip("\n")
        if existing_text.strip("\n") == wanted_text:
            return bom + modeline_prefix + source

        header = _format_license_block(license_text)
        suffix = source[end:]
        # Avoid accumulating blank lines when replacing
        suffix = suffix.lstrip("\n")
        return bom + modeline_prefix + source[:start] + header + suffix

    # No detectable top-of-file license statement: prepend the license
    if _has_license_header(source):
        return bom + modeline_prefix + source

    header = _format_license_block(license_text)
    return bom + modeline_prefix + header + source


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: license_cpp.py <file> [file ...]", file=sys.stderr)
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

        if _is_excluded_path(path):
            continue

        try:
            original = path.read_text(encoding="utf-8")
        except OSError as exc:
            errors.append(f"Cannot read {path_str}: {exc}")
            continue

        original_norm = original.replace("\r\n", "\n").replace("\r", "\n")
        processed = _apply_to_source(original_norm, license_text)

        if processed != original_norm:
            try:
                # Preserve trailing newline behavior of the processed output
                path.write_text(processed, encoding="utf-8")
                changed_files.append(path_str)
            except OSError as exc:
                errors.append(f"Cannot write {path_str}: {exc}")

    if changed_files:
        print("license_cpp: added LICENSE header to:")
        for f in changed_files:
            print(f"  {f}")

    if errors:
        for e in errors:
            print(f"ERROR: {e}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
