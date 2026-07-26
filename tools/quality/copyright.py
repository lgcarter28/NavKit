# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import argparse
import sys
from pathlib import Path

COPYRIGHT_YEAR = "2026"
COPYRIGHT_OWNER = "William Gordon Carter"

CPP_HEADER = (
    f"// Copyright (c) {COPYRIGHT_YEAR} {COPYRIGHT_OWNER}.\n"
    "// All Rights Reserved.\n"
)

HASH_HEADER = (
    f"# Copyright (c) {COPYRIGHT_YEAR} {COPYRIGHT_OWNER}.\n"
    "# All Rights Reserved.\n"
)

EXCLUDE_DIRS = {
    ".git",
    ".vs",
    ".vscode",
    ".venv",
    "artifacts",
    "build",
    "install",
    "out",
    "output",
    "__pycache__",
}

EXCLUDE_PATH_PARTS = {
    ("data", "logs"),
}

CPP_EXTENSIONS = {
    ".c",
    ".cc",
    ".cpp",
    ".cxx",
    ".h",
    ".hh",
    ".hpp",
    ".hxx",
    ".ipp",
    ".tpp",
}

PY_EXTENSIONS = {".py"}
CMAKE_EXTENSIONS = {".cmake"}
SPECIAL_FILENAMES = {"CMakeLists.txt", "conanfile.py"}

KNOWN_COPYRIGHT_MARKERS = (
    "Copyright (c)",
    "All Rights Reserved",
)


def repo_root() -> Path:
    script = Path(__file__).resolve()
    if script.parent.name == "quality":
        return script.parents[2]
    return Path.cwd().resolve()


def has_excluded_path_part(path: Path) -> bool:
    parts = path.parts
    for excluded in EXCLUDE_PATH_PARTS:
        if len(excluded) <= len(parts):
            for i in range(len(parts) - len(excluded) + 1):
                if tuple(parts[i : i + len(excluded)]) == excluded:
                    return True
    return False


def should_exclude(path: Path) -> bool:
    if any(part in EXCLUDE_DIRS for part in path.parts):
        return True
    return has_excluded_path_part(path)


def is_supported_file(path: Path) -> bool:
    if path.name in SPECIAL_FILENAMES:
        return True
    suffix = path.suffix.lower()
    return suffix in CPP_EXTENSIONS or suffix in PY_EXTENSIONS or suffix in CMAKE_EXTENSIONS


def find_files(root: Path) -> list[Path]:
    files: list[Path] = []
    for path in root.rglob("*"):
        if not path.is_file():
            continue
        if should_exclude(path):
            continue
        if is_supported_file(path):
            files.append(path)
    return sorted(files)


def header_for(path: Path) -> str:
    if path.suffix.lower() in CPP_EXTENSIONS:
        return CPP_HEADER
    return HASH_HEADER


def split_lines_preserve(text: str) -> list[str]:
    return text.splitlines(keepends=True)


def line_has_copyright(line: str) -> bool:
    return any(marker in line for marker in KNOWN_COPYRIGHT_MARKERS)


def first_non_shebang_encoding_index(lines: list[str]) -> int:
    """Return insertion index after shebang/encoding lines for script-like files."""
    idx = 0

    if idx < len(lines) and lines[idx].startswith("#!"):
        idx += 1

    # PEP 263 encoding comment must be on line 1 or 2. Preserve it above header.
    if idx < len(lines) and "coding" in lines[idx] and ("#" in lines[idx]):
        idx += 1

    # Preserve a single blank line after shebang/encoding if present.
    if idx > 0 and idx < len(lines) and lines[idx].strip() == "":
        idx += 1

    return idx


def has_header(text: str, path: Path) -> bool:
    lines = split_lines_preserve(text)
    idx = first_non_shebang_encoding_index(lines) if header_for(path).startswith("#") else 0
    search_window = "".join(lines[idx : idx + 6])
    return all(marker in search_window for marker in KNOWN_COPYRIGHT_MARKERS)


def remove_existing_top_header(lines: list[str], path: Path) -> list[str]:
    """Remove existing simple copyright header near top, if present.

    This keeps the tool idempotent and allows owner/year text to be changed later.
    It only removes a small two-line copyright/all-rights-reserved block near the
    insertion point, not arbitrary license blocks elsewhere in the file.
    """
    idx = first_non_shebang_encoding_index(lines) if header_for(path).startswith("#") else 0
    end = min(len(lines), idx + 6)

    remove_start: int | None = None
    remove_end: int | None = None

    for i in range(idx, end):
        if i < len(lines) and "Copyright (c)" in lines[i]:
            remove_start = i
            remove_end = i + 1
            while remove_end < len(lines) and remove_end < i + 4:
                if "All Rights Reserved" in lines[remove_end]:
                    remove_end += 1
                    break
                if lines[remove_end].strip() == "":
                    break
                remove_end += 1
            break

    if remove_start is None or remove_end is None:
        return lines

    # Remove one trailing blank line after the old header; the new header adds one.
    if remove_end < len(lines) and lines[remove_end].strip() == "":
        remove_end += 1

    return lines[:remove_start] + lines[remove_end:]


def with_header(text: str, path: Path) -> str:
    lines = split_lines_preserve(text)
    lines = remove_existing_top_header(lines, path)

    insert_at = first_non_shebang_encoding_index(lines) if header_for(path).startswith("#") else 0
    header_lines = split_lines_preserve(header_for(path) + "\n")

    new_lines = lines[:insert_at] + header_lines + lines[insert_at:]
    return "".join(new_lines)


def check_files(files: list[Path]) -> list[Path]:
    missing: list[Path] = []
    for path in files:
        text = path.read_text(encoding="utf-8", errors="replace")
        if not has_header(text, path):
            missing.append(path)
    return missing


def write_files(files: list[Path]) -> list[Path]:
    changed: list[Path] = []
    for path in files:
        original = path.read_text(encoding="utf-8", errors="replace")
        updated = with_header(original, path)
        if updated != original:
            path.write_text(updated, encoding="utf-8", newline="")
            changed.append(path)
    return changed


def print_paths(paths: list[Path], root: Path) -> None:
    for path in paths:
        try:
            print(path.relative_to(root).as_posix())
        except ValueError:
            print(path.as_posix())


def main() -> int:
    parser = argparse.ArgumentParser(description="Check or insert NavKit copyright headers.")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true", help="Check files without modifying them.")
    mode.add_argument("--write", action="store_true", help="Insert/update headers in supported files.")
    parser.add_argument(
        "--root",
        type=Path,
        default=None,
        help="Repository root. Defaults to parent of tools/ or current working directory.",
    )
    args = parser.parse_args()

    root = args.root.resolve() if args.root is not None else repo_root()
    files = find_files(root)

    if args.check:
        missing = check_files(files)
        if missing:
            print("Files missing copyright headers:")
            print_paths(missing, root)
            return 1
        print(f"All {len(files)} supported files contain copyright headers.")
        return 0

    changed = write_files(files)
    if changed:
        print("Updated copyright headers:")
        print_paths(changed, root)
    else:
        print(f"All {len(files)} supported files already up to date.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
