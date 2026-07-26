# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Small, dependency-free helpers for offline analysis performance evidence."""

from __future__ import annotations

import hashlib
import json
import os
import time
from pathlib import Path
from typing import Iterable, Mapping


def canonical_json_digest(payload: Mapping[str, object]) -> str:
    """Return a stable SHA-256 digest for one JSON-compatible mapping."""
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def file_digest(path: Path, block_bytes: int = 1024 * 1024) -> str:
    """Return the SHA-256 digest of one input artifact without loading it all at once."""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(block_bytes):
            digest.update(block)
    return digest.hexdigest()


def input_manifest_digest(source: Path, excluded_paths: Iterable[Path] = ()) -> dict[str, object]:
    """Describe and hash analysis inputs beneath ``source`` deterministically."""
    excluded = {path.resolve() for path in excluded_paths}
    generated_directories = {
        "summary",
        "figures",
        "interactive_figures",
        "consistency_figures",
        "consistency_reports",
    }
    files = []
    for path in sorted(source.rglob("*")):
        relative_parts = path.relative_to(source).parts
        if (
            not path.is_file()
            or path.resolve() in excluded
            or generated_directories.intersection(relative_parts)
            or path.suffix.lower() not in {".csv", ".json"}
        ):
            continue
        files.append(path)
    entries: list[dict[str, object]] = []
    for path in files:
        stat = path.stat()
        entries.append(
            {
                "path": path.relative_to(source).as_posix(),
                "size_bytes": stat.st_size,
                "sha256": file_digest(path),
            }
        )
    payload: dict[str, object] = {"source": str(source.resolve()), "files": entries}
    return {
        "sha256": canonical_json_digest(payload),
        "file_count": len(entries),
        "total_bytes": sum(int(entry["size_bytes"]) for entry in entries),
    }


def tree_size_bytes(root: Path) -> int:
    """Return the recursive file size below ``root`` when it exists."""
    if not root.exists():
        return 0
    if root.is_file():
        return root.stat().st_size
    return sum(path.stat().st_size for path in root.rglob("*") if path.is_file())


def process_memory_bytes() -> int | None:
    """Return current resident process memory when the host exposes it cheaply."""
    if os.name == "nt":
        try:
            import ctypes
            from ctypes import wintypes

            class ProcessMemoryCounters(ctypes.Structure):
                """Windows PROCESS_MEMORY_COUNTERS_EX layout for working-set data."""

                _fields_ = [
                    ("cb", wintypes.DWORD),
                    ("PageFaultCount", wintypes.DWORD),
                    ("PeakWorkingSetSize", ctypes.c_size_t),
                    ("WorkingSetSize", ctypes.c_size_t),
                    ("QuotaPeakPagedPoolUsage", ctypes.c_size_t),
                    ("QuotaPagedPoolUsage", ctypes.c_size_t),
                    ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t),
                    ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
                    ("PagefileUsage", ctypes.c_size_t),
                    ("PeakPagefileUsage", ctypes.c_size_t),
                    ("PrivateUsage", ctypes.c_size_t),
                ]

            counters = ProcessMemoryCounters()
            counters.cb = ctypes.sizeof(counters)
            kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
            psapi = ctypes.WinDLL("psapi", use_last_error=True)
            kernel32.GetCurrentProcess.restype = wintypes.HANDLE
            psapi.GetProcessMemoryInfo.argtypes = (
                wintypes.HANDLE,
                ctypes.c_void_p,
                wintypes.DWORD,
            )
            psapi.GetProcessMemoryInfo.restype = wintypes.BOOL
            process = kernel32.GetCurrentProcess()
            ok = psapi.GetProcessMemoryInfo(
                process,
                ctypes.byref(counters),
                counters.cb,
            )
            return int(counters.WorkingSetSize) if ok else None
        except (AttributeError, OSError):
            return None
    try:
        import resource

        value = int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
        return value if value > 0 else None
    except (ImportError, AttributeError):
        return None


class StageTimer:
    """Collect monotonic elapsed-time and optional resident-memory observations."""

    def __init__(self) -> None:
        self._started_s = time.perf_counter()
        self._last_mark_s = self._started_s
        self._marks: dict[str, dict[str, float | int | None]] = {}

    def mark(self, name: str) -> None:
        """Record elapsed time and resident memory for one completed stage."""
        now_s = time.perf_counter()
        self._marks[name] = {
            "elapsed_s": now_s - self._last_mark_s,
            "total_elapsed_s": now_s - self._started_s,
            "process_memory_bytes": process_memory_bytes(),
        }
        self._last_mark_s = now_s

    def snapshot(self) -> dict[str, dict[str, float | int | None]]:
        """Return a copy suitable for JSON/HDF5 metadata."""
        return dict(self._marks)
