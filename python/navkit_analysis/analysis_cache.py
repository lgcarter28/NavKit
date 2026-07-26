# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Deterministic cache-manifest helpers for offline analysis artifacts."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Mapping

from navkit_analysis.analysis_performance import canonical_json_digest


def cache_fingerprint(kind: str, inputs: Mapping[str, object]) -> str:
    """Return a typed, stable fingerprint for one derived analysis artifact."""
    return canonical_json_digest({"kind": kind, "inputs": dict(inputs)})


def cache_is_current(marker_path: Path, fingerprint: str, outputs: list[Path]) -> bool:
    """Return whether a marker and every expected output match one fingerprint."""
    if not marker_path.is_file() or not all(path.is_file() for path in outputs):
        return False
    try:
        value = json.loads(marker_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return False
    return isinstance(value, dict) and value.get("fingerprint") == fingerprint


def cached_output_paths(marker_path: Path, fingerprint: str) -> list[Path] | None:
    """Return verified cache outputs from one marker, or ``None`` when stale."""
    if not marker_path.is_file():
        return None
    try:
        value = json.loads(marker_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None
    if not isinstance(value, dict) or value.get("fingerprint") != fingerprint:
        return None
    outputs = value.get("outputs")
    if not isinstance(outputs, list) or not all(isinstance(path, str) for path in outputs):
        return None
    paths = [Path(path) for path in outputs]
    return paths if all(path.is_file() for path in paths) else None


def write_cache_marker(
    marker_path: Path,
    *,
    kind: str,
    fingerprint: str,
    inputs: Mapping[str, object],
    outputs: list[Path],
) -> None:
    """Write provenance for a safely reusable derived artifact family."""
    marker_path.parent.mkdir(parents=True, exist_ok=True)
    marker_path.write_text(
        json.dumps(
            {
                "kind": kind,
                "fingerprint": fingerprint,
                "inputs": dict(inputs),
                "outputs": [str(path.resolve()) for path in outputs],
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
