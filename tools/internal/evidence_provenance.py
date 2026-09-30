# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Generation-time source provenance for reusable qualification evidence."""

from __future__ import annotations

import hashlib
import json
import subprocess
from pathlib import Path

from navkit_analysis.analysis_performance import canonical_json_digest, file_digest
from run_sim import default_swil_executable


def _git_output(root: Path, arguments: list[str]) -> bytes:
    completed = subprocess.run(
        ["git", "-C", str(root), *arguments],
        check=True,
        capture_output=True,
    )
    return completed.stdout


def git_provenance(root: Path) -> dict[str, object]:
    """Describe the exact repository state that generated reusable evidence."""
    resolved_root = root.resolve()
    revision = _git_output(resolved_root, ["rev-parse", "HEAD"]).decode("ascii").strip()
    status = _git_output(
        resolved_root,
        ["status", "--porcelain=v1", "-z", "--untracked-files=all"],
    )
    dirty = bool(status)
    dirty_tree_sha256: str | None = None
    if dirty:
        digest = hashlib.sha256()
        digest.update(b"tracked-diff\0")
        digest.update(_git_output(resolved_root, ["diff", "--binary", "HEAD", "--", "."]))
        digest.update(b"\0status\0")
        digest.update(status)
        untracked_output = _git_output(
            resolved_root,
            ["ls-files", "--others", "--exclude-standard", "-z"],
        )
        for encoded_path in sorted(filter(None, untracked_output.split(b"\0"))):
            relative_path = encoded_path.decode("utf-8", errors="surrogateescape")
            source_path = (resolved_root / relative_path).resolve()
            if not source_path.is_relative_to(resolved_root):
                raise ValueError(f"untracked source path escaped repository: {relative_path}")
            digest.update(b"\0untracked\0")
            digest.update(encoded_path)
            digest.update(b"\0")
            digest.update(bytes.fromhex(file_digest(source_path)))
        dirty_tree_sha256 = digest.hexdigest()
    return {
        "revision": revision,
        "dirty": dirty,
        "dirty_tree_sha256": dirty_tree_sha256,
    }


def build_artifact_provenance(
    resolved_build_dir: Path,
    build_type: str,
) -> dict[str, object]:
    """Fingerprint the manifest and selected application executable used as evidence."""
    build_manifest_path = resolved_build_dir / "navkit_build_manifest.json"
    if not build_manifest_path.is_file():
        raise FileNotFoundError(
            f"missing build manifest in {resolved_build_dir}; build the selected product first"
        )
    loaded_manifest = json.loads(build_manifest_path.read_text(encoding="utf-8"))
    if not isinstance(loaded_manifest, dict):
        raise ValueError(f"build manifest root must be an object: {build_manifest_path}")
    executable = default_swil_executable(resolved_build_dir, build_type)
    if not executable.is_file():
        raise FileNotFoundError(f"missing application executable: {executable}")
    return {
        "directory": str(resolved_build_dir),
        "manifest": {
            "path": str(build_manifest_path),
            "sha256": file_digest(build_manifest_path),
            "canonical_sha256": canonical_json_digest(loaded_manifest),
        },
        "application_executable": {
            "path": str(executable),
            "sha256": file_digest(executable),
            "size_bytes": executable.stat().st_size,
        },
    }
