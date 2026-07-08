# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

from pathlib import Path

DEFAULT_GENERATOR = "Ninja"


def repo_root_from_tools_file(file_path: str) -> Path:
    """Return the repository root for scripts that live directly under tools/."""
    return Path(file_path).resolve().parents[1]


def validate_navkit_config(navkit_config: str) -> Path:
    """Return a validated config path relative to config/compiletime."""
    config_path = Path(navkit_config)
    if config_path.is_absolute():
        raise ValueError("--navkit-config must be relative to config/compiletime")
    if any(part == ".." for part in config_path.parts):
        raise ValueError("--navkit-config must not contain '..'")
    if config_path.suffix != ".hpp":
        raise ValueError("--navkit-config must name a .hpp file")
    return config_path


def generator_slug(generator: str) -> str:
    """Return a filesystem-safe, readable generator name for build directories."""
    slug = "".join(char if char.isalnum() else "-" for char in generator).strip("-")
    while "--" in slug:
        slug = slug.replace("--", "-")
    return slug or "generator"


def default_build_dir(
    root: Path, build_type: str, navkit_config: str, generator: str = DEFAULT_GENERATOR
) -> Path:
    """Derive the default build tree from generator, build type, and selected config header."""
    config_path = validate_navkit_config(navkit_config)
    return root / "build" / generator_slug(generator) / build_type / config_path.with_suffix("")


def resolve_build_dir(
    root: Path,
    build_type: str,
    navkit_config: str,
    build_dir_arg: Path | None,
    *,
    generator: str = DEFAULT_GENERATOR,
) -> Path:
    """Resolve an explicit build directory or the generator/config-rooted default."""
    if build_dir_arg is None:
        return default_build_dir(root, build_type, navkit_config, generator)

    build_dir = build_dir_arg if build_dir_arg.is_absolute() else root / build_dir_arg
    resolved = build_dir.resolve()
    if not resolved.is_relative_to(root):
        raise ValueError(f"--build-dir must stay inside the repository: {build_dir_arg}")
    return resolved
