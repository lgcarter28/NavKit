# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any


JsonObject = dict[str, Any]


def merge_json_object(source: JsonObject, target: JsonObject) -> None:
    for key, value in source.items():
        existing = target.get(key)
        if isinstance(existing, dict) and isinstance(value, dict):
            merge_json_object(value, existing)
        else:
            target[key] = copy.deepcopy(value)


def _reference_chain(stack: list[Path], next_path: Path) -> str:
    return " -> ".join(str(path) for path in [*stack, next_path])


def _resolve_json_value(value: Any, *, containing_file: Path, stack: list[Path]) -> Any:
    if isinstance(value, list):
        return [
            _resolve_json_value(element, containing_file=containing_file, stack=stack)
            for element in value
        ]
    if not isinstance(value, dict):
        return value
    if "components" in value:
        raise ValueError(
            "legacy runtime config 'components' assembly is unsupported; declare each field "
            "inline or with an explicit {'config': '<relative-path>'} reference"
        )
    if "config" in value:
        unexpected = set(value).difference({"config", "overrides"})
        if unexpected:
            raise ValueError(
                "a runtime config reference may contain only 'config' and optional "
                f"'overrides'; unexpected keys: {sorted(unexpected)}"
            )
        reference = value["config"]
        if not isinstance(reference, str):
            raise ValueError("runtime config reference 'config' must be a path string")
        overrides = value.get("overrides", {})
        if not isinstance(overrides, dict):
            raise ValueError("runtime config reference 'overrides' must be an object")

        resolved = _load_runtime_config_impl(containing_file.parent / reference, stack=stack)
        if overrides:
            resolved_overrides = _resolve_json_value(
                overrides, containing_file=containing_file, stack=stack
            )
            if not isinstance(resolved_overrides, dict):
                raise ValueError("runtime config reference overrides must resolve to an object")
            merge_json_object(resolved_overrides, resolved)
        return resolved
    if "overrides" in value:
        raise ValueError(
            "runtime config 'overrides' is valid only beside an explicit 'config' reference"
        )
    return {
        key: _resolve_json_value(member, containing_file=containing_file, stack=stack)
        for key, member in value.items()
    }


def _load_runtime_config_impl(path: Path, *, stack: list[Path]) -> JsonObject:
    resolved_path = path.resolve()
    if not resolved_path.is_file():
        raise ValueError(
            "runtime config reference does not exist; reference chain: "
            f"{_reference_chain(stack, resolved_path)}"
        )
    if resolved_path in stack:
        raise ValueError(
            "runtime config reference cycle detected: "
            f"{_reference_chain(stack, resolved_path)}"
        )

    stack.append(resolved_path)
    try:
        raw = json.loads(resolved_path.read_text(encoding="utf-8"))
        if not isinstance(raw, dict):
            raise ValueError(f"runtime config root must be an object: {resolved_path}")
        resolved = _resolve_json_value(raw, containing_file=resolved_path, stack=stack)
        if not isinstance(resolved, dict):
            raise ValueError(f"runtime config root must resolve to an object: {resolved_path}")
        return resolved
    except (OSError, ValueError, json.JSONDecodeError) as error:
        raise ValueError(f"{error} [while loading {resolved_path}]") from error
    finally:
        stack.pop()


def load_runtime_config(path: Path) -> JsonObject:
    """Load one explicit runtime graph with file-relative references and cycle checks."""

    return _load_runtime_config_impl(path, stack=[])


def resolve_runtime_asset_paths(config: JsonObject, source_config: Path) -> JsonObject:
    """Make relocatable effective configs retain authored scenario-relative asset paths.

    Runtime JSON references are resolved before execution, and the resulting document is
    normally written into the run output directory. Unlike JSON references, ordinary asset
    paths retain the documented main-scenario-relative convention. Canonicalize the currently
    supported CSV trajectory asset before relocating that effective document.
    """

    resolved = copy.deepcopy(config)
    simulation = resolved.get("simulation")
    if not isinstance(simulation, dict):
        return resolved
    source = simulation.get("source")
    if not isinstance(source, dict) or source.get("type") != "csv":
        return resolved
    csv_path = source.get("csv_path")
    if isinstance(csv_path, str):
        path = Path(csv_path)
        if not path.is_absolute():
            source["csv_path"] = str((source_config.resolve().parent / path).resolve())
    return resolved


def runtime_run_name(config: JsonObject) -> str:
    run_name = config.get("run_name")
    if run_name is None:
        raise ValueError("runtime config is missing required 'run_name'")
    if not isinstance(run_name, str):
        raise ValueError("runtime config 'run_name' must be a string")
    return run_name


def runtime_output_dir(config: JsonObject) -> Path:
    output_dir = config.get("output_dir")
    if output_dir is None:
        raise ValueError("runtime config is missing required 'output_dir'")
    if not isinstance(output_dir, str):
        raise ValueError("runtime config 'output_dir' must be a string")
    return Path(output_dir)


def apply_runtime_overrides(
    config: JsonObject, *, output_dir: Path | None = None, run_name: str | None = None
) -> JsonObject:
    updated = copy.deepcopy(config)
    if output_dir is not None:
        updated["output_dir"] = str(output_dir)
        if run_name is None:
            updated["run_name"] = output_dir.name
    if run_name is not None:
        updated["run_name"] = run_name
    return updated


def write_effective_runtime_config(config: JsonObject, output_dir: Path) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / "effective_runtime_config.json"
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    return path
