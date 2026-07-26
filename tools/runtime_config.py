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
        if key == "components":
            continue
        existing = target.get(key)
        if isinstance(existing, dict) and isinstance(value, dict):
            merge_json_object(value, existing)
        else:
            target[key] = value


def load_runtime_config(path: Path) -> JsonObject:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("runtime config root must be a JSON object")

    merged: JsonObject = {}
    components = raw.get("components", {})
    if components:
        if not isinstance(components, dict):
            raise ValueError("runtime config 'components' must be an object")
        for role, component in components.items():
            if not isinstance(component, str):
                raise ValueError(f"runtime config component '{role}' must be a string")
            merge_json_object(load_runtime_config(path.parent / component), merged)
    merge_json_object(raw, merged)
    return merged


def runtime_run_name(config: JsonObject) -> str:
    run_name = config.get("run_name", "ecef_ins_gnss_lc_gyro_accel_bias_stationary_nominal")
    if not isinstance(run_name, str):
        raise ValueError("runtime config 'run_name' must be a string")
    return run_name


def runtime_output_dir(config: JsonObject) -> Path:
    default_output_dir = f"output/logs/{runtime_run_name(config)}"
    output_dir = config.get("output_dir", default_output_dir)
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
