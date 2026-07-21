# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import argparse
import concurrent.futures
import copy
import hashlib
import json
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from navkit_build_dirs import DEFAULT_GENERATOR
from perf_artifacts import DEFAULT_NAVKIT_CONFIG
from runtime_config import JsonObject, load_runtime_config


CAMPAIGN_SCHEMA = "navkit.monte_carlo_campaign.v1"
RUN_SCHEMA = "navkit.monte_carlo_run.v1"
SUPPORTED_SEED_POLICY = "derive_all"


@dataclass(frozen=True)
class CampaignConfig:
    campaign_path: Path
    campaign_name: str
    nominal_config: Path
    run_count: int
    start_index: int
    master_seed: int
    seed_policy: str
    build_type: str
    parallel_jobs: int
    max_plot_points: int | None
    output_root: Path
    run_analysis: bool
    navkit_config: str
    generator: str
    build_dir: Path | None


@dataclass(frozen=True)
class RunPlan:
    run_index: int
    run_name: str
    run_dir: Path
    input_path: Path
    effective_config: JsonObject
    derived_seeds: dict[str, int]
    command: list[str]


@dataclass(frozen=True)
class RunResult:
    run_index: int
    run_name: str
    run_dir: Path
    input_path: Path
    derived_seeds: dict[str, int]
    command: list[str]
    return_code: int
    elapsed_s: float
    status: str


def require_object(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"expected '{path}' to be an object")
    return value


def require_string(value: dict[str, Any], key: str, path: str) -> str:
    item = value.get(key)
    if not isinstance(item, str):
        raise ValueError(f"missing required string '{path}.{key}'")
    return item


def require_int(value: dict[str, Any], key: str, path: str) -> int:
    item = value.get(key)
    if not isinstance(item, int):
        raise ValueError(f"missing required integer '{path}.{key}'")
    return item


def optional_int(value: dict[str, Any], key: str, default: int, path: str) -> int:
    item = value.get(key, default)
    if not isinstance(item, int):
        raise ValueError(f"expected '{path}.{key}' to be an integer")
    return item


def optional_bool(value: dict[str, Any], key: str, default: bool, path: str) -> bool:
    item = value.get(key, default)
    if not isinstance(item, bool):
        raise ValueError(f"expected '{path}.{key}' to be a boolean")
    return item


def optional_string(value: dict[str, Any], key: str, default: str, path: str) -> str:
    item = value.get(key, default)
    if not isinstance(item, str):
        raise ValueError(f"expected '{path}.{key}' to be a string")
    return item


def resolve_relative_to_config(config_path: Path, candidate: str) -> Path:
    path = Path(candidate)
    if path.is_absolute():
        return path
    return (config_path.parent / path).resolve()


def load_campaign_config(
    campaign_path: Path,
    *,
    output_root_override: Path | None,
    build_type_override: str | None,
    run_count_override: int | None,
    start_index_override: int | None,
    parallel_jobs_override: int | None,
    max_plot_points_override: int | None,
    navkit_config: str,
    generator: str,
    build_dir: Path | None,
) -> CampaignConfig:
    raw = json.loads(campaign_path.read_text(encoding="utf-8"))
    root = require_object(raw, "root")

    campaign_type = root.get("type", "monte_carlo_campaign")
    if campaign_type != "monte_carlo_campaign":
        raise ValueError("Monte Carlo campaign config 'type' must be 'monte_carlo_campaign'")

    campaign_name = require_string(root, "campaign_name", "root")
    nominal_config = resolve_relative_to_config(
        campaign_path, require_string(root, "nominal_config", "root")
    )

    runs = require_object(root.get("runs"), "runs")
    run_count = run_count_override if run_count_override is not None else require_int(runs, "count", "runs")
    start_index = (
        start_index_override if start_index_override is not None else optional_int(runs, "start_index", 0, "runs")
    )
    if run_count <= 0:
        raise ValueError("runs.count must be positive")
    if start_index < 0:
        raise ValueError("runs.start_index must be nonnegative")

    randomization = require_object(root.get("randomization"), "randomization")
    master_seed = require_int(randomization, "master_seed", "randomization")
    seed_policy = optional_string(
        randomization, "seed_policy", SUPPORTED_SEED_POLICY, "randomization"
    )
    if seed_policy != SUPPORTED_SEED_POLICY:
        raise ValueError(
            f"unsupported randomization.seed_policy '{seed_policy}'; "
            f"only '{SUPPORTED_SEED_POLICY}' is currently supported"
        )

    execution = require_object(root.get("execution", {}), "execution")
    build_type = build_type_override or optional_string(
        execution, "build_type", "Release", "execution"
    )
    if build_type not in {"Debug", "Release"}:
        raise ValueError("execution.build_type must be 'Debug' or 'Release'")
    parallel_jobs = (
        parallel_jobs_override
        if parallel_jobs_override is not None
        else optional_int(execution, "parallel_jobs", 1, "execution")
    )
    if parallel_jobs <= 0:
        raise ValueError("execution.parallel_jobs must be positive")

    analysis = require_object(root.get("analysis", {}), "analysis")
    max_plot_points = (
        max_plot_points_override
        if max_plot_points_override is not None
        else analysis.get("max_plot_points")
    )
    if max_plot_points is not None and not isinstance(max_plot_points, int):
        raise ValueError("analysis.max_plot_points must be an integer when provided")
    if max_plot_points is not None and max_plot_points <= 1:
        raise ValueError("analysis.max_plot_points must be greater than one")

    output = require_object(root.get("output", {}), "output")
    output_root = output_root_override or Path(
        optional_string(output, "root", "output/monte_carlo", "output")
    )
    run_analysis = optional_bool(output, "run_analysis", False, "output")

    return CampaignConfig(
        campaign_path=campaign_path,
        campaign_name=campaign_name,
        nominal_config=nominal_config,
        run_count=run_count,
        start_index=start_index,
        master_seed=master_seed,
        seed_policy=seed_policy,
        build_type=build_type,
        parallel_jobs=parallel_jobs,
        max_plot_points=max_plot_points,
        output_root=output_root,
        run_analysis=run_analysis,
        navkit_config=navkit_config,
        generator=generator,
        build_dir=build_dir,
    )


def escape_json_pointer_token(token: str) -> str:
    return token.replace("~", "~0").replace("/", "~1")


def unescape_json_pointer_token(token: str) -> str:
    return token.replace("~1", "/").replace("~0", "~")


def discover_seed_paths(node: Any, path: str = "") -> list[str]:
    seed_paths: list[str] = []
    if isinstance(node, dict):
        if "seed" in node:
            seed_paths.append(f"{path}/seed")
        for key, value in node.items():
            child_path = f"{path}/{escape_json_pointer_token(str(key))}"
            seed_paths.extend(discover_seed_paths(value, child_path))
    elif isinstance(node, list):
        for index, value in enumerate(node):
            seed_paths.extend(discover_seed_paths(value, f"{path}/{index}"))
    return seed_paths


def set_json_pointer(root: JsonObject, pointer: str, value: int) -> None:
    if not pointer.startswith("/"):
        raise ValueError(f"expected absolute JSON pointer, got '{pointer}'")
    tokens = [unescape_json_pointer_token(token) for token in pointer.split("/")[1:]]
    current: Any = root
    for token in tokens[:-1]:
        if isinstance(current, list):
            current = current[int(token)]
        else:
            current = current[token]
    final_token = tokens[-1]
    if isinstance(current, list):
        current[int(final_token)] = value
    else:
        current[final_token] = value


def derive_seed(master_seed: int, run_index: int, json_pointer: str) -> int:
    message = f"{master_seed}:{run_index}:{json_pointer}".encode("utf-8")
    digest = hashlib.blake2b(message, digest_size=8).digest()
    seed = int.from_bytes(digest, byteorder="little", signed=False) & 0xFFFFFFFF
    return 1 if seed == 0 else seed


def campaign_dir(config: CampaignConfig) -> Path:
    return config.output_root / config.campaign_name


def build_run_plans(config: CampaignConfig, nominal_config: JsonObject) -> list[RunPlan]:
    root_dir = campaign_dir(config)
    tools_dir = Path(__file__).resolve().parent
    seed_paths = discover_seed_paths(nominal_config)
    plans: list[RunPlan] = []

    for offset in range(config.run_count):
        run_index = config.start_index + offset
        run_id = f"run_{run_index:06d}"
        run_name = f"{config.campaign_name}_{run_id}"
        run_dir = root_dir / "runs" / run_id
        run_config = copy.deepcopy(nominal_config)
        run_config["run_name"] = run_name
        run_config["output_dir"] = str(run_dir)

        derived_seeds: dict[str, int] = {}
        for seed_path in seed_paths:
            seed = derive_seed(config.master_seed, run_index, seed_path)
            set_json_pointer(run_config, seed_path, seed)
            derived_seeds[seed_path] = seed

        input_path = run_dir / "input.effective.json"
        command = [
            sys.executable,
            str(tools_dir / "run_sim.py"),
            "--build-type",
            config.build_type,
            "--generator",
            config.generator,
            "--navkit-config",
            config.navkit_config,
            "--config",
            str(input_path),
            "--no-profile-report",
            "--no-profile-trace",
        ]
        if config.build_dir is not None:
            command.extend(["--build-dir", str(config.build_dir)])

        plans.append(
            RunPlan(
                run_index=run_index,
                run_name=run_name,
                run_dir=run_dir,
                input_path=input_path,
                effective_config=run_config,
                derived_seeds=derived_seeds,
                command=command,
            )
        )

    return plans


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def run_one(plan: RunPlan) -> RunResult:
    plan.run_dir.mkdir(parents=True, exist_ok=True)
    write_json(plan.input_path, plan.effective_config)

    started_s = time.perf_counter()
    completed = subprocess.run(plan.command, check=False, capture_output=True, text=True)
    elapsed_s = time.perf_counter() - started_s
    (plan.run_dir / "simulation_stdout.txt").write_text(completed.stdout, encoding="utf-8")
    (plan.run_dir / "simulation_stderr.txt").write_text(completed.stderr, encoding="utf-8")
    status = "passed" if completed.returncode == 0 else "failed"

    result = RunResult(
        run_index=plan.run_index,
        run_name=plan.run_name,
        run_dir=plan.run_dir,
        input_path=plan.input_path,
        derived_seeds=plan.derived_seeds,
        command=plan.command,
        return_code=completed.returncode,
        elapsed_s=elapsed_s,
        status=status,
    )
    write_run_manifest(result)
    return result


def write_run_manifest(result: RunResult) -> None:
    write_json(
        result.run_dir / "run_manifest.json",
        {
            "schema": RUN_SCHEMA,
            "run_index": result.run_index,
            "run_name": result.run_name,
            "run_dir": str(result.run_dir),
            "input_config": str(result.input_path),
            "derived_seeds": result.derived_seeds,
            "command": result.command,
            "return_code": result.return_code,
            "status": result.status,
            "elapsed_s": result.elapsed_s,
        },
    )


def write_campaign_config(config: CampaignConfig, seed_paths: list[str]) -> None:
    root_dir = campaign_dir(config)
    write_json(
        root_dir / "campaign_config.effective.json",
        {
            "schema": CAMPAIGN_SCHEMA,
            "campaign_name": config.campaign_name,
            "nominal_config": str(config.nominal_config),
            "runs": {
                "count": config.run_count,
                "start_index": config.start_index,
            },
            "randomization": {
                "master_seed": config.master_seed,
                "seed_policy": config.seed_policy,
                "seed_paths": seed_paths,
            },
            "execution": {
                "build_type": config.build_type,
                "parallel_jobs": config.parallel_jobs,
                "navkit_config": config.navkit_config,
                "generator": config.generator,
                "build_dir": str(config.build_dir) if config.build_dir is not None else None,
            },
            "analysis": {
                "max_plot_points": config.max_plot_points,
            },
            "output": {
                "root": str(config.output_root),
                "campaign_dir": str(root_dir),
                "run_analysis": config.run_analysis,
            },
        },
    )


def write_campaign_manifest(
    config: CampaignConfig, results: list[RunResult], plot_elapsed_s: float | None
) -> None:
    root_dir = campaign_dir(config)
    write_json(
        root_dir / "campaign_manifest.json",
        {
            "schema": CAMPAIGN_SCHEMA,
            "campaign_name": config.campaign_name,
            "nominal_config": str(config.nominal_config),
            "master_seed": config.master_seed,
            "seed_policy": config.seed_policy,
            "run_count": len(results),
            "passed_count": sum(1 for result in results if result.status == "passed"),
            "failed_count": sum(1 for result in results if result.status != "passed"),
            "plot_elapsed_s": plot_elapsed_s,
            "runs": [
                {
                    "run_index": result.run_index,
                    "run_name": result.run_name,
                    "run_dir": str(result.run_dir),
                    "input_config": str(result.input_path),
                    "derived_seeds": result.derived_seeds,
                    "return_code": result.return_code,
                    "status": result.status,
                    "elapsed_s": result.elapsed_s,
                }
                for result in sorted(results, key=lambda item: item.run_index)
            ],
        },
    )


def run_analysis_for_run(run_dir: Path) -> int:
    tools_dir = Path(__file__).resolve().parent
    command = [sys.executable, str(tools_dir / "run_analysis.py"), str(run_dir)]
    completed = subprocess.run(command, check=False, capture_output=True, text=True)
    (run_dir / "analysis_stdout.txt").write_text(completed.stdout, encoding="utf-8")
    (run_dir / "analysis_stderr.txt").write_text(completed.stderr, encoding="utf-8")
    return completed.returncode


def generate_campaign_plots(
    successful_run_dirs: list[Path], figures_dir: Path, max_plot_points: int | None
) -> tuple[int, float]:
    root = Path(__file__).resolve().parents[1]
    python_root = root / "python"
    if str(python_root) not in sys.path:
        sys.path.insert(0, str(python_root))
    os.environ.setdefault("MPLBACKEND", "Agg")

    from navkit_analysis.monte_carlo import plot_monte_carlo_aggregates
    import matplotlib.pyplot as plt

    started_s = time.perf_counter()
    figures = plot_monte_carlo_aggregates(successful_run_dirs, figures_dir, max_plot_points)
    for figure in figures:
        plt.close(figure)
    elapsed_s = time.perf_counter() - started_s
    return (0 if figures else 1), elapsed_s


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a NavKit Monte Carlo campaign.")
    parser.add_argument("config", type=Path, help="Monte Carlo campaign JSON config.")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=None,
        help="Override the campaign output root from the Monte Carlo JSON.",
    )
    parser.add_argument(
        "--build-type",
        choices=["Release", "Debug"],
        default=None,
        help="Override execution.build_type from the Monte Carlo JSON.",
    )
    parser.add_argument(
        "--run-count",
        type=int,
        default=None,
        help="Override runs.count from the Monte Carlo JSON.",
    )
    parser.add_argument(
        "--start-index",
        type=int,
        default=None,
        help="Override runs.start_index from the Monte Carlo JSON.",
    )
    parser.add_argument(
        "--parallel-jobs",
        type=int,
        default=None,
        help="Override execution.parallel_jobs from the Monte Carlo JSON.",
    )
    parser.add_argument(
        "--max-plot-points",
        type=int,
        default=None,
        help="Override analysis.max_plot_points from the Monte Carlo JSON.",
    )
    parser.add_argument("--generator", default=DEFAULT_GENERATOR, help="CMake generator used.")
    parser.add_argument(
        "--navkit-config",
        default=DEFAULT_NAVKIT_CONFIG,
        help="Compile-time config header used to locate the matching build tree/executable.",
    )
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=None,
        help="Build directory override forwarded to run_sim.py.",
    )
    args = parser.parse_args()

    campaign_path = args.config.resolve()
    config = load_campaign_config(
        campaign_path,
        output_root_override=args.output_root,
        build_type_override=args.build_type,
        run_count_override=args.run_count,
        start_index_override=args.start_index,
        parallel_jobs_override=args.parallel_jobs,
        max_plot_points_override=args.max_plot_points,
        navkit_config=args.navkit_config,
        generator=args.generator,
        build_dir=args.build_dir,
    )
    nominal_config = load_runtime_config(config.nominal_config)
    seed_paths = discover_seed_paths(nominal_config)
    root_dir = campaign_dir(config)
    write_campaign_config(config, seed_paths)

    print(f"Monte Carlo campaign: {config.campaign_name}")
    print(f"Nominal config: {config.nominal_config}")
    print(f"Output: {root_dir}")
    print(f"Runs: {config.run_count} starting at {config.start_index}")
    print(f"Parallel jobs: {config.parallel_jobs}")
    print(f"Max plot points: {config.max_plot_points}")
    print(f"Seed paths: {seed_paths}")

    plans = build_run_plans(config, nominal_config)
    results: list[RunResult] = []
    simulation_started_s = time.perf_counter()
    with concurrent.futures.ThreadPoolExecutor(max_workers=config.parallel_jobs) as executor:
        future_to_plan = {executor.submit(run_one, plan): plan for plan in plans}
        for future in concurrent.futures.as_completed(future_to_plan):
            result = future.result()
            results.append(result)
            print(
                f"{result.run_name}: {result.status} "
                f"({result.elapsed_s:.3f} s, return {result.return_code})"
            )
    simulation_elapsed_s = time.perf_counter() - simulation_started_s

    if config.run_analysis:
        for result in results:
            if result.status == "passed":
                analysis_return_code = run_analysis_for_run(result.run_dir)
                if analysis_return_code != 0:
                    print(f"{result.run_name}: analysis failed with return {analysis_return_code}")

    successful_run_dirs = [
        result.run_dir
        for result in sorted(results, key=lambda item: item.run_index)
        if result.status == "passed"
    ]
    plot_return_code = 1
    plot_elapsed_s: float | None = None
    if successful_run_dirs:
        plot_return_code, plot_elapsed_s = generate_campaign_plots(
            successful_run_dirs,
            root_dir / "summary" / "figures",
            config.max_plot_points,
        )
    combined_elapsed_s = simulation_elapsed_s + (plot_elapsed_s if plot_elapsed_s is not None else 0.0)
    print("Monte Carlo run timing:")
    print(f"  simulation: {simulation_elapsed_s:.3f} s")
    if plot_elapsed_s is not None:
        print(f"  plotting:   {plot_elapsed_s:.3f} s")
    else:
        print("  plotting:   not run")
    print(f"  total:      {combined_elapsed_s:.3f} s")
    write_campaign_manifest(config, results, plot_elapsed_s)

    failed_count = sum(1 for result in results if result.status != "passed")
    if failed_count > 0:
        return 1
    return plot_return_code


if __name__ == "__main__":
    raise SystemExit(main())
