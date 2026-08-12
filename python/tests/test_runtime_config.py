# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Focused tests for the explicit runtime-config reference graph."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "tools"))

from internal.runtime_config import (  # noqa: E402
    load_runtime_config,
    resolve_runtime_asset_paths,
    runtime_output_dir,
    runtime_run_name,
)


class RuntimeConfigReferenceTests(unittest.TestCase):
    @staticmethod
    def _write(path: Path, value: object) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value), encoding="utf-8")

    def test_resolves_nested_references_relative_to_the_containing_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_runtime_config_") as temp_dir:
            root = Path(temp_dir)
            self._write(
                root / "shared" / "navigation.json",
                {"sensors": {"gnss": {"enabled": True}}},
            )
            self._write(
                root / "mission" / "phase.json",
                {
                    "id": "cruise",
                    "navigation": {"config": "../shared/navigation.json"},
                },
            )
            self._write(
                root / "scenario.json",
                {
                    "mission": {
                        "phases": [{"config": "mission/phase.json"}],
                    }
                },
            )

            resolved = load_runtime_config(root / "scenario.json")

            self.assertEqual(resolved["mission"]["phases"][0]["id"], "cruise")
            self.assertTrue(
                resolved["mission"]["phases"][0]["navigation"]["sensors"][
                    "gnss"
                ]["enabled"]
            )

    def test_applies_explicit_deep_overrides_after_loading_reference(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_runtime_config_") as temp_dir:
            root = Path(temp_dir)
            self._write(
                root / "components" / "autopilot.json",
                {
                    "type": "first_order",
                    "rate_hz": 200.0,
                    "limits": {"roll_deg": 60.0, "pitch_deg": 30.0},
                },
            )
            self._write(
                root / "scenario.json",
                {
                    "autopilot": {
                        "config": "components/autopilot.json",
                        "overrides": {
                            "rate_hz": 400.0,
                            "limits": {"pitch_deg": 20.0},
                        },
                    }
                },
            )

            resolved = load_runtime_config(root / "scenario.json")

            self.assertEqual(resolved["autopilot"]["type"], "first_order")
            self.assertEqual(resolved["autopilot"]["rate_hz"], 400.0)
            self.assertEqual(resolved["autopilot"]["limits"]["roll_deg"], 60.0)
            self.assertEqual(resolved["autopilot"]["limits"]["pitch_deg"], 20.0)

    def test_rejects_reference_cycles_with_the_reference_chain(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_runtime_config_") as temp_dir:
            root = Path(temp_dir)
            self._write(root / "a.json", {"next": {"config": "b.json"}})
            self._write(root / "b.json", {"next": {"config": "a.json"}})

            with self.assertRaisesRegex(ValueError, "reference cycle detected") as raised:
                load_runtime_config(root / "a.json")

            message = str(raised.exception)
            self.assertIn("a.json", message)
            self.assertIn("b.json", message)

    def test_rejects_malformed_or_ambiguous_reference_objects(self) -> None:
        malformed_cases = (
            (
                {"value": {"config": "component.json", "enabled": True}},
                "may contain only 'config'",
            ),
            (
                {"value": {"overrides": {"enabled": True}}},
                "valid only beside an explicit 'config'",
            ),
            ({"value": {"config": 7}}, "must be a path string"),
            (
                {"components": {"mission": "mission.json"}},
                "legacy runtime config 'components' assembly is unsupported",
            ),
        )

        for document, expected_message in malformed_cases:
            with self.subTest(expected_message=expected_message):
                with tempfile.TemporaryDirectory(
                    prefix="navkit_runtime_config_"
                ) as temp_dir:
                    root = Path(temp_dir)
                    self._write(root / "component.json", {"enabled": False})
                    self._write(root / "scenario.json", document)
                    with self.assertRaisesRegex(ValueError, expected_message):
                        load_runtime_config(root / "scenario.json")

    def test_rejects_missing_references_with_the_reference_chain(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_runtime_config_") as temp_dir:
            root = Path(temp_dir)
            self._write(root / "scenario.json", {"mission": {"config": "missing.json"}})

            with self.assertRaisesRegex(ValueError, "reference does not exist") as raised:
                load_runtime_config(root / "scenario.json")

            message = str(raised.exception)
            self.assertIn("scenario.json", message)
            self.assertIn("missing.json", message)

    def test_canonicalizes_csv_assets_before_relocating_effective_config(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_runtime_config_") as temp_dir:
            root = Path(temp_dir)
            source_config = root / "scenario" / "csv_playback.json"
            expected_csv = root / "scenario" / "data" / "truth.csv"
            self._write(source_config, {"simulation": {"source": {"type": "csv"}}})
            expected_csv.parent.mkdir(parents=True, exist_ok=True)
            expected_csv.write_text("time_s\n", encoding="utf-8")

            resolved = resolve_runtime_asset_paths(
                {
                    "simulation": {
                        "source": {"type": "csv", "csv_path": "data/truth.csv"}
                    }
                },
                source_config,
            )

            self.assertEqual(
                Path(resolved["simulation"]["source"]["csv_path"]),
                expected_csv.resolve(),
            )

    def test_requires_explicit_run_identity_and_output_directory(self) -> None:
        with self.assertRaisesRegex(ValueError, "missing required 'run_name'"):
            runtime_run_name({})
        with self.assertRaisesRegex(ValueError, "missing required 'output_dir'"):
            runtime_output_dir({"run_name": "test"})

    def test_all_supported_scenarios_resolve_with_exact_phase_mappings(self) -> None:
        scenario_dir = REPOSITORY_ROOT / "config" / "runtime" / "navkit" / "scenario"
        scenario_paths = sorted(scenario_dir.glob("*.json"))

        self.assertEqual(len(scenario_paths), 29)
        for scenario_path in scenario_paths:
            with self.subTest(scenario=scenario_path.name):
                resolved = load_runtime_config(scenario_path)
                phases = resolved["mission"]["phases"]
                phase_ids = {phase["id"] for phase in phases}
                phase_behavior = resolved["simulation"]["phase_behavior"]
                self.assertEqual(set(phase_behavior), phase_ids)
                self.assertIsInstance(runtime_run_name(resolved), str)
                self.assertIsInstance(runtime_output_dir(resolved), Path)


if __name__ == "__main__":
    unittest.main()
