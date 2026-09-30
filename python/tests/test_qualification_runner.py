# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Focused qualification-runner provenance and baseline-policy tests."""

from __future__ import annotations

import copy
import contextlib
import io
import json
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import h5py
import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "python"))
sys.path.insert(0, str(REPOSITORY_ROOT / "tools"))

import run_qualification as qualification_runner  # noqa: E402
from navkit_analysis.analysis_performance import canonical_json_digest  # noqa: E402
from navkit_analysis.qualification import (  # noqa: E402
    QualificationCampaignSpec,
    QualificationCriterionSpec,
    QualificationDisposition,
    QualificationSuite,
    QualificationWindow,
)
from navkit_analysis.regression import (  # noqa: E402
    TRUTH_RECONSTRUCTION_METRIC_UNITS,
)
from navkit_analysis.schema import (  # noqa: E402
    DETERMINISTIC_REGRESSION_SUITE_SCHEMA,
)


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def _synthetic_provenance() -> dict[str, object]:
    return {
        "git": {
            "revision": "a" * 40,
            "dirty": False,
            "dirty_tree_sha256": None,
        },
        "build": {
            "directory": "synthetic/build",
            "manifest": {
                "path": "synthetic/build/navkit_build_manifest.json",
                "sha256": "b" * 64,
                "canonical_sha256": "c" * 64,
            },
            "application_executable": {
                "path": "synthetic/build/navkit_swil.exe",
                "sha256": "d" * 64,
                "size_bytes": 1024,
            },
        },
    }


class QualificationRunnerProvenanceTests(unittest.TestCase):
    """Exercise runner-only compatibility checks without executing campaigns."""

    def test_reused_evidence_requires_current_compiled_artifact(self) -> None:
        observed = _synthetic_provenance()
        current = copy.deepcopy(observed)
        validation = qualification_runner._validate_current_build_provenance(
            observed,
            current,
            "synthetic evidence",
        )
        self.assertTrue(validation["build_matches_current"])
        self.assertTrue(validation["source_matches_current"])

        source_changed = copy.deepcopy(current)
        source_changed["git"]["revision"] = "e" * 40
        validation = qualification_runner._validate_current_build_provenance(
            observed,
            source_changed,
            "synthetic evidence",
        )
        self.assertFalse(validation["source_matches_current"])

        binary_changed = copy.deepcopy(current)
        binary_changed["build"]["application_executable"]["sha256"] = "f" * 64
        with self.assertRaisesRegex(ValueError, "current selected build"):
            qualification_runner._validate_current_build_provenance(
                observed,
                binary_changed,
                "synthetic evidence",
            )

    def test_canonical_json_file_digest_ignores_formatting_and_key_order(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_qualification_runner_") as temp:
            root = Path(temp)
            compact = root / "compact.json"
            formatted = root / "formatted.json"
            compact.write_text(
                '{"outer":{"beta":2,"alpha":1},"values":[3,2,1]}',
                encoding="utf-8",
            )
            formatted.write_text(
                """{
  "values": [3, 2, 1],
  "outer": {
    "alpha": 1,
    "beta": 2
  }
}
""",
                encoding="utf-8",
            )

            self.assertEqual(
                qualification_runner._canonical_json_file_digest(compact),
                qualification_runner._canonical_json_file_digest(formatted),
            )

    def test_bundle_runtime_config_is_portable_and_normalizes_run_identity_and_seeds(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_qualification_runner_") as temp:
            root = Path(temp)
            nominal_path = root / "nominal.json"
            campaign_path = root / "campaign.json"
            nominal = {
                "run_name": "selected_nominal",
                "output_dir": "output/logs/selected_nominal",
                "imu": {"seed": 101, "white_noise_variance": 0.25},
                "gnss": {"seed": 202, "position_variance": [1.0, 1.0, 4.0]},
                "mission": {"duration_s": 60.0},
            }
            _write_json(nominal_path, nominal)
            _write_json(campaign_path, {"nominal_config": "nominal.json"})
            campaign = QualificationCampaignSpec(
                name="synthetic",
                config=campaign_path,
                campaign_name="synthetic_campaign",
                window_name="steady_state",
            )
            seed_paths = ["/gnss/seed", "/imu/seed"]
            master_seed = 1234
            embedded_campaign = {
                "randomization": {
                    "master_seed": master_seed,
                    "seed_policy": "derive_all",
                    "seed_paths": seed_paths,
                }
            }
            embedded_campaign["runs"] = {"count": 1}

            packaged = copy.deepcopy(nominal)
            packaged["run_name"] = "synthetic_campaign_run_0042"
            packaged["output_dir"] = "D:/moved/evidence/run_0042"
            packaged_seeds = {
                seed_path: qualification_runner.derive_seed(master_seed, 0, seed_path)
                for seed_path in seed_paths
            }
            packaged["imu"]["seed"] = packaged_seeds["/imu/seed"]
            packaged["gnss"]["seed"] = packaged_seeds["/gnss/seed"]
            original_bundle = root / "original" / "analysis_bundle.h5"
            original_bundle.parent.mkdir(parents=True)
            self._write_bundle_runtime_config(
                original_bundle, packaged, derived_seeds=packaged_seeds
            )
            moved_bundle = root / "archive" / "nested" / "analysis_bundle.h5"
            moved_bundle.parent.mkdir(parents=True)
            original_bundle.replace(moved_bundle)

            validation = qualification_runner._validate_bundle_runtime_config(
                campaign,
                moved_bundle,
                embedded_campaign,
            )

            self.assertEqual(validation["seed_paths"], seed_paths)
            self.assertEqual(
                validation["normalized_nominal_sha256"],
                validation["normalized_packaged_run_sha256"],
            )
            self.assertEqual(validation["validated_run_count"], 1)

            changed = copy.deepcopy(packaged)
            changed["mission"]["duration_s"] = 61.0
            self._write_bundle_runtime_config(
                moved_bundle, changed, derived_seeds=packaged_seeds
            )
            with self.assertRaisesRegex(ValueError, "runtime config does not match"):
                qualification_runner._validate_bundle_runtime_config(
                    campaign,
                    moved_bundle,
                    embedded_campaign,
                )

    def test_bundle_runtime_config_rejects_seed_path_contract_mismatch(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_qualification_runner_") as temp:
            root = Path(temp)
            nominal = {
                "imu": {"seed": 11},
                "gnss": {"seed": 22},
            }
            nominal_path = root / "nominal.json"
            campaign_path = root / "campaign.json"
            bundle_path = root / "analysis_bundle.h5"
            _write_json(nominal_path, nominal)
            _write_json(campaign_path, {"nominal_config": "nominal.json"})
            self._write_bundle_runtime_config(
                bundle_path, nominal, derived_seeds={"/imu/seed": 11, "/gnss/seed": 22}
            )
            campaign = QualificationCampaignSpec(
                name="synthetic",
                config=campaign_path,
                campaign_name="synthetic_campaign",
                window_name="steady_state",
            )

            with self.assertRaisesRegex(ValueError, "seed paths do not match"):
                qualification_runner._validate_bundle_runtime_config(
                    campaign,
                    bundle_path,
                    {
                        "randomization": {
                            "master_seed": 5,
                            "seed_policy": "derive_all",
                            "seed_paths": ["/imu/seed"],
                        },
                        "runs": {"count": 1},
                    },
                )

    def test_bundle_runtime_config_validates_every_packaged_run_and_count(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_qualification_runner_") as temp:
            root = Path(temp)
            nominal_path = root / "nominal.json"
            campaign_path = root / "campaign.json"
            bundle_path = root / "analysis_bundle.h5"
            nominal = {
                "run_name": "nominal",
                "output_dir": "output/logs/nominal",
                "imu": {"seed": 11},
                "mission": {"duration_s": 60.0},
            }
            _write_json(nominal_path, nominal)
            _write_json(campaign_path, {"nominal_config": "nominal.json"})
            campaign = QualificationCampaignSpec(
                name="synthetic",
                config=campaign_path,
                campaign_name="synthetic_campaign",
                window_name="steady_state",
            )
            first = copy.deepcopy(nominal)
            master_seed = 4321
            first_seed = qualification_runner.derive_seed(master_seed, 0, "/imu/seed")
            first["imu"]["seed"] = first_seed
            second = copy.deepcopy(nominal)
            second_seed = qualification_runner.derive_seed(master_seed, 1, "/imu/seed")
            second["imu"]["seed"] = second_seed
            with h5py.File(bundle_path, "w") as bundle:
                for run_index, runtime_config, seed in (
                    (0, first, first_seed),
                    (1, second, second_seed),
                ):
                    run_name = f"run_{run_index:06d}"
                    run = bundle.create_group(f"runs/{run_name}")
                    run.attrs["metadata"] = json.dumps(
                        {
                            "runtime_config": runtime_config,
                            "campaign_run_manifest": {
                                "run_index": run_index,
                                "derived_seeds": {"/imu/seed": seed},
                            },
                        },
                        separators=(",", ":"),
                    )
            embedded = {
                "randomization": {
                    "master_seed": master_seed,
                    "seed_policy": "derive_all",
                    "seed_paths": ["/imu/seed"],
                },
                "runs": {"count": 2},
            }

            validation = qualification_runner._validate_bundle_runtime_config(
                campaign, bundle_path, embedded
            )
            self.assertEqual(validation["validated_run_count"], 2)

            with h5py.File(bundle_path, "r+") as bundle:
                metadata = json.loads(bundle["runs/run_000001"].attrs["metadata"])
                metadata["runtime_config"]["mission"]["duration_s"] = 61.0
                bundle["runs/run_000001"].attrs["metadata"] = json.dumps(metadata)
            with self.assertRaisesRegex(ValueError, "run 'run_000001'.*does not match"):
                qualification_runner._validate_bundle_runtime_config(
                    campaign, bundle_path, embedded
                )

            embedded["runs"]["count"] = 3
            with self.assertRaisesRegex(ValueError, "packaged run count"):
                qualification_runner._validate_bundle_runtime_config(
                    campaign, bundle_path, embedded
                )

    def test_embedded_campaign_requires_selected_product_and_generator(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_qualification_runner_") as temp:
            root = Path(temp)
            campaign_path = root / "campaign.json"
            _write_json(root / "nominal.json", {})
            selected = {
                "campaign_name": "synthetic_campaign",
                "nominal_config": "nominal.json",
                "runs": {"count": 10, "start_index": 4},
                "randomization": {
                    "master_seed": 1234,
                    "seed_policy": "derive_all",
                },
                "execution": {"build_type": "Release"},
            }
            _write_json(campaign_path, selected)
            campaign = QualificationCampaignSpec(
                name="synthetic",
                config=campaign_path,
                campaign_name="synthetic_campaign",
                window_name="steady_state",
            )
            embedded = copy.deepcopy(selected)
            embedded["provenance"] = {
                "nominal_config_sha256": canonical_json_digest({}),
                **_synthetic_provenance(),
            }
            embedded["execution"].update(
                {
                    "navkit_config": qualification_runner.DEFAULT_NAVKIT_CONFIG,
                    "generator": qualification_runner.DEFAULT_GENERATOR,
                }
            )
            metadata = {"campaign_config": embedded}

            qualification_runner._validate_embedded_campaign_config(
                campaign, metadata, 10, _synthetic_provenance()
            )

            for field, replacement in (
                ("navkit_config", "apps/navkit_swil/variants/Different.hpp"),
                ("generator", "Different Generator"),
            ):
                with self.subTest(field=field):
                    mismatched = copy.deepcopy(metadata)
                    mismatched["campaign_config"]["execution"][field] = replacement
                    with self.assertRaisesRegex(
                        ValueError,
                        "does not match selected qualification input",
                    ):
                        qualification_runner._validate_embedded_campaign_config(
                            campaign, mismatched, 10, _synthetic_provenance()
                        )

    def test_deterministic_report_must_match_selected_suite_contract(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_qualification_runner_") as temp:
            root = Path(temp)
            suite, report = self._deterministic_fixture(root)

            validation = qualification_runner._validate_deterministic_report(
                suite, report, _synthetic_provenance()
            )

            self.assertEqual(validation["suite_name"], "synthetic_regression")
            self.assertEqual(validation["build_type"], "Release")
            self.assertEqual(validation["case_count"], 1)

            internally_consistent_failure = copy.deepcopy(report)
            failed_case = internally_consistent_failure["cases"][0]
            failed_case["metrics"]["position_max_norm_m"] = 2.0
            selected_case = qualification_runner.load_deterministic_regression_suite(
                suite.deterministic_suite
            ).cases[0]
            failed_passed, failed_checks = (
                qualification_runner.evaluate_truth_reconstruction(
                    failed_case["metrics"], selected_case
                )
            )
            failed_case["passed"] = failed_passed
            failed_case["checks"] = failed_checks
            internally_consistent_failure["failed_count"] = 1
            internally_consistent_failure["passed"] = False
            failed_validation = qualification_runner._validate_deterministic_report(
                suite, internally_consistent_failure, _synthetic_provenance()
            )
            self.assertFalse(failed_validation["passed"])
            self.assertEqual(failed_validation["failed_count"], 1)

            mutations = {
                "suite identity": lambda value: value.update(
                    {"suite_name": "different_suite"}
                ),
                "suite digest": lambda value: value["suite"].update(
                    {"canonical_sha256": "f" * 64}
                ),
                "execution build": lambda value: value["execution"].update(
                    {"build_type": "Debug"}
                ),
                "product config": lambda value: value["execution"].update(
                    {"navkit_config": "apps/navkit_swil/Different.hpp"}
                ),
                "generator": lambda value: value["execution"].update(
                    {"generator": "Different Generator"}
                ),
                "case identity": lambda value: value["cases"][0].update(
                    {"name": "different_case"}
                ),
                "scenario digest": lambda value: value["cases"][0][
                    "scenario"
                ].update({"effective_sha256": "0" * 64}),
                "case passed": lambda value: value["cases"][0].update(
                    {"passed": False}
                ),
                "case metric": lambda value: value["cases"][0]["metrics"].update(
                    {"position_max_norm_m": 2.0}
                ),
                "case check": lambda value: value["cases"][0]["checks"][
                    "position_max_norm_m"
                ].update({"measured": 2.0}),
                "case return code": lambda value: value["cases"][0].update(
                    {"return_code": 1}
                ),
                "failed count": lambda value: value.update({"failed_count": 1}),
                "top-level passed": lambda value: value.update({"passed": False}),
            }
            for description, mutate in mutations.items():
                with self.subTest(description=description):
                    mismatched = copy.deepcopy(report)
                    mutate(mismatched)
                    with self.assertRaisesRegex(
                        ValueError, "does not match selected suite contract"
                    ):
                        qualification_runner._validate_deterministic_report(
                            suite, mismatched, _synthetic_provenance()
                        )

    def test_required_criterion_rejects_nonfinite_selected_evidence(self) -> None:
        suite, campaign = self._qualification_evidence_fixture()
        criterion = QualificationCriterionSpec(
            name="pva_consistency",
            campaigns=(campaign.name,),
            kind="nees",
            group="pva",
            metric="normalized_window_mean",
            test="equivalence",
            confidence=0.95,
            disposition=QualificationDisposition.REQUIRED,
            rationale="synthetic",
            minimum=0.5,
            maximum=1.5,
        )
        series = SimpleNamespace(
            time_s=np.array([0.0, 1.0]),
            values=np.array([[3.0, np.nan], [3.0, 3.0]]),
            dof=3,
            accepted=None,
            name="pva",
        )

        with self.assertRaisesRegex(ValueError, "nonfinite selected evidence"):
            qualification_runner._criterion_check(
                campaign, criterion, suite.windows[campaign.window_name], series
            )

        series.values = np.full((2, 2), 3.0)
        _, details = qualification_runner._criterion_check(
            campaign, criterion, suite.windows[campaign.window_name], series
        )
        self.assertEqual(details["selected_value_count"], 4)
        self.assertEqual(details["finite_value_count"], 4)
        self.assertEqual(details["finite_coverage"], 1.0)

    def test_acceptance_criterion_rejects_nonbinary_selected_evidence(self) -> None:
        suite, campaign = self._qualification_evidence_fixture()
        criterion = QualificationCriterionSpec(
            name="acceptance",
            campaigns=(campaign.name,),
            kind="nis",
            group="gnss_position",
            metric="acceptance_rate",
            test="minimum",
            confidence=0.95,
            disposition=QualificationDisposition.REQUIRED,
            rationale="synthetic",
            minimum=0.5,
        )
        series = SimpleNamespace(
            time_s=np.array([0.0, 1.0]),
            values=np.ones((2, 2)),
            dof=3,
            accepted=np.array([[1.0, 0.5], [1.0, 1.0]]),
            name="gnss_position",
        )

        with self.assertRaisesRegex(ValueError, "only zero or one"):
            qualification_runner._criterion_check(
                campaign, criterion, suite.windows[campaign.window_name], series
            )

        series.accepted = np.ones((2, 2))
        check, _ = qualification_runner._criterion_check(
            campaign, criterion, suite.windows[campaign.window_name], series
        )
        self.assertTrue(check.passed)

    def test_fractional_campaign_window_uses_full_ins_timebase(self) -> None:
        suite, campaign = self._qualification_evidence_fixture()
        suite = replace(
            suite,
            windows={
                campaign.window_name: QualificationWindow.duration_fraction(
                    0.5, 1.0
                )
            },
        )
        full_ins = SimpleNamespace(time_s=np.linspace(0.0, 10.0, 11))
        resolved = qualification_runner._resolved_campaign_window(
            suite, campaign, {("nees", "full_ins"): full_ins}
        )

        self.assertEqual(resolved, QualificationWindow.absolute_seconds(5.0, 10.0))

        criterion = QualificationCriterionSpec(
            name="position_nis",
            campaigns=(campaign.name,),
            kind="nis",
            group="gnss_position",
            metric="normalized_window_mean",
            test="equivalence",
            confidence=0.95,
            disposition=QualificationDisposition.REQUIRED,
            rationale="synthetic",
            minimum=0.5,
            maximum=1.5,
        )
        nis = SimpleNamespace(
            time_s=np.arange(2.0, 11.0),
            values=np.full((2, 9), 3.0),
            dof=3,
            accepted=None,
            name="gnss_position",
        )
        _, details = qualification_runner._criterion_check(
            campaign, criterion, resolved, nis
        )
        self.assertEqual(details["window_start_s"], 5.0)
        self.assertEqual(details["window_end_s"], 10.0)

        nis.time_s = np.arange(6.0, 11.0)
        nis.values = np.full((2, 5), 3.0)
        with self.assertRaisesRegex(ValueError, "must lie within"):
            qualification_runner._criterion_check(
                campaign, criterion, resolved, nis
            )

    def test_cross_covariance_ratio_handles_zero_denominator(self) -> None:
        suite, campaign = self._qualification_evidence_fixture()
        series = SimpleNamespace(
            time_s=np.array([0.0, 1.0]),
            values=np.zeros((2, 2)),
        )
        series_items = {
            ("nees", name): series
            for name in ("full_ins", "pva", "gyro_bias", "accel_bias")
        }

        diagnosis = qualification_runner._cross_covariance_diagnosis(
            campaign, suite.windows[campaign.window_name], series_items
        )

        assert diagnosis is not None
        self.assertIsNone(diagnosis["mean_joint_to_marginal_sum_ratio"])
        self.assertIsNone(diagnosis["median_joint_to_marginal_sum_ratio"])
        self.assertEqual(diagnosis["ratio_sample_count"], 0)
        self.assertEqual(diagnosis["zero_denominator_count"], 2)

    def test_schur_diagnosis_rejects_any_skipped_sampled_run(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_qualification_runner_") as temp:
            root = Path(temp)
            suite, campaign = self._qualification_evidence_fixture(root)
            bundle_path = root / "analysis_bundle.h5"
            with h5py.File(bundle_path, "w") as bundle:
                bundle.create_group("runs/run_000000")

            with self.assertRaisesRegex(
                ValueError, "1 of 1 sampled runs exceeded the allowed maximum of 0"
            ):
                qualification_runner._schur_covariance_diagnosis(
                    bundle_path, campaign, suite.windows[campaign.window_name]
                )

    def test_report_writer_rejects_nonfinite_json(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_qualification_runner_") as temp:
            output = Path(temp) / "report.json"
            with self.assertRaisesRegex(ValueError, "strict-JSON serializable"):
                qualification_runner._write_json(output, {"bad": float("nan")})
            self.assertFalse(output.exists())

    def test_bundle_without_embedded_runtime_config_is_not_reusable(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_qualification_runner_") as temp:
            root = Path(temp)
            nominal_path = root / "nominal.json"
            campaign_path = root / "campaign.json"
            bundle_path = root / "legacy_bundle.h5"
            nominal = {"imu": {"seed": 11}, "mission": {"duration_s": 60.0}}
            _write_json(nominal_path, nominal)
            _write_json(campaign_path, {"nominal_config": "nominal.json"})
            with h5py.File(bundle_path, "w") as bundle:
                run = bundle.create_group("runs/run_000000")
                run.attrs["metadata"] = json.dumps(
                    {"effective_config_path": "missing/effective_runtime_config.json"}
                )
            campaign = QualificationCampaignSpec(
                name="synthetic",
                config=campaign_path,
                campaign_name="synthetic_campaign",
                window_name="steady_state",
            )

            with self.assertRaisesRegex(
                ValueError, "lacks embedded effective runtime config"
            ):
                qualification_runner._validate_bundle_runtime_config(
                    campaign,
                    bundle_path,
                    {
                        "randomization": {
                            "master_seed": 5,
                            "seed_policy": "derive_all",
                            "seed_paths": ["/imu/seed"],
                        },
                        "runs": {"count": 1},
                    },
                )

    def test_baseline_updates_are_owned_by_qualification_tier(self) -> None:
        for tier in ("smoke", "diagnostic"):
            with self.subTest(tier=tier):
                with self.assertRaisesRegex(
                    ValueError, "require tier 'qualification'"
                ):
                    qualification_runner._validate_baseline_update_request(
                        tier, True
                    )

        qualification_runner._validate_baseline_update_request(
            "qualification", True
        )
        qualification_runner._validate_baseline_update_request(
            "qualification", True, "qualification"
        )
        qualification_runner._validate_baseline_update_request("smoke", False)
        with self.assertRaisesRegex(
            ValueError, "existing baseline must have tier 'qualification'"
        ):
            qualification_runner._validate_baseline_update_request(
                "qualification", True, "smoke"
            )

    def test_baseline_gate_only_blocks_qualification_tier(self) -> None:
        for tier in ("smoke", "diagnostic"):
            with self.subTest(tier=tier):
                report: dict[str, object] = {
                    "status": "pass",
                    "passed": True,
                    "baseline": {"status": "missing"},
                }
                qualification_runner._apply_baseline_gate(report, tier)
                self.assertTrue(report["passed"])
                self.assertEqual(report["status"], "pass")
                self.assertEqual(
                    report["baseline_gate"],
                    {
                        "required": False,
                        "passed": True,
                        "baseline_status": "missing",
                    },
                )

        for baseline_status, expected_passed in (
            ("compared", True),
            ("updated", True),
            ("missing", False),
            ("incompatible_suite", False),
        ):
            with self.subTest(baseline_status=baseline_status):
                report = {
                    "status": "pass",
                    "passed": True,
                    "baseline": {"status": baseline_status},
                }
                qualification_runner._apply_baseline_gate(report, "qualification")
                self.assertEqual(report["passed"], expected_passed)
                self.assertEqual(
                    report["baseline_gate"],
                    {
                        "required": True,
                        "passed": expected_passed,
                        "baseline_status": baseline_status,
                    },
                )
                self.assertEqual(
                    report["status"], "pass" if expected_passed else "fail"
                )

    def test_managed_baseline_matches_selected_suite_contract(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_qualification_runner_") as temp:
            suite, baseline = self._managed_baseline_fixture(Path(temp))

            validation = qualification_runner._validate_managed_baseline(suite)

            self.assertEqual(validation["suite_name"], suite.name)
            self.assertEqual(validation["tier"], "qualification")
            self.assertEqual(validation["expected_run_count"], 37)
            self.assertEqual(validation["campaign_count"], 2)
            self.assertEqual(validation["check_count"], 3)
            self.assertEqual(
                validation["qualification_status"], "pass_with_known_findings"
            )
            self.assertEqual(
                qualification_runner._canonical_json_file_digest(suite.source),
                baseline["metadata"]["suite_sha256"],
            )

    def test_managed_baseline_rejects_stale_or_invalid_contracts(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_qualification_runner_") as temp:
            suite, baseline = self._managed_baseline_fixture(Path(temp))

            def set_wrong_schema(value: dict[str, object]) -> None:
                value["schema"] = "navkit.qualification_baseline.invalid"

            def set_wrong_tier(value: dict[str, object]) -> None:
                value["metadata"]["tier"] = "diagnostic"

            def set_wrong_suite_name(value: dict[str, object]) -> None:
                value["metadata"]["suite_name"] = "different_suite"

            def set_wrong_suite_digest(value: dict[str, object]) -> None:
                value["metadata"]["suite_sha256"] = "0" * 64

            def set_wrong_deterministic_digest(value: dict[str, object]) -> None:
                value["metadata"]["deterministic_suite_sha256"] = "1" * 64

            def set_invalid_deterministic_report_digest(
                value: dict[str, object],
            ) -> None:
                value["metadata"]["deterministic_report_sha256"] = "invalid"

            def set_wrong_campaign_identity(value: dict[str, object]) -> None:
                value["metadata"]["campaigns"][0]["name"] = "unknown_campaign"

            def set_wrong_campaign_source_digest(value: dict[str, object]) -> None:
                value["metadata"]["campaigns"][0]["source_config_sha256"] = (
                    "2" * 64
                )

            def set_wrong_campaign_nominal_digest(value: dict[str, object]) -> None:
                value["metadata"]["campaigns"][0]["nominal_config_sha256"] = (
                    "3" * 64
                )

            def set_invalid_campaign_package_digest(value: dict[str, object]) -> None:
                value["metadata"]["campaigns"][0]["package_fingerprint"] = "invalid"

            def set_invalid_campaign_cache_digest(value: dict[str, object]) -> None:
                value["metadata"]["campaigns"][0][
                    "consistency_cache_fingerprint"
                ] = "invalid"

            def clear_campaign_generation_stability(value: dict[str, object]) -> None:
                value["metadata"]["campaigns"][0]["generation_stable"] = False

            def set_wrong_run_count(value: dict[str, object]) -> None:
                value["metadata"]["expected_run_count"] = 36

            def remove_check(value: dict[str, object]) -> None:
                del value["checks"]["campaign_a.required_consistency"]

            def add_check(value: dict[str, object]) -> None:
                value["checks"]["unexpected.check"] = copy.deepcopy(
                    value["checks"]["campaign_a.required_consistency"]
                )

            def change_check_contract(value: dict[str, object]) -> None:
                value["checks"]["campaign_a.required_consistency"]["minimum"] = 0.6

            def change_sample_count(value: dict[str, object]) -> None:
                value["checks"]["campaign_b.required_consistency"][
                    "sample_count"
                ] = 36

            def fail_required_check(value: dict[str, object]) -> None:
                value["checks"]["campaign_a.required_consistency"]["passed"] = False

            def falsify_required_check_claim(value: dict[str, object]) -> None:
                check = value["checks"]["campaign_a.required_consistency"]
                check["mean"] = 2.0
                check["ci_lower"] = 1.9
                check["ci_upper"] = 2.1

            def invert_confidence_interval(value: dict[str, object]) -> None:
                check = value["checks"]["campaign_a.required_consistency"]
                check["ci_lower"] = 1.1
                check["ci_upper"] = 0.9

            def move_mean_outside_interval(value: dict[str, object]) -> None:
                value["checks"]["campaign_a.required_consistency"]["mean"] = 1.2

            def set_inconsistent_aggregate_status(value: dict[str, object]) -> None:
                value["qualification_status"] = "pass"

            mutations = {
                "schema": set_wrong_schema,
                "tier": set_wrong_tier,
                "suite name": set_wrong_suite_name,
                "suite digest": set_wrong_suite_digest,
                "deterministic digest": set_wrong_deterministic_digest,
                "deterministic report digest": (
                    set_invalid_deterministic_report_digest
                ),
                "campaign identity": set_wrong_campaign_identity,
                "campaign source digest": set_wrong_campaign_source_digest,
                "campaign nominal digest": set_wrong_campaign_nominal_digest,
                "campaign package digest": set_invalid_campaign_package_digest,
                "campaign cache digest": set_invalid_campaign_cache_digest,
                "campaign generation stability": clear_campaign_generation_stability,
                "expected run count": set_wrong_run_count,
                "missing check": remove_check,
                "extra check": add_check,
                "changed check contract": change_check_contract,
                "sample count": change_sample_count,
                "failed required check": fail_required_check,
                "false required claim": falsify_required_check_claim,
                "inverted confidence interval": invert_confidence_interval,
                "mean outside confidence interval": move_mean_outside_interval,
                "aggregate status": set_inconsistent_aggregate_status,
            }
            for description, mutate in mutations.items():
                with self.subTest(description=description):
                    mismatched = copy.deepcopy(baseline)
                    mutate(mismatched)
                    _write_json(suite.baseline_path, mismatched)
                    with self.assertRaises(ValueError):
                        qualification_runner._validate_managed_baseline(suite)

    def test_validate_baseline_cli_reports_success_and_failure(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_qualification_runner_") as temp:
            suite, baseline = self._managed_baseline_fixture(Path(temp))
            standard_output = io.StringIO()
            standard_error = io.StringIO()
            arguments = [
                "run_qualification.py",
                str(suite.source),
                "--validate-baseline",
            ]

            with (
                patch.object(
                    qualification_runner,
                    "load_qualification_suite",
                    return_value=suite,
                ),
                patch.object(sys, "argv", arguments),
                contextlib.redirect_stdout(standard_output),
                contextlib.redirect_stderr(standard_error),
            ):
                self.assertEqual(qualification_runner.main(), 0)
            self.assertIn("Managed baseline valid", standard_output.getvalue())
            self.assertEqual(standard_error.getvalue(), "")

            baseline["metadata"]["tier"] = "smoke"
            _write_json(suite.baseline_path, baseline)
            standard_output = io.StringIO()
            standard_error = io.StringIO()
            with (
                patch.object(
                    qualification_runner,
                    "load_qualification_suite",
                    return_value=suite,
                ),
                patch.object(sys, "argv", arguments),
                contextlib.redirect_stdout(standard_output),
                contextlib.redirect_stderr(standard_error),
            ):
                self.assertEqual(qualification_runner.main(), 1)
            self.assertEqual(standard_output.getvalue(), "")
            self.assertIn(
                "Managed baseline validation failed", standard_error.getvalue()
            )

    @staticmethod
    def _write_bundle_runtime_config(
        path: Path,
        runtime_config: dict[str, object],
        *,
        derived_seeds: dict[str, int],
    ) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with h5py.File(path, "w") as bundle:
            run = bundle.create_group("runs/run_000000")
            run.attrs["metadata"] = json.dumps(
                {
                    "runtime_config": runtime_config,
                    "campaign_run_manifest": {
                        "run_index": 0,
                        "derived_seeds": derived_seeds,
                    },
                },
                separators=(",", ":"),
            )

    @staticmethod
    def _deterministic_fixture(
        root: Path,
    ) -> tuple[QualificationSuite, dict[str, object]]:
        scenario_path = root / "scenario.json"
        scenario = {
            "run_name": "synthetic",
            "output_dir": "output/logs/synthetic",
            "mission": {"duration_s": 2.0},
        }
        _write_json(scenario_path, scenario)
        thresholds = {
            metric_name: 1.0
            for metric_name in TRUTH_RECONSTRUCTION_METRIC_UNITS
        }
        navkit_config = "apps/navkit_swil/variants/Synthetic.hpp"
        suite_path = root / "deterministic_suite.json"
        suite_document = {
            "schema": DETERMINISTIC_REGRESSION_SUITE_SCHEMA,
            "suite_name": "synthetic_regression",
            "execution": {
                "build_type": "Release",
                "navkit_config": navkit_config,
            },
            "output": {"root": "output/regression/synthetic"},
            "cases": [
                {
                    "name": "synthetic_case",
                    "scenario": "scenario.json",
                    "minimum_duration_s": 1.0,
                    "minimum_sample_count": 2,
                    "thresholds": thresholds,
                    "sensor_update_counts": {
                        "gnss_position": {"minimum": 0, "maximum": 0},
                        "gnss_velocity": {"minimum": 1},
                    },
                }
            ],
        }
        _write_json(suite_path, suite_document)
        selected_case = qualification_runner.load_deterministic_regression_suite(
            suite_path
        ).cases[0]
        metrics: dict[str, float | int] = {
            metric_name: 0.5 for metric_name in TRUTH_RECONSTRUCTION_METRIC_UNITS
        }
        metrics.update(
            {
                "duration_s": 2.0,
                "sample_count": 2,
                "gnss_position_update_count": 0,
                "gnss_velocity_update_count": 1,
            }
        )
        case_passed, case_checks = qualification_runner.evaluate_truth_reconstruction(
            metrics, selected_case
        )
        qualification_source = root / "qualification_suite.json"
        _write_json(qualification_source, {"suite_name": "qualification"})
        suite = QualificationSuite(
            name="qualification",
            source=qualification_source,
            deterministic_suite=suite_path,
            campaign_sizes={"smoke": 2, "qualification": 10},
            output_root=root / "output",
            baseline_path=root / "baseline.json",
            windows={},
            campaigns=(),
            criteria=(),
        )
        report: dict[str, object] = {
            "suite_name": "synthetic_regression",
            "suite": {
                "canonical_sha256": qualification_runner._canonical_json_file_digest(
                    suite_path
                )
            },
            "execution": {
                "build_type": "Release",
                "navkit_config": navkit_config,
                "generator": qualification_runner.DEFAULT_GENERATOR,
                "build_manifest": {
                    "build_type": "Release",
                    "navkit_config": navkit_config,
                    "generator": qualification_runner.DEFAULT_GENERATOR,
                },
                "provenance": _synthetic_provenance(),
            },
            "case_count": 1,
            "failed_count": 0,
            "passed": True,
            "cases": [
                {
                    "name": "synthetic_case",
                    "passed": case_passed,
                    "return_code": 0,
                    "metrics": metrics,
                    "checks": case_checks,
                    "thresholds": thresholds,
                    "minimum_duration_s": 1.0,
                    "minimum_sample_count": 2,
                    "sensor_update_counts": {
                        "gnss_position": {"minimum": 0, "maximum": 0},
                        "gnss_velocity": {"minimum": 1, "maximum": None},
                    },
                    "scenario": {
                        "effective_sha256": canonical_json_digest(scenario)
                    },
                }
            ],
        }
        return suite, report

    @staticmethod
    def _qualification_evidence_fixture(
        root: Path | None = None,
    ) -> tuple[QualificationSuite, QualificationCampaignSpec]:
        fixture_root = root if root is not None else Path("synthetic")
        campaign = QualificationCampaignSpec(
            name="synthetic",
            config=fixture_root / "campaign.json",
            campaign_name="synthetic_campaign",
            window_name="steady_state",
        )
        suite = QualificationSuite(
            name="qualification",
            source=fixture_root / "suite.json",
            deterministic_suite=fixture_root / "deterministic.json",
            campaign_sizes={"qualification": 2},
            output_root=fixture_root / "output",
            baseline_path=fixture_root / "baseline.json",
            windows={
                "steady_state": QualificationWindow.absolute_seconds(0.0, 1.0)
            },
            campaigns=(campaign,),
            criteria=(),
        )
        return suite, campaign

    @staticmethod
    def _managed_baseline_fixture(
        root: Path,
    ) -> tuple[QualificationSuite, dict[str, object]]:
        suite_path = root / "qualification_suite.json"
        deterministic_path = root / "deterministic_suite.json"
        baseline_path = root / "baseline.json"
        _write_json(
            suite_path,
            {
                "schema": "navkit.qualification_suite.v1",
                "suite_name": "synthetic_managed_baseline",
            },
        )
        _write_json(
            deterministic_path,
            {
                "schema": DETERMINISTIC_REGRESSION_SUITE_SCHEMA,
                "suite_name": "synthetic_deterministic",
            },
        )

        campaigns: list[QualificationCampaignSpec] = []
        campaign_provenance: list[dict[str, object]] = []
        for campaign_name, nominal_value in (
            ("campaign_a", 1),
            ("campaign_b", 2),
        ):
            nominal_path = root / f"{campaign_name}_nominal.json"
            campaign_path = root / f"{campaign_name}.json"
            nominal = {
                "run_name": f"{campaign_name}_nominal",
                "mission": {"profile": campaign_name},
                "simulator": {"seed": nominal_value},
            }
            campaign_document = {
                "schema": "navkit.monte_carlo_campaign.v2",
                "type": "monte_carlo_campaign",
                "campaign_name": f"{campaign_name}_mc",
                "nominal_config": nominal_path.name,
                "runs": {"count": 100, "start_index": 0},
                "randomization": {
                    "master_seed": 1234,
                    "seed_policy": "derive_all",
                },
                "execution": {"build_type": "Release"},
            }
            _write_json(nominal_path, nominal)
            _write_json(campaign_path, campaign_document)
            campaign = QualificationCampaignSpec(
                name=campaign_name,
                config=campaign_path,
                campaign_name=f"{campaign_name}_mc",
                window_name="steady_state",
            )
            campaigns.append(campaign)
            provenance = qualification_runner._campaign_input_provenance(campaign)
            campaign_provenance.append(
                {
                    "name": campaign_name,
                    "source_config_sha256": provenance["source_config_sha256"],
                    "nominal_config_sha256": provenance["nominal_config_sha256"],
                    "package_fingerprint": "a" * 64,
                    "consistency_cache_fingerprint": "b" * 64,
                    "evidence_origin": "executed_current_run",
                    "generation_artifacts": {
                        "git": {
                            "revision": "synthetic",
                            "dirty": False,
                            "dirty_tree_sha256": None,
                        },
                        "build": {
                            "manifest_sha256": "c" * 64,
                            "manifest_canonical_sha256": "d" * 64,
                            "application_executable_sha256": "e" * 64,
                            "application_executable_size_bytes": 1,
                        },
                    },
                    "generation_stable": True,
                }
            )

        required = QualificationCriterionSpec(
            name="required_consistency",
            campaigns=("campaign_a", "campaign_b"),
            kind="nees",
            group="pva",
            metric="normalized_window_mean",
            test="equivalence",
            confidence=0.95,
            disposition=QualificationDisposition.REQUIRED,
            rationale="synthetic required criterion",
            minimum=0.7,
            maximum=1.3,
        )
        known_finding = QualificationCriterionSpec(
            name="known_finding",
            campaigns=("campaign_b",),
            kind="nees",
            group="full_ins",
            metric="normalized_window_mean",
            test="maximum",
            confidence=0.9,
            disposition=QualificationDisposition.KNOWN_FINDING,
            rationale="synthetic active known finding",
            maximum=1.5,
        )
        suite = QualificationSuite(
            name="synthetic_managed_baseline",
            source=suite_path,
            deterministic_suite=deterministic_path,
            campaign_sizes={"smoke": 2, "qualification": 37},
            output_root=root / "output",
            baseline_path=baseline_path,
            windows={
                "steady_state": QualificationWindow.absolute_seconds(0.0, 1.0)
            },
            campaigns=tuple(campaigns),
            criteria=(required, known_finding),
        )

        required_check = {
            "kind": "equivalence",
            "disposition": "required",
            "passed": True,
            "mean": 1.0,
            "ci_lower": 0.9,
            "ci_upper": 1.1,
            "confidence": 0.95,
            "sample_count": 37,
            "minimum": 0.7,
            "maximum": 1.3,
        }
        known_finding_check = {
            "kind": "upper_bound",
            "disposition": "known_finding",
            "passed": False,
            "mean": 1.8,
            "ci_lower": 1.7,
            "ci_upper": 1.9,
            "confidence": 0.9,
            "sample_count": 37,
            "minimum": None,
            "maximum": 1.5,
        }
        baseline: dict[str, object] = {
            "schema": "navkit.qualification_baseline.v1",
            "qualification_status": "pass_with_known_findings",
            "metadata": {
                "suite_name": suite.name,
                "suite_sha256": qualification_runner._canonical_json_file_digest(
                    suite.source
                ),
                "tier": "qualification",
                "expected_run_count": 37,
                "deterministic_suite_sha256": (
                    qualification_runner._canonical_json_file_digest(
                        suite.deterministic_suite
                    )
                ),
                "deterministic_report_sha256": "f" * 64,
                "campaigns": campaign_provenance,
            },
            "checks": {
                "campaign_a.required_consistency": copy.deepcopy(required_check),
                "campaign_b.required_consistency": copy.deepcopy(required_check),
                "campaign_b.known_finding": known_finding_check,
            },
        }
        _write_json(baseline_path, baseline)
        return suite, baseline


if __name__ == "__main__":
    unittest.main()
