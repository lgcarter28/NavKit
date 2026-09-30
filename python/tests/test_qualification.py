# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from scipy.stats import t as student_t

from navkit_analysis.qualification import (
    QUALIFICATION_BASELINE_SCHEMA,
    MeanConfidenceInterval,
    QualificationDisposition,
    QualificationStatus,
    QualificationWindow,
    aggregate_qualification,
    evaluate_equivalence,
    evaluate_lower_bound,
    evaluate_upper_bound,
    load_qualification_suite,
    qualification_baseline_deltas,
    qualification_baseline_snapshot,
    reduce_window_mean_per_run,
    select_qualification_window,
    student_t_mean_confidence_interval,
    write_qualification_baseline,
)


def _interval(mean: float, lower: float, upper: float) -> MeanConfidenceInterval:
    return MeanConfidenceInterval(
        mean=mean,
        lower=lower,
        upper=upper,
        confidence=0.95,
        sample_count=10,
        sample_standard_deviation=1.0,
        standard_error=0.1,
    )


class QualificationWindowTests(unittest.TestCase):
    def test_selects_closed_absolute_window(self) -> None:
        time_s = np.array([10.0, 11.0, 12.0, 13.0, 14.0])

        selection = select_qualification_window(
            time_s, QualificationWindow.absolute_seconds(11.0, 13.0)
        )

        np.testing.assert_array_equal(
            selection.mask, np.array([False, True, True, True, False])
        )
        self.assertEqual(selection.epoch_count, 3)
        self.assertEqual(selection.start_s, 11.0)
        self.assertEqual(selection.end_s, 13.0)

    def test_resolves_fraction_against_nonzero_time_origin(self) -> None:
        time_s = np.array([10.0, 12.0, 14.0, 16.0, 18.0])

        selection = select_qualification_window(
            time_s, QualificationWindow.duration_fraction(0.25, 0.75)
        )

        self.assertEqual(selection.start_s, 12.0)
        self.assertEqual(selection.end_s, 16.0)
        np.testing.assert_array_equal(
            selection.mask, np.array([False, True, True, True, False])
        )

    def test_rejects_invalid_time_and_window_contracts(self) -> None:
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            select_qualification_window(
                np.array([0.0, 1.0, 1.0]),
                QualificationWindow.absolute_seconds(0.0, 1.0),
            )
        with self.assertRaisesRegex(ValueError, r"within \[0, 1\]"):
            select_qualification_window(
                np.array([0.0, 1.0]),
                QualificationWindow.duration_fraction(-0.1, 0.5),
            )
        with self.assertRaisesRegex(ValueError, "time-history coverage"):
            select_qualification_window(
                np.array([0.0, 1.0]),
                QualificationWindow.absolute_seconds(0.0, 2.0),
            )


class IndependentRunReductionTests(unittest.TestCase):
    def test_reduces_epochs_within_each_run_before_ensemble_inference(self) -> None:
        time_s = np.array([0.0, 1.0, 2.0, 3.0])
        values = np.array(
            [
                [100.0, 1.0, 3.0, 100.0],
                [100.0, 5.0, 7.0, 100.0],
                [100.0, 9.0, 11.0, 100.0],
            ]
        )

        reduction = reduce_window_mean_per_run(
            time_s,
            values,
            QualificationWindow.absolute_seconds(1.0, 2.0),
        )
        interval = student_t_mean_confidence_interval(reduction.run_means)

        np.testing.assert_allclose(reduction.run_means, np.array([2.0, 6.0, 10.0]))
        self.assertEqual(reduction.epoch_count, 2)
        self.assertEqual(reduction.run_count, 3)
        self.assertEqual(interval.sample_count, 3)

    def test_reports_partial_finite_coverage_and_can_require_complete_evidence(
        self,
    ) -> None:
        time_s = np.array([0.0, 1.0, 2.0])
        values = np.array([[1.0, np.nan, 3.0], [2.0, 4.0, 6.0]])
        reduction = reduce_window_mean_per_run(
            time_s, values, QualificationWindow.absolute_seconds(0.0, 2.0)
        )
        np.testing.assert_allclose(reduction.run_means, np.array([2.0, 4.0]))
        self.assertEqual(reduction.selected_value_count, 6)
        self.assertEqual(reduction.finite_value_count, 5)
        self.assertEqual(reduction.minimum_finite_epoch_count_per_run, 2)
        self.assertAlmostEqual(reduction.finite_coverage, 5.0 / 6.0)

        with self.assertRaisesRegex(ValueError, "nonfinite selected evidence"):
            reduce_window_mean_per_run(
                time_s,
                values,
                QualificationWindow.absolute_seconds(0.0, 2.0),
                require_all_finite=True,
            )

        values[0, :] = np.nan
        with self.assertRaisesRegex(ValueError, r"run indices \[0\]"):
            reduce_window_mean_per_run(
                time_s, values, QualificationWindow.absolute_seconds(0.0, 2.0)
            )

    def test_rejects_history_shape_mismatch(self) -> None:
        with self.assertRaisesRegex(ValueError, "shape"):
            reduce_window_mean_per_run(
                np.array([0.0, 1.0]),
                np.array([1.0, 2.0]),
                QualificationWindow.absolute_seconds(0.0, 1.0),
            )
        with self.assertRaisesRegex(ValueError, "match time_s"):
            reduce_window_mean_per_run(
                np.array([0.0, 1.0]),
                np.ones((2, 3)),
                QualificationWindow.absolute_seconds(0.0, 1.0),
            )


class StudentConfidenceIntervalTests(unittest.TestCase):
    def test_matches_student_t_definition(self) -> None:
        values = np.array([1.0, 2.0, 4.0, 8.0])

        interval = student_t_mean_confidence_interval(values, confidence=0.90)

        expected_mean = float(np.mean(values))
        expected_standard_error = float(np.std(values, ddof=1) / np.sqrt(values.size))
        expected_half_width = float(
            student_t.ppf(0.95, df=values.size - 1) * expected_standard_error
        )
        self.assertAlmostEqual(interval.mean, expected_mean)
        self.assertAlmostEqual(interval.standard_error, expected_standard_error)
        self.assertAlmostEqual(interval.lower, expected_mean - expected_half_width)
        self.assertAlmostEqual(interval.upper, expected_mean + expected_half_width)

    def test_rejects_non_independent_shape_small_sample_and_bad_confidence(self) -> None:
        with self.assertRaisesRegex(ValueError, "at least two"):
            student_t_mean_confidence_interval(np.array([1.0]))
        with self.assertRaisesRegex(ValueError, "independent scalar"):
            student_t_mean_confidence_interval(np.ones((2, 2)))
        with self.assertRaisesRegex(ValueError, "strictly between"):
            student_t_mean_confidence_interval(np.array([1.0, 2.0]), confidence=1.0)


class QualificationCheckTests(unittest.TestCase):
    def test_equivalence_requires_entire_interval_inside_bounds(self) -> None:
        self.assertTrue(evaluate_equivalence("inside", _interval(1.0, 0.9, 1.1), 0.8, 1.2).passed)
        self.assertFalse(evaluate_equivalence("low", _interval(1.0, 0.7, 1.1), 0.8, 1.2).passed)
        self.assertFalse(evaluate_equivalence("high", _interval(1.0, 0.9, 1.3), 0.8, 1.2).passed)

    def test_one_sided_checks_use_conservative_interval_edge(self) -> None:
        self.assertTrue(evaluate_lower_bound("lower", _interval(5.0, 4.0, 6.0), 4.0).passed)
        self.assertFalse(evaluate_lower_bound("lower", _interval(5.0, 3.9, 6.0), 4.0).passed)
        self.assertTrue(evaluate_upper_bound("upper", _interval(5.0, 4.0, 6.0), 6.0).passed)
        self.assertFalse(evaluate_upper_bound("upper", _interval(5.0, 4.0, 6.1), 6.0).passed)

    def test_aggregate_distinguishes_fail_known_finding_and_information(self) -> None:
        passing = evaluate_upper_bound("required", _interval(1.0, 0.9, 1.1), 2.0)
        known = evaluate_upper_bound(
            "known",
            _interval(3.0, 2.9, 3.1),
            2.0,
            QualificationDisposition.KNOWN_FINDING,
        )
        informational = evaluate_upper_bound(
            "info",
            _interval(3.0, 2.9, 3.1),
            2.0,
            QualificationDisposition.INFORMATIONAL,
        )

        self.assertEqual(
            aggregate_qualification([passing, informational]).status,
            QualificationStatus.PASS,
        )
        known_report = aggregate_qualification([passing, known, informational])
        self.assertEqual(
            known_report.status, QualificationStatus.PASS_WITH_KNOWN_FINDINGS
        )
        self.assertFalse(known_report.passed)

        required_failure = evaluate_lower_bound(
            "required_failure", _interval(1.0, 0.9, 1.1), 2.0
        )
        self.assertEqual(
            aggregate_qualification([required_failure, known]).status,
            QualificationStatus.FAIL,
        )

    def test_aggregate_rejects_duplicate_check_names(self) -> None:
        check = evaluate_upper_bound("duplicate", _interval(1.0, 0.9, 1.1), 2.0)
        with self.assertRaisesRegex(ValueError, "unique"):
            aggregate_qualification([check, check])

    def test_aggregate_rejects_empty_evidence(self) -> None:
        with self.assertRaisesRegex(ValueError, "at least one check"):
            aggregate_qualification([])


class QualificationBaselineTests(unittest.TestCase):
    def setUp(self) -> None:
        self.check = evaluate_equivalence(
            "normalized_nees",
            _interval(1.0, 0.95, 1.05),
            0.8,
            1.2,
        )
        self.report = aggregate_qualification([self.check])

    def test_snapshot_and_deltas_are_versioned_and_traceable(self) -> None:
        snapshot = qualification_baseline_snapshot(
            self.report, {"campaign": "synthetic"}
        )
        self.assertEqual(snapshot["schema"], QUALIFICATION_BASELINE_SCHEMA)
        self.assertEqual(snapshot["qualification_status"], "pass")
        self.assertEqual(snapshot["metadata"], {"campaign": "synthetic"})

        updated_check = evaluate_equivalence(
            "normalized_nees",
            _interval(1.1, 1.0, 1.2),
            0.8,
            1.2,
        )
        deltas = qualification_baseline_deltas(
            aggregate_qualification([updated_check]), snapshot
        )
        self.assertEqual(len(deltas), 1)
        self.assertAlmostEqual(deltas[0].estimate_delta, 0.1)
        self.assertAlmostEqual(deltas[0].lower_delta, 0.05)
        self.assertAlmostEqual(deltas[0].upper_delta, 0.15)

    def test_delta_rejects_wrong_schema_and_missing_check(self) -> None:
        with self.assertRaisesRegex(ValueError, "schema"):
            qualification_baseline_deltas(self.report, {"schema": "wrong.v1"})
        with self.assertRaisesRegex(ValueError, "missing"):
            qualification_baseline_deltas(
                self.report,
                {"schema": QUALIFICATION_BASELINE_SCHEMA, "checks": {}},
            )

    def test_delta_rejects_changed_contract_and_unaccepted_baseline(self) -> None:
        snapshot = qualification_baseline_snapshot(self.report)
        checks = snapshot["checks"]
        assert isinstance(checks, dict)
        check = checks["normalized_nees"]
        assert isinstance(check, dict)
        check["confidence"] = 0.90
        with self.assertRaisesRegex(ValueError, "contract does not match"):
            qualification_baseline_deltas(self.report, snapshot)

        snapshot = qualification_baseline_snapshot(self.report)
        snapshot["qualification_status"] = "fail"
        with self.assertRaisesRegex(ValueError, "accepted qualification"):
            qualification_baseline_deltas(self.report, snapshot)

    def test_delta_rejects_inconsistent_serialized_claims(self) -> None:
        def baseline_check(snapshot: dict[str, object]) -> dict[str, object]:
            checks = snapshot["checks"]
            assert isinstance(checks, dict)
            check = checks["normalized_nees"]
            assert isinstance(check, dict)
            return check

        snapshot = qualification_baseline_snapshot(self.report)
        check = baseline_check(snapshot)
        check["mean"] = 2.0
        check["ci_lower"] = 1.9
        check["ci_upper"] = 2.1
        with self.assertRaisesRegex(ValueError, "passed status is inconsistent"):
            qualification_baseline_deltas(self.report, snapshot)

        snapshot = qualification_baseline_snapshot(self.report)
        check = baseline_check(snapshot)
        check["ci_lower"] = 1.1
        check["ci_upper"] = 0.9
        with self.assertRaisesRegex(ValueError, "lower <= mean <= upper"):
            qualification_baseline_deltas(self.report, snapshot)

        snapshot = qualification_baseline_snapshot(self.report)
        check = baseline_check(snapshot)
        check["mean"] = 1.1
        with self.assertRaisesRegex(ValueError, "lower <= mean <= upper"):
            qualification_baseline_deltas(self.report, snapshot)

    def test_delta_rejects_extra_check_and_inconsistent_known_finding_status(self) -> None:
        snapshot = qualification_baseline_snapshot(self.report)
        checks = snapshot["checks"]
        assert isinstance(checks, dict)
        checks["stale"] = dict(checks["normalized_nees"])
        with self.assertRaisesRegex(ValueError, "unexpected=.*stale"):
            qualification_baseline_deltas(self.report, snapshot)

        known_check = evaluate_upper_bound(
            "known",
            _interval(3.0, 2.9, 3.1),
            2.0,
            QualificationDisposition.KNOWN_FINDING,
        )
        known_report = aggregate_qualification([known_check])
        snapshot = qualification_baseline_snapshot(known_report)
        snapshot["qualification_status"] = "pass"
        with self.assertRaisesRegex(ValueError, "inconsistent"):
            qualification_baseline_deltas(known_report, snapshot)

    def test_writer_preserves_known_findings_and_rejects_required_failures(self) -> None:
        known_check = evaluate_upper_bound(
            "known",
            _interval(3.0, 2.9, 3.1),
            2.0,
            QualificationDisposition.KNOWN_FINDING,
        )
        known_report = aggregate_qualification([known_check])

        with tempfile.TemporaryDirectory(prefix="navkit_qualification_") as temp_dir:
            path = Path(temp_dir) / "nested" / "baseline.json"
            known_path = Path(temp_dir) / "known" / "baseline.json"
            write_qualification_baseline(known_path, known_report)
            self.assertEqual(
                json.loads(known_path.read_text(encoding="utf-8"))[
                    "qualification_status"
                ],
                "pass_with_known_findings",
            )

            failed_check = evaluate_lower_bound(
                "failed", _interval(1.0, 0.9, 1.1), 2.0
            )
            with self.assertRaisesRegex(ValueError, "required check failed"):
                write_qualification_baseline(
                    Path(temp_dir) / "failed" / "baseline.json",
                    aggregate_qualification([failed_check]),
                )

            written = write_qualification_baseline(
                path, self.report, {"campaign": "synthetic"}
            )
            loaded = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(loaded, written)
            self.assertEqual(loaded["schema"], QUALIFICATION_BASELINE_SCHEMA)
            self.assertEqual(list(path.parent.glob("*.tmp")), [])

            with self.assertRaises(FileExistsError):
                write_qualification_baseline(path, self.report)
            write_qualification_baseline(
                path,
                self.report,
                {"campaign": "replacement"},
                replace_existing=True,
            )
            replaced = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(replaced["metadata"], {"campaign": "replacement"})

    def test_atomic_writer_rejects_non_json_metadata_without_partial_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_qualification_") as temp_dir:
            path = Path(temp_dir) / "baseline.json"
            with self.assertRaisesRegex(ValueError, "strict-JSON"):
                write_qualification_baseline(
                    path,
                    self.report,
                    {"invalid": float("nan")},
                )
            self.assertFalse(path.exists())
            self.assertEqual(list(path.parent.glob("*.tmp")), [])


class QualificationSuiteParserTests(unittest.TestCase):
    def _write_suite_inputs(
        self, root: Path
    ) -> tuple[Path, dict[str, object], Path, Path]:
        deterministic = root / "deterministic.json"
        deterministic.write_text(
            json.dumps(
                {
                    "schema": "navkit.deterministic_regression_suite.v1",
                    "suite_name": "deterministic",
                }
            ),
            encoding="utf-8",
        )
        campaign = root / "campaign.json"
        campaign.write_text(
            json.dumps(
                {
                    "schema": "navkit.monte_carlo_campaign.v2",
                    "campaign_name": "synthetic_campaign",
                }
            ),
            encoding="utf-8",
        )
        document: dict[str, object] = {
            "schema": "navkit.qualification_suite.v1",
            "suite_name": "synthetic",
            "deterministic_suite": deterministic.name,
            "campaign_sizes": {"smoke": 2, "qualification": 10},
            "output": {"root": "output/qualification/synthetic"},
            "baseline": {"path": "baselines/synthetic.json"},
            "windows": {
                "steady": {"start_fraction": 0.2, "end_fraction": 1.0}
            },
            "campaigns": [
                {"name": "dynamic", "config": campaign.name, "window": "steady"}
            ],
            "criteria": [
                {
                    "name": "pva_nees",
                    "campaigns": ["*"],
                    "kind": "nees",
                    "group": "pva",
                    "metric": "normalized_window_mean",
                    "test": "equivalence",
                    "minimum": 0.7,
                    "maximum": 1.3,
                    "confidence": 0.95,
                    "disposition": "required",
                    "rationale": "Synthetic parser contract.",
                }
            ],
        }
        suite_path = root / "suite.json"
        suite_path.write_text(json.dumps(document), encoding="utf-8")
        return suite_path, document, deterministic, campaign

    def test_loads_and_resolves_versioned_linked_inputs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_qualification_") as temp_dir:
            root = Path(temp_dir)
            suite_path, _, deterministic, campaign = self._write_suite_inputs(root)

            suite = load_qualification_suite(suite_path)

            self.assertEqual(suite.name, "synthetic")
            self.assertEqual(suite.deterministic_suite, deterministic.resolve())
            self.assertEqual(suite.campaign_sizes["qualification"], 10)
            self.assertEqual(suite.campaigns[0].config, campaign.resolve())
            self.assertEqual(suite.campaigns[0].campaign_name, "synthetic_campaign")
            self.assertEqual(suite.criteria[0].campaigns, ("dynamic",))
            self.assertTrue(suite.windows["steady"].fractional)

    def test_rejects_wrong_linked_schema_and_duplicate_source_campaign(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_qualification_") as temp_dir:
            root = Path(temp_dir)
            suite_path, document, _, campaign = self._write_suite_inputs(root)
            campaign.write_text(
                json.dumps(
                    {
                        "schema": "navkit.deterministic_regression_suite.v1",
                        "campaign_name": "synthetic_campaign",
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "expected"):
                load_qualification_suite(suite_path)

            campaign.write_text(
                json.dumps(
                    {
                        "schema": "navkit.monte_carlo_campaign.v2",
                        "campaign_name": "synthetic_campaign",
                    }
                ),
                encoding="utf-8",
            )
            duplicate = root / "duplicate.json"
            duplicate.write_text(campaign.read_text(encoding="utf-8"), encoding="utf-8")
            campaigns = document["campaigns"]
            assert isinstance(campaigns, list)
            campaigns.append(
                {"name": "duplicate", "config": duplicate.name, "window": "steady"}
            )
            criteria = document["criteria"]
            assert isinstance(criteria, list)
            criterion = criteria[0]
            assert isinstance(criterion, dict)
            criterion["campaigns"] = ["dynamic", "duplicate"]
            suite_path.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "duplicates Monte Carlo campaign_name"):
                load_qualification_suite(suite_path)

    def test_rejects_ambiguous_bounds_metric_kind_and_unreferenced_campaign(self) -> None:
        mutations = (
            ("minimum.*must not provide maximum", {"test": "minimum"}),
            (
                "acceptance_rate requires kind 'nis'",
                {"metric": "acceptance_rate"},
            ),
            ("not referenced", {"campaigns": []}),
        )
        for expected, mutation in mutations:
            with self.subTest(expected=expected):
                with tempfile.TemporaryDirectory(
                    prefix="navkit_qualification_"
                ) as temp_dir:
                    root = Path(temp_dir)
                    suite_path, document, _, _ = self._write_suite_inputs(root)
                    criteria = document["criteria"]
                    assert isinstance(criteria, list)
                    criterion = criteria[0]
                    assert isinstance(criterion, dict)
                    criterion.update(mutation)
                    if mutation == {"campaigns": []}:
                        criterion["campaigns"] = ["dynamic"]
                        campaigns = document["campaigns"]
                        assert isinstance(campaigns, list)
                        campaigns.append(
                            {
                                "name": "unreferenced",
                                "config": "unreferenced.json",
                                "window": "steady",
                            }
                        )
                        (root / "unreferenced.json").write_text(
                            json.dumps(
                                {
                                    "schema": "navkit.monte_carlo_campaign.v2",
                                    "campaign_name": "unreferenced_campaign",
                                }
                            ),
                            encoding="utf-8",
                        )
                    suite_path.write_text(json.dumps(document), encoding="utf-8")
                    with self.assertRaisesRegex(ValueError, expected):
                        load_qualification_suite(suite_path)


if __name__ == "__main__":
    unittest.main()
    load_qualification_suite,
