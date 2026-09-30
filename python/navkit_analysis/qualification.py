# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Statistical qualification primitives for independent Monte Carlo runs.

The functions in this module operate on time histories already loaded from an
analysis bundle or consistency cache.  A qualification window is reduced to
one scalar per Monte Carlo run before any ensemble inference is performed.
This preserves the run, rather than the individual epoch, as the independent
experimental unit.
"""

from __future__ import annotations

import json
import os
import tempfile
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from scipy.stats import t as student_t

from navkit_analysis.schema import (
    DETERMINISTIC_REGRESSION_SUITE_SCHEMA,
    MONTE_CARLO_CAMPAIGN_SCHEMA,
    QUALIFICATION_BASELINE_SCHEMA,
    QUALIFICATION_SUITE_SCHEMA,
    validate_schema,
)


class QualificationDisposition(str, Enum):
    """Policy applied to the result of one qualification check."""

    REQUIRED = "required"
    KNOWN_FINDING = "known_finding"
    INFORMATIONAL = "informational"


class QualificationStatus(str, Enum):
    """Aggregate qualification status."""

    PASS = "pass"
    PASS_WITH_KNOWN_FINDINGS = "pass_with_known_findings"
    FAIL = "fail"


class QualificationCheckKind(str, Enum):
    """Supported confidence-interval check semantics."""

    EQUIVALENCE = "equivalence"
    LOWER_BOUND = "lower_bound"
    UPPER_BOUND = "upper_bound"


@dataclass(frozen=True)
class QualificationWindow:
    """Closed qualification window expressed in seconds or duration fractions."""

    start: float
    end: float
    fractional: bool = False

    @classmethod
    def absolute_seconds(cls, start_s: float, end_s: float) -> QualificationWindow:
        """Construct a window whose endpoints are absolute timestamps in seconds."""
        return cls(start=float(start_s), end=float(end_s), fractional=False)

    @classmethod
    def duration_fraction(
        cls, start_fraction: float, end_fraction: float
    ) -> QualificationWindow:
        """Construct a window relative to the observed time-history duration."""
        return cls(
            start=float(start_fraction),
            end=float(end_fraction),
            fractional=True,
        )


@dataclass(frozen=True)
class WindowSelection:
    """Resolved closed time window and its selected epoch mask."""

    start_s: float
    end_s: float
    mask: np.ndarray

    @property
    def epoch_count(self) -> int:
        """Return the number of selected cache epochs."""
        return int(np.count_nonzero(self.mask))


@dataclass(frozen=True)
class IndependentRunReduction:
    """One window-mean statistic for each independent Monte Carlo run."""

    run_means: np.ndarray
    window_start_s: float
    window_end_s: float
    epoch_count: int
    finite_value_count: int
    selected_value_count: int
    minimum_finite_epoch_count_per_run: int

    @property
    def run_count(self) -> int:
        """Return the number of independent run statistics."""
        return int(self.run_means.size)

    @property
    def finite_coverage(self) -> float:
        """Return the fraction of selected run/epoch values that were finite."""
        return float(self.finite_value_count / self.selected_value_count)


@dataclass(frozen=True)
class MeanConfidenceInterval:
    """Student-t confidence interval for an ensemble mean."""

    mean: float
    lower: float
    upper: float
    confidence: float
    sample_count: int
    sample_standard_deviation: float
    standard_error: float


@dataclass(frozen=True)
class QualificationCheck:
    """Result of applying one qualification contract to an ensemble CI."""

    name: str
    kind: QualificationCheckKind
    disposition: QualificationDisposition
    passed: bool
    confidence_interval: MeanConfidenceInterval
    minimum: float | None = None
    maximum: float | None = None


@dataclass(frozen=True)
class QualificationReport:
    """Aggregate status and checks for one qualification evaluation."""

    status: QualificationStatus
    checks: tuple[QualificationCheck, ...]

    @property
    def passed(self) -> bool:
        """Return true only for an unqualified pass with no active known finding."""
        return self.status is QualificationStatus.PASS

    @property
    def required_checks_passed(self) -> bool:
        """Return true when no required qualification contract failed."""
        return self.status is not QualificationStatus.FAIL


@dataclass(frozen=True)
class QualificationBaselineDelta:
    """Change in one qualification estimate and confidence interval."""

    name: str
    estimate_delta: float
    lower_delta: float
    upper_delta: float


@dataclass(frozen=True)
class QualificationCampaignSpec:
    """One Monte Carlo campaign selected by a qualification suite."""

    name: str
    config: Path
    campaign_name: str
    window_name: str


@dataclass(frozen=True)
class QualificationCriterionSpec:
    """One reusable qualification criterion applied to named campaigns."""

    name: str
    campaigns: tuple[str, ...]
    kind: str
    group: str
    metric: str
    test: str
    confidence: float
    disposition: QualificationDisposition
    rationale: str
    minimum: float | None = None
    maximum: float | None = None


@dataclass(frozen=True)
class QualificationSuite:
    """Validated configuration for deterministic and stochastic qualification."""

    name: str
    source: Path
    deterministic_suite: Path
    campaign_sizes: Mapping[str, int]
    output_root: Path
    baseline_path: Path
    windows: Mapping[str, QualificationWindow]
    campaigns: tuple[QualificationCampaignSpec, ...]
    criteria: tuple[QualificationCriterionSpec, ...]


def _required_object(value: object, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be an object")
    return value


def _required_string(value: object, path: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{path} must be a nonempty string")
    return value


def _reject_unknown_fields(
    value: Mapping[str, object], allowed: set[str], path: str
) -> None:
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise ValueError(f"{path} contains unknown fields {unknown}")


def _finite_float(value: object, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"{path} must be a finite number")
    converted = float(value)
    if not np.isfinite(converted):
        raise ValueError(f"{path} must be a finite number")
    return converted


def _load_versioned_object(path: Path, schema: str) -> dict[str, Any]:
    document = json.loads(path.read_text(encoding="utf-8"))
    root = _required_object(document, str(path))
    validate_schema(root, schema, str(path))
    return root


def _load_campaign_name(path: Path) -> str:
    root = _load_versioned_object(path, MONTE_CARLO_CAMPAIGN_SCHEMA)
    return _required_string(root.get("campaign_name"), f"{path}.campaign_name")


def _parse_window(value: object, path: str) -> QualificationWindow:
    window = _required_object(value, path)
    absolute_fields = {"start_s", "end_s"}
    fractional_fields = {"start_fraction", "end_fraction"}
    fields = set(window)
    if fields == absolute_fields:
        return QualificationWindow.absolute_seconds(
            _finite_float(window["start_s"], f"{path}.start_s"),
            _finite_float(window["end_s"], f"{path}.end_s"),
        )
    if fields == fractional_fields:
        return QualificationWindow.duration_fraction(
            _finite_float(window["start_fraction"], f"{path}.start_fraction"),
            _finite_float(window["end_fraction"], f"{path}.end_fraction"),
        )
    raise ValueError(
        f"{path} must contain exactly start_s/end_s or "
        "start_fraction/end_fraction"
    )


def load_qualification_suite(path: Path) -> QualificationSuite:
    """Load a versioned qualification suite and resolve all linked inputs."""
    source = path.resolve()
    document = json.loads(source.read_text(encoding="utf-8"))
    root = _required_object(document, str(source))
    validate_schema(root, QUALIFICATION_SUITE_SCHEMA, str(source))
    _reject_unknown_fields(
        root,
        {
            "schema",
            "suite_name",
            "deterministic_suite",
            "campaign_sizes",
            "output",
            "baseline",
            "windows",
            "campaigns",
            "criteria",
        },
        "qualification suite",
    )
    suite_name = _required_string(root.get("suite_name"), "suite_name")
    deterministic_suite = (
        source.parent
        / _required_string(root.get("deterministic_suite"), "deterministic_suite")
    ).resolve()
    if not deterministic_suite.is_file():
        raise ValueError(
            f"deterministic_suite does not exist: {deterministic_suite}"
        )
    _load_versioned_object(
        deterministic_suite, DETERMINISTIC_REGRESSION_SUITE_SCHEMA
    )

    sizes_value = _required_object(root.get("campaign_sizes"), "campaign_sizes")
    if not sizes_value:
        raise ValueError("campaign_sizes must not be empty")
    campaign_sizes: dict[str, int] = {}
    for name, count in sizes_value.items():
        if not isinstance(name, str) or not name:
            raise ValueError("campaign_sizes names must be nonempty strings")
        if isinstance(count, bool) or not isinstance(count, int) or count < 2:
            raise ValueError(f"campaign_sizes.{name} must be an integer of at least two")
        campaign_sizes[name] = count

    output = _required_object(root.get("output"), "output")
    _reject_unknown_fields(output, {"root"}, "output")
    output_root = Path(_required_string(output.get("root"), "output.root"))
    baseline = _required_object(root.get("baseline"), "baseline")
    _reject_unknown_fields(baseline, {"path"}, "baseline")
    baseline_path = (
        source.parent / _required_string(baseline.get("path"), "baseline.path")
    ).resolve()

    windows_value = _required_object(root.get("windows"), "windows")
    if not windows_value:
        raise ValueError("windows must not be empty")
    windows: dict[str, QualificationWindow] = {}
    for name, value in windows_value.items():
        if not isinstance(name, str) or not name:
            raise ValueError("windows names must be nonempty strings")
        windows[name] = _parse_window(value, f"windows.{name}")

    campaigns_value = root.get("campaigns")
    if not isinstance(campaigns_value, list) or not campaigns_value:
        raise ValueError("campaigns must be a nonempty array")
    campaigns: list[QualificationCampaignSpec] = []
    campaign_names: set[str] = set()
    source_campaign_names: set[str] = set()
    for index, value in enumerate(campaigns_value):
        item_path = f"campaigns[{index}]"
        campaign = _required_object(value, item_path)
        _reject_unknown_fields(campaign, {"name", "config", "window"}, item_path)
        name = _required_string(campaign.get("name"), f"{item_path}.name")
        if name in campaign_names:
            raise ValueError(f"duplicate campaign name '{name}'")
        campaign_names.add(name)
        config = (
            source.parent
            / _required_string(campaign.get("config"), f"{item_path}.config")
        ).resolve()
        if not config.is_file():
            raise ValueError(f"{item_path}.config does not exist: {config}")
        window_name = _required_string(
            campaign.get("window"), f"{item_path}.window"
        )
        if window_name not in windows:
            raise ValueError(f"{item_path}.window references unknown window '{window_name}'")
        source_campaign_name = _load_campaign_name(config)
        if source_campaign_name in source_campaign_names:
            raise ValueError(
                f"{item_path}.config duplicates Monte Carlo campaign_name "
                f"'{source_campaign_name}'"
            )
        source_campaign_names.add(source_campaign_name)
        campaigns.append(
            QualificationCampaignSpec(
                name=name,
                config=config,
                campaign_name=source_campaign_name,
                window_name=window_name,
            )
        )

    criteria_value = root.get("criteria")
    if not isinstance(criteria_value, list) or not criteria_value:
        raise ValueError("criteria must be a nonempty array")
    criteria: list[QualificationCriterionSpec] = []
    criterion_names: set[str] = set()
    for index, value in enumerate(criteria_value):
        item_path = f"criteria[{index}]"
        criterion = _required_object(value, item_path)
        _reject_unknown_fields(
            criterion,
            {
                "name",
                "campaigns",
                "kind",
                "group",
                "metric",
                "test",
                "confidence",
                "disposition",
                "rationale",
                "minimum",
                "maximum",
            },
            item_path,
        )
        name = _required_string(criterion.get("name"), f"{item_path}.name")
        if name in criterion_names:
            raise ValueError(f"duplicate criterion name '{name}'")
        criterion_names.add(name)
        selected = criterion.get("campaigns")
        if not isinstance(selected, list) or not selected or not all(
            isinstance(item, str) and item for item in selected
        ):
            raise ValueError(f"{item_path}.campaigns must be a nonempty string array")
        selected_names = (
            tuple(campaign.name for campaign in campaigns)
            if selected == ["*"]
            else tuple(selected)
        )
        if len(selected_names) != len(set(selected_names)):
            raise ValueError(f"{item_path}.campaigns contains duplicate names")
        unknown = sorted(set(selected_names) - campaign_names)
        if unknown:
            raise ValueError(f"{item_path}.campaigns contains unknown names {unknown}")
        test = _required_string(criterion.get("test"), f"{item_path}.test")
        if test not in {"equivalence", "minimum", "maximum"}:
            raise ValueError(f"{item_path}.test is unsupported: {test}")
        metric = _required_string(criterion.get("metric"), f"{item_path}.metric")
        if metric not in {"normalized_window_mean", "acceptance_rate"}:
            raise ValueError(f"{item_path}.metric is unsupported: {metric}")
        minimum = (
            _finite_float(criterion["minimum"], f"{item_path}.minimum")
            if "minimum" in criterion
            else None
        )
        maximum = (
            _finite_float(criterion["maximum"], f"{item_path}.maximum")
            if "maximum" in criterion
            else None
        )
        if test == "equivalence" and (minimum is None or maximum is None):
            raise ValueError(f"{item_path} equivalence requires minimum and maximum")
        if test == "equivalence" and maximum is not None and minimum is not None:
            if maximum < minimum:
                raise ValueError(f"{item_path} equivalence bounds must be ordered")
        if test == "minimum" and minimum is None:
            raise ValueError(f"{item_path} minimum test requires minimum")
        if test == "minimum" and maximum is not None:
            raise ValueError(f"{item_path} minimum test must not provide maximum")
        if test == "maximum" and maximum is None:
            raise ValueError(f"{item_path} maximum test requires maximum")
        if test == "maximum" and minimum is not None:
            raise ValueError(f"{item_path} maximum test must not provide minimum")
        confidence = _finite_float(
            criterion.get("confidence", 0.95), f"{item_path}.confidence"
        )
        if confidence <= 0.0 or confidence >= 1.0:
            raise ValueError(f"{item_path}.confidence must lie between zero and one")
        try:
            disposition = QualificationDisposition(
                _required_string(
                    criterion.get("disposition"), f"{item_path}.disposition"
                )
            )
        except ValueError as error:
            raise ValueError(f"{item_path}.disposition is unsupported") from error
        kind = _required_string(criterion.get("kind"), f"{item_path}.kind")
        if kind not in {"nees", "nis"}:
            raise ValueError(f"{item_path}.kind is unsupported: {kind}")
        if metric == "acceptance_rate" and kind != "nis":
            raise ValueError(f"{item_path}.acceptance_rate requires kind 'nis'")
        criteria.append(
            QualificationCriterionSpec(
                name=name,
                campaigns=selected_names,
                kind=kind,
                group=_required_string(criterion.get("group"), f"{item_path}.group"),
                metric=metric,
                test=test,
                confidence=confidence,
                disposition=disposition,
                rationale=_required_string(
                    criterion.get("rationale"), f"{item_path}.rationale"
                ),
                minimum=minimum,
                maximum=maximum,
            )
        )

    referenced_campaigns = {
        campaign_name
        for criterion in criteria
        for campaign_name in criterion.campaigns
    }
    unreferenced_campaigns = sorted(campaign_names - referenced_campaigns)
    if unreferenced_campaigns:
        raise ValueError(
            "campaigns are not referenced by any qualification criterion: "
            f"{unreferenced_campaigns}"
        )

    return QualificationSuite(
        name=suite_name,
        source=source,
        deterministic_suite=deterministic_suite,
        campaign_sizes=campaign_sizes,
        output_root=output_root,
        baseline_path=baseline_path,
        windows=windows,
        campaigns=tuple(campaigns),
        criteria=tuple(criteria),
    )


def _time_history(time_s: np.ndarray) -> np.ndarray:
    converted = np.asarray(time_s, dtype=float)
    if converted.ndim != 1 or converted.size == 0:
        raise ValueError("time_s must be a nonempty one-dimensional array")
    if not np.isfinite(converted).all():
        raise ValueError("time_s must contain only finite values")
    if converted.size > 1 and np.any(np.diff(converted) <= 0.0):
        raise ValueError("time_s must be strictly increasing")
    return converted


def select_qualification_window(
    time_s: np.ndarray, window: QualificationWindow
) -> WindowSelection:
    """Resolve and select a closed absolute or fractional qualification window."""
    times = _time_history(time_s)
    if not np.isfinite(window.start) or not np.isfinite(window.end):
        raise ValueError("qualification window endpoints must be finite")
    if window.end < window.start:
        raise ValueError("qualification window end must not precede its start")

    if window.fractional:
        if window.start < 0.0 or window.end > 1.0:
            raise ValueError("fractional qualification windows must lie within [0, 1]")
        duration_s = times[-1] - times[0]
        start_s = times[0] + (window.start * duration_s)
        end_s = times[0] + (window.end * duration_s)
    else:
        start_s = window.start
        end_s = window.end
        if start_s < times[0] or end_s > times[-1]:
            raise ValueError(
                "absolute qualification window must lie within the time-history coverage"
            )

    mask = (times >= start_s) & (times <= end_s)
    if not np.any(mask):
        raise ValueError("qualification window selects no cache epochs")
    return WindowSelection(start_s=float(start_s), end_s=float(end_s), mask=mask)


def reduce_window_mean_per_run(
    time_s: np.ndarray,
    values: np.ndarray,
    window: QualificationWindow,
    *,
    require_all_finite: bool = False,
) -> IndependentRunReduction:
    """Reduce a time window to one arithmetic mean per independent run.

    Finite epochs are averaged within each run.  A run with no finite value in
    the selected window is rejected rather than silently reducing the ensemble
    sample count.  Qualification criteria that require complete evidence can
    set ``require_all_finite`` to reject any missing selected run/epoch value.
    """
    times = _time_history(time_s)
    histories = np.asarray(values, dtype=float)
    if histories.ndim != 2:
        raise ValueError("values must have shape (run, epoch)")
    if histories.shape[0] == 0 or histories.shape[1] != times.size:
        raise ValueError("values must contain at least one run and match time_s epochs")

    selection = select_qualification_window(times, window)
    selected = histories[:, selection.mask]
    finite = np.isfinite(selected)
    finite_counts = np.count_nonzero(finite, axis=1)
    missing_runs = np.flatnonzero(finite_counts == 0)
    if missing_runs.size:
        raise ValueError(
            "qualification window has no finite samples for run indices "
            f"{missing_runs.tolist()}"
        )
    nonfinite_counts = selected.shape[1] - finite_counts
    incomplete_runs = np.flatnonzero(nonfinite_counts > 0)
    if require_all_finite and incomplete_runs.size:
        evidence_gaps = {
            int(index): int(nonfinite_counts[index]) for index in incomplete_runs
        }
        raise ValueError(
            "qualification window contains nonfinite selected evidence; "
            f"nonfinite sample counts by run index are {evidence_gaps}"
        )
    run_means = np.sum(np.where(finite, selected, 0.0), axis=1) / finite_counts
    selected_value_count = int(selected.size)
    return IndependentRunReduction(
        run_means=np.asarray(run_means, dtype=float),
        window_start_s=selection.start_s,
        window_end_s=selection.end_s,
        epoch_count=selection.epoch_count,
        finite_value_count=int(np.count_nonzero(finite)),
        selected_value_count=selected_value_count,
        minimum_finite_epoch_count_per_run=int(np.min(finite_counts)),
    )


def student_t_mean_confidence_interval(
    independent_values: np.ndarray,
    confidence: float = 0.95,
) -> MeanConfidenceInterval:
    """Return a two-sided Student-t CI from independent scalar observations."""
    values = np.asarray(independent_values, dtype=float)
    if values.ndim != 1 or values.size < 2:
        raise ValueError("at least two independent scalar observations are required")
    if not np.isfinite(values).all():
        raise ValueError("independent observations must contain only finite values")
    if not np.isfinite(confidence) or confidence <= 0.0 or confidence >= 1.0:
        raise ValueError("confidence must lie strictly between zero and one")

    sample_count = int(values.size)
    mean = float(np.mean(values))
    sample_standard_deviation = float(np.std(values, ddof=1))
    standard_error = sample_standard_deviation / np.sqrt(float(sample_count))
    critical_value = float(
        student_t.ppf(0.5 + (0.5 * confidence), df=sample_count - 1)
    )
    half_width = critical_value * standard_error
    return MeanConfidenceInterval(
        mean=mean,
        lower=mean - half_width,
        upper=mean + half_width,
        confidence=float(confidence),
        sample_count=sample_count,
        sample_standard_deviation=sample_standard_deviation,
        standard_error=standard_error,
    )


def evaluate_equivalence(
    name: str,
    confidence_interval: MeanConfidenceInterval,
    minimum: float,
    maximum: float,
    disposition: QualificationDisposition = QualificationDisposition.REQUIRED,
) -> QualificationCheck:
    """Pass when the entire confidence interval lies within closed bounds."""
    if not np.isfinite(minimum) or not np.isfinite(maximum) or maximum < minimum:
        raise ValueError("equivalence bounds must be finite and ordered")
    return QualificationCheck(
        name=name,
        kind=QualificationCheckKind.EQUIVALENCE,
        disposition=disposition,
        passed=bool(
            confidence_interval.lower >= minimum
            and confidence_interval.upper <= maximum
        ),
        confidence_interval=confidence_interval,
        minimum=float(minimum),
        maximum=float(maximum),
    )


def evaluate_lower_bound(
    name: str,
    confidence_interval: MeanConfidenceInterval,
    minimum: float,
    disposition: QualificationDisposition = QualificationDisposition.REQUIRED,
) -> QualificationCheck:
    """Pass when the entire confidence interval lies at or above a lower bound."""
    if not np.isfinite(minimum):
        raise ValueError("lower bound must be finite")
    return QualificationCheck(
        name=name,
        kind=QualificationCheckKind.LOWER_BOUND,
        disposition=disposition,
        passed=bool(confidence_interval.lower >= minimum),
        confidence_interval=confidence_interval,
        minimum=float(minimum),
    )


def evaluate_upper_bound(
    name: str,
    confidence_interval: MeanConfidenceInterval,
    maximum: float,
    disposition: QualificationDisposition = QualificationDisposition.REQUIRED,
) -> QualificationCheck:
    """Pass when the entire confidence interval lies at or below an upper bound."""
    if not np.isfinite(maximum):
        raise ValueError("upper bound must be finite")
    return QualificationCheck(
        name=name,
        kind=QualificationCheckKind.UPPER_BOUND,
        disposition=disposition,
        passed=bool(confidence_interval.upper <= maximum),
        confidence_interval=confidence_interval,
        maximum=float(maximum),
    )


def qualification_check_claim_passes(
    kind: QualificationCheckKind | str,
    mean: float,
    lower: float,
    upper: float,
    minimum: float | None,
    maximum: float | None,
) -> bool:
    """Validate a serialized confidence-interval claim and recompute its result."""
    try:
        converted_kind = QualificationCheckKind(kind)
    except ValueError as error:
        raise ValueError(f"unsupported qualification check kind: {kind}") from error
    numeric_values = np.asarray([mean, lower, upper], dtype=float)
    if not np.isfinite(numeric_values).all():
        raise ValueError("qualification confidence interval must contain finite values")
    if lower > mean or mean > upper:
        raise ValueError(
            "qualification confidence interval must satisfy lower <= mean <= upper"
        )
    if converted_kind is QualificationCheckKind.EQUIVALENCE:
        if minimum is None or maximum is None:
            raise ValueError("equivalence qualification claim requires both bounds")
        if not np.isfinite(minimum) or not np.isfinite(maximum) or maximum < minimum:
            raise ValueError("equivalence qualification bounds must be finite and ordered")
        return bool(lower >= minimum and upper <= maximum)
    if converted_kind is QualificationCheckKind.LOWER_BOUND:
        if minimum is None or maximum is not None or not np.isfinite(minimum):
            raise ValueError("lower-bound qualification claim requires one finite minimum")
        return bool(lower >= minimum)
    if maximum is None or minimum is not None or not np.isfinite(maximum):
        raise ValueError("upper-bound qualification claim requires one finite maximum")
    return bool(upper <= maximum)


def aggregate_qualification(
    checks: Sequence[QualificationCheck],
) -> QualificationReport:
    """Aggregate check dispositions without hiding active known findings."""
    converted = tuple(checks)
    if not converted:
        raise ValueError("qualification requires at least one check")
    names = [check.name for check in converted]
    if len(names) != len(set(names)):
        raise ValueError("qualification check names must be unique")

    required_failure = any(
        check.disposition is QualificationDisposition.REQUIRED and not check.passed
        for check in converted
    )
    active_known_finding = any(
        check.disposition is QualificationDisposition.KNOWN_FINDING and not check.passed
        for check in converted
    )
    if required_failure:
        status = QualificationStatus.FAIL
    elif active_known_finding:
        status = QualificationStatus.PASS_WITH_KNOWN_FINDINGS
    else:
        status = QualificationStatus.PASS
    return QualificationReport(status=status, checks=converted)


def qualification_baseline_snapshot(
    report: QualificationReport,
    metadata: Mapping[str, object] | None = None,
) -> dict[str, object]:
    """Create a versioned, JSON-compatible baseline snapshot."""
    checks: dict[str, object] = {}
    for check in report.checks:
        interval = check.confidence_interval
        checks[check.name] = {
            "kind": check.kind.value,
            "disposition": check.disposition.value,
            "passed": bool(check.passed),
            "mean": interval.mean,
            "ci_lower": interval.lower,
            "ci_upper": interval.upper,
            "confidence": interval.confidence,
            "sample_count": interval.sample_count,
            "minimum": check.minimum,
            "maximum": check.maximum,
        }
    return {
        "schema": QUALIFICATION_BASELINE_SCHEMA,
        "qualification_status": report.status.value,
        "metadata": dict(metadata or {}),
        "checks": checks,
    }


def qualification_baseline_deltas(
    report: QualificationReport,
    baseline: Mapping[str, object],
) -> tuple[QualificationBaselineDelta, ...]:
    """Return current-minus-baseline deltas for matching qualification checks."""
    if baseline.get("schema") != QUALIFICATION_BASELINE_SCHEMA:
        raise ValueError(
            f"baseline must declare schema '{QUALIFICATION_BASELINE_SCHEMA}'"
        )
    baseline_checks = baseline.get("checks")
    if not isinstance(baseline_checks, Mapping):
        raise ValueError("baseline checks must be an object")
    current_names = {check.name for check in report.checks}
    baseline_names = set(baseline_checks)
    if baseline_names != current_names:
        raise ValueError(
            "baseline qualification checks do not match current checks; "
            f"missing={sorted(current_names - baseline_names)}, "
            f"unexpected={sorted(baseline_names - current_names)}"
        )

    baseline_status = baseline.get("qualification_status")
    if baseline_status not in {
        QualificationStatus.PASS.value,
        QualificationStatus.PASS_WITH_KNOWN_FINDINGS.value,
    }:
        raise ValueError("baseline does not contain accepted qualification evidence")

    deltas: list[QualificationBaselineDelta] = []
    active_known_finding = False
    for check in report.checks:
        baseline_check = baseline_checks.get(check.name)
        if not isinstance(baseline_check, Mapping):
            raise ValueError(f"baseline is missing qualification check '{check.name}'")
        expected_contract = {
            "kind": check.kind.value,
            "disposition": check.disposition.value,
            "confidence": check.confidence_interval.confidence,
            "minimum": check.minimum,
            "maximum": check.maximum,
        }
        observed_contract = {
            field: baseline_check.get(field) for field in expected_contract
        }
        if observed_contract != expected_contract:
            raise ValueError(
                f"baseline check '{check.name}' contract does not match current evidence"
            )
        baseline_passed = baseline_check.get("passed")
        if not isinstance(baseline_passed, bool):
            raise ValueError(f"baseline check '{check.name}' has invalid passed status")
        try:
            baseline_mean = float(baseline_check["mean"])
            baseline_lower = float(baseline_check["ci_lower"])
            baseline_upper = float(baseline_check["ci_upper"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                f"baseline check '{check.name}' has invalid confidence-interval values"
            ) from error
        if not np.isfinite([baseline_mean, baseline_lower, baseline_upper]).all():
            raise ValueError(f"baseline check '{check.name}' contains nonfinite values")
        expected_passed = qualification_check_claim_passes(
            check.kind,
            baseline_mean,
            baseline_lower,
            baseline_upper,
            check.minimum,
            check.maximum,
        )
        if baseline_passed is not expected_passed:
            raise ValueError(
                f"baseline check '{check.name}' passed status is inconsistent "
                "with its confidence interval and contract"
            )
        if (
            check.disposition is QualificationDisposition.REQUIRED
            and not expected_passed
        ):
            raise ValueError(
                f"baseline check '{check.name}' records a failed required contract"
            )
        active_known_finding = active_known_finding or (
            check.disposition is QualificationDisposition.KNOWN_FINDING
            and not expected_passed
        )
        interval = check.confidence_interval
        deltas.append(
            QualificationBaselineDelta(
                name=check.name,
                estimate_delta=interval.mean - baseline_mean,
                lower_delta=interval.lower - baseline_lower,
                upper_delta=interval.upper - baseline_upper,
            )
        )
    expected_status = (
        QualificationStatus.PASS_WITH_KNOWN_FINDINGS.value
        if active_known_finding
        else QualificationStatus.PASS.value
    )
    if baseline_status != expected_status:
        raise ValueError(
            "baseline qualification_status is inconsistent with its check dispositions"
        )
    return tuple(deltas)


def write_qualification_baseline(
    path: Path,
    report: QualificationReport,
    metadata: Mapping[str, object] | None = None,
    *,
    replace_existing: bool = False,
) -> dict[str, object]:
    """Atomically write accepted evidence through an explicit overwrite guard.

    Active known findings are retained in the baseline and its aggregate
    ``pass_with_known_findings`` status.  A failed required check always blocks
    baseline creation or replacement.
    """
    if not report.required_checks_passed:
        raise ValueError(
            "qualification baseline cannot be written when a required check failed"
        )
    snapshot = qualification_baseline_snapshot(report, metadata)
    try:
        serialized = json.dumps(snapshot, indent=2, allow_nan=False) + "\n"
    except (TypeError, ValueError) as error:
        raise ValueError("qualification baseline is not strict-JSON serializable") from error

    destination = path.resolve()
    if destination.exists() and not replace_existing:
        raise FileExistsError(
            f"qualification baseline already exists: {destination}; "
            "set replace_existing=True to replace it"
        )
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=destination.parent,
            prefix=f".{destination.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary:
            temporary_path = Path(temporary.name)
            temporary.write(serialized)
            temporary.flush()
            os.fsync(temporary.fileno())
        temporary_path.replace(destination)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()
    return snapshot
