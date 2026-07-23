# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Versioned schema and compatibility helpers for offline analysis artifacts."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping


ANALYSIS_BUNDLE_SCHEMA = "navkit.analysis_bundle.v1"
MONTE_CARLO_CAMPAIGN_SCHEMA = "navkit.monte_carlo_campaign.v1"
MONTE_CARLO_REPORT_SCHEMA = "navkit.monte_carlo_report.v1"
MONTE_CARLO_RUN_SCHEMA = "navkit.monte_carlo_run.v1"
PLOT_SPEC_SCHEMA = "navkit.plot_spec.v1"


class SchemaCompatibilityError(ValueError):
    """Raised when an artifact cannot be consumed by the selected analysis API."""


@dataclass(frozen=True)
class SchemaStatus:
    """Compatibility result for one source artifact."""

    expected: str
    observed: str | None
    compatible: bool
    legacy_unversioned: bool


def schema_family(schema: str) -> str:
    """Return a schema family without its trailing major-version marker."""
    family, separator, version = schema.rpartition(".v")
    if not separator or not family or not version.isdecimal():
        raise SchemaCompatibilityError(
            f"invalid schema '{schema}'; expected a name ending in '.v<major>'"
        )
    return family


def schema_major(schema: str) -> int:
    """Return the integer major version embedded in a NavKit schema name."""
    schema_family(schema)
    return int(schema.rpartition(".v")[2])


def validate_schema(
    metadata: Mapping[str, object],
    expected: str,
    source_description: str,
    *,
    allow_legacy_unversioned: bool = False,
) -> SchemaStatus:
    """Validate major-version compatibility for one metadata mapping.

    Existing CSV logs predate schema tags.  They remain readable only when the
    caller intentionally opts into the explicit legacy path; generated HDF5
    bundles and campaign/report JSON must always declare a compatible schema.
    """
    observed_value = metadata.get("schema")
    if observed_value is None:
        if allow_legacy_unversioned:
            return SchemaStatus(expected, None, True, True)
        raise SchemaCompatibilityError(
            f"{source_description} is missing required schema '{expected}'"
        )
    if not isinstance(observed_value, str):
        raise SchemaCompatibilityError(f"{source_description} has a non-string schema value")

    if schema_family(observed_value) != schema_family(expected):
        raise SchemaCompatibilityError(
            f"{source_description} uses schema '{observed_value}', expected '{expected}'"
        )
    if schema_major(observed_value) != schema_major(expected):
        raise SchemaCompatibilityError(
            f"{source_description} uses incompatible schema '{observed_value}', "
            f"expected major version {schema_major(expected)}"
        )
    return SchemaStatus(expected, observed_value, True, False)
