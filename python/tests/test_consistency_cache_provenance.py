# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Consistency-cache provenance and invalidation tests."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import h5py

from navkit_analysis.consistency import (
    consistency_cache_provenance,
    load_consistency_cache,
    refresh_consistency_cache,
)
from navkit_analysis.schema import ANALYSIS_BUNDLE_SCHEMA


class ConsistencyCacheProvenanceTests(unittest.TestCase):
    """Verify cached statistics remain tied to their input and evaluator."""

    @staticmethod
    def _write_empty_bundle(path: Path, package_fingerprint: str = "package-a") -> None:
        with h5py.File(path, "w") as bundle:
            bundle.attrs["schema"] = ANALYSIS_BUNDLE_SCHEMA
            bundle.attrs["metadata"] = json.dumps(
                {
                    "package_fingerprint": package_fingerprint,
                    "storage": {"compression": "none"},
                }
            )
            bundle.create_group("runs")

    def test_refresh_writes_a_reusable_cache_contract(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_consistency_cache_") as temp:
            bundle_path = Path(temp) / "analysis_bundle.h5"
            self._write_empty_bundle(bundle_path)

            refresh_consistency_cache(bundle_path, max_points=40)
            provenance = consistency_cache_provenance(bundle_path, max_points=40)
            nees, nis, marginal = load_consistency_cache(bundle_path, max_points=40)

            fingerprint = provenance["fingerprint"]
            self.assertIsInstance(fingerprint, str)
            self.assertEqual(len(fingerprint), 64)
            contract = provenance["contract"]
            self.assertIsInstance(contract, dict)
            assert isinstance(contract, dict)
            self.assertEqual(contract["bundle_package_fingerprint"], "package-a")
            self.assertEqual(contract["max_points"], 40)
            self.assertEqual(contract["selected_kinds"], ["marginal", "nees", "nis"])
            self.assertEqual((nees, nis, marginal), ([], [], []))

    def test_bundle_fingerprint_change_invalidates_cache(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_consistency_bundle_") as temp:
            bundle_path = Path(temp) / "analysis_bundle.h5"
            self._write_empty_bundle(bundle_path)
            refresh_consistency_cache(bundle_path)

            with h5py.File(bundle_path, "r+") as bundle:
                metadata = json.loads(str(bundle.attrs["metadata"]))
                metadata["package_fingerprint"] = "package-b"
                bundle.attrs["metadata"] = json.dumps(metadata)

            with self.assertRaisesRegex(ValueError, "does not match"):
                load_consistency_cache(bundle_path)

    def test_evaluator_change_invalidates_cache(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_consistency_evaluator_") as temp:
            bundle_path = Path(temp) / "analysis_bundle.h5"
            self._write_empty_bundle(bundle_path)
            refresh_consistency_cache(bundle_path)

            with mock.patch(
                "navkit_analysis.consistency._consistency_evaluator_content_sha256",
                return_value="changed-evaluator",
            ):
                with self.assertRaisesRegex(ValueError, "does not match"):
                    load_consistency_cache(bundle_path)

    def test_legacy_cache_without_contract_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_consistency_legacy_") as temp:
            bundle_path = Path(temp) / "analysis_bundle.h5"
            self._write_empty_bundle(bundle_path)
            with h5py.File(bundle_path, "r+") as bundle:
                bundle.create_group("aggregate/consistency/series")

            with self.assertRaisesRegex(ValueError, "no provenance contract"):
                load_consistency_cache(bundle_path)

    def test_requested_cache_options_are_validated(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_consistency_options_") as temp:
            bundle_path = Path(temp) / "analysis_bundle.h5"
            self._write_empty_bundle(bundle_path)
            refresh_consistency_cache(bundle_path, max_points=40, selected_kinds=("nees",))

            load_consistency_cache(
                bundle_path,
                max_points=40,
                selected_kinds=("nees",),
            )
            with self.assertRaisesRegex(ValueError, "does not match"):
                load_consistency_cache(bundle_path, max_points=20, selected_kinds=("nees",))
            with self.assertRaisesRegex(ValueError, "does not match"):
                load_consistency_cache(bundle_path, max_points=40)


if __name__ == "__main__":
    unittest.main()
