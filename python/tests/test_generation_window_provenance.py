# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Focused Monte Carlo generation-window provenance tests."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "python"))
sys.path.insert(0, str(REPOSITORY_ROOT / "tools"))

from navkit_analysis.analysis_performance import content_set_provenance  # noqa: E402

import run_monte_carlo  # noqa: E402


class GenerationWindowProvenanceTests(unittest.TestCase):
    """Verify narrow content identities and campaign drift rejection."""

    def test_content_set_identity_ignores_host_path_but_tracks_content(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_content_set_a_") as first_temp:
            with tempfile.TemporaryDirectory(prefix="navkit_content_set_b_") as second_temp:
                first = Path(first_temp) / "implementation.py"
                second = Path(second_temp) / "implementation.py"
                first.write_text("value = 1\n", encoding="utf-8")
                second.write_text("value = 1\n", encoding="utf-8")

                first_provenance = content_set_provenance({"implementation": first})
                second_provenance = content_set_provenance({"implementation": second})
                self.assertEqual(first_provenance["sha256"], second_provenance["sha256"])

                second.write_text("value = 2\n", encoding="utf-8")
                changed_provenance = content_set_provenance({"implementation": second})
                self.assertNotEqual(first_provenance["sha256"], changed_provenance["sha256"])

    def test_generation_window_allows_unrelated_git_state_change(self) -> None:
        start = {
            "git": {"revision": "a", "dirty": False, "dirty_tree_sha256": None},
            "build": {"artifact": "same"},
            "tooling": {"sha256": "same"},
        }
        completion = {
            "git": {"revision": "a", "dirty": True, "dirty_tree_sha256": "unrelated"},
            "build": {"artifact": "same"},
            "tooling": {"sha256": "same"},
        }

        provenance = run_monte_carlo._completed_execution_provenance(start, completion)

        self.assertTrue(provenance["generation_stable"])
        self.assertEqual(provenance["completion"], completion)
        self.assertEqual(provenance["git"], start["git"])

    def test_generation_window_rejects_build_artifact_drift(self) -> None:
        start = {"build": {"artifact": "first"}, "tooling": {"sha256": "same"}}
        completion = {"build": {"artifact": "second"}, "tooling": {"sha256": "same"}}

        with self.assertRaisesRegex(RuntimeError, "application artifact or build manifest"):
            run_monte_carlo._completed_execution_provenance(start, completion)

    def test_generation_window_rejects_tooling_drift(self) -> None:
        start = {"build": {"artifact": "same"}, "tooling": {"sha256": "first"}}
        completion = {"build": {"artifact": "same"}, "tooling": {"sha256": "second"}}

        with self.assertRaisesRegex(RuntimeError, "generation tooling changed"):
            run_monte_carlo._completed_execution_provenance(start, completion)


if __name__ == "__main__":
    unittest.main()
