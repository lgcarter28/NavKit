# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Focused generation-time evidence-provenance tests."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "python"))
sys.path.insert(0, str(REPOSITORY_ROOT / "tools"))

import run_monte_carlo as monte_carlo_runner  # noqa: E402
import run_regression as regression_runner  # noqa: E402
from internal.evidence_provenance import git_provenance  # noqa: E402
from navkit_analysis.analysis_performance import canonical_json_digest, file_digest  # noqa: E402
from run_sim import default_swil_executable  # noqa: E402


class EvidenceProvenanceTests(unittest.TestCase):
    """Verify source and executable fingerprints used to reject stale evidence."""

    def test_git_dirty_digest_includes_tracked_and_untracked_content(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_evidence_git_") as temp:
            root = Path(temp)
            subprocess.run(["git", "init", "--quiet", str(root)], check=True)
            subprocess.run(
                ["git", "-C", str(root), "config", "user.email", "test@navkit.local"],
                check=True,
            )
            subprocess.run(
                ["git", "-C", str(root), "config", "user.name", "NavKit Test"],
                check=True,
            )
            tracked = root / "tracked.txt"
            tracked.write_text("initial\n", encoding="utf-8")
            subprocess.run(["git", "-C", str(root), "add", "tracked.txt"], check=True)
            subprocess.run(
                ["git", "-C", str(root), "commit", "--quiet", "-m", "initial"],
                check=True,
            )

            clean = git_provenance(root)
            self.assertFalse(clean["dirty"])
            self.assertIsNone(clean["dirty_tree_sha256"])

            tracked.write_text("changed\n", encoding="utf-8")
            untracked = root / "untracked.txt"
            untracked.write_text("first\n", encoding="utf-8")
            first = git_provenance(root)
            self.assertTrue(first["dirty"])

            untracked.write_text("second\n", encoding="utf-8")
            second = git_provenance(root)
            self.assertNotEqual(first["dirty_tree_sha256"], second["dirty_tree_sha256"])

    def test_campaign_provenance_fingerprints_selected_build_artifacts(self) -> None:
        with tempfile.TemporaryDirectory(prefix="navkit_evidence_build_") as temp:
            root = Path(temp).resolve()
            build_dir = root / "build"
            build_dir.mkdir(parents=True)
            navkit_config = "apps/navkit_swil/variants/Test.hpp"
            build_manifest = {
                "schema": "navkit.build_manifest.v1",
                "build_type": "Release",
                "generator": "Ninja",
                "navkit_config": navkit_config,
            }
            build_manifest_path = build_dir / "navkit_build_manifest.json"
            build_manifest_path.write_text(
                json.dumps(build_manifest, indent=2) + "\n", encoding="utf-8"
            )
            executable = default_swil_executable(build_dir, "Release")
            executable.parent.mkdir(parents=True, exist_ok=True)
            executable.write_bytes(b"navkit SWIL artifact")
            config = monte_carlo_runner.CampaignConfig(
                campaign_path=root / "campaign.json",
                campaign_name="test_campaign",
                nominal_config=root / "nominal.json",
                run_count=1,
                start_index=0,
                master_seed=1,
                seed_policy="derive_all",
                build_type="Release",
                parallel_jobs=1,
                max_plot_points=None,
                analysis_renderer="plotly",
                plot_start_time_s=None,
                plot_end_time_s=None,
                output_root=root / "output",
                plot_individual_runs=False,
                package_analysis=False,
                bundle_mode="analysis",
                bundle_compression="gzip",
                aggregate_plots=(),
                consistency_dashboards=False,
                consistency_heatmap_modes=None,
                analysis_parallel_jobs=1,
                navkit_config=navkit_config,
                generator="Ninja",
                build_dir=build_dir,
            )
            git = {
                "revision": "abc123",
                "dirty": True,
                "dirty_tree_sha256": "def456",
            }
            with mock.patch.object(monte_carlo_runner, "git_provenance", return_value=git):
                provenance = monte_carlo_runner._execution_provenance(root, config)
            with mock.patch.object(regression_runner, "git_provenance", return_value=git):
                regression_provenance = regression_runner._artifact_provenance(
                    root,
                    build_dir,
                    "Release",
                    build_manifest_path,
                    build_manifest,
                )

            self.assertEqual(provenance["git"], git)
            build = provenance["build"]
            self.assertIsInstance(build, dict)
            assert isinstance(build, dict)
            manifest = build["manifest"]
            application = build["application_executable"]
            self.assertEqual(manifest["sha256"], file_digest(build_manifest_path))
            self.assertEqual(manifest["canonical_sha256"], canonical_json_digest(build_manifest))
            self.assertEqual(application["sha256"], file_digest(executable))
            self.assertEqual(application["size_bytes"], executable.stat().st_size)
            self.assertEqual(regression_provenance["git"], provenance["git"])
            self.assertEqual(regression_provenance["build"], provenance["build"])
            self.assertIn("tooling", provenance)

            monte_carlo_runner.write_campaign_config(config, [], {}, provenance)
            monte_carlo_runner.write_campaign_manifest(config, [], None, provenance)
            campaign_dir = config.output_root / config.campaign_name
            effective = json.loads(
                (campaign_dir / "campaign_config.effective.json").read_text(encoding="utf-8")
            )
            manifest_document = json.loads(
                (campaign_dir / "campaign_manifest.json").read_text(encoding="utf-8")
            )
            self.assertEqual(effective["provenance"]["git"], git)
            self.assertEqual(effective["provenance"]["build"], provenance["build"])
            self.assertEqual(manifest_document["provenance"], provenance)

            build_manifest["generator"] = "Unix Makefiles"
            build_manifest_path.write_text(
                json.dumps(build_manifest, indent=2) + "\n", encoding="utf-8"
            )
            with mock.patch.object(monte_carlo_runner, "git_provenance", return_value=git):
                with self.assertRaisesRegex(ValueError, "uses generator"):
                    monte_carlo_runner._execution_provenance(root, config)


if __name__ == "__main__":
    unittest.main()
