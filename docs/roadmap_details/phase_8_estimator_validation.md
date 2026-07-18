# Phase 8 - Estimator Validation and Consistency

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

Estimator validation follows Monte Carlo and trajectory expansion so the validation suite can operate on multiple repeatable scenarios rather than a single demo run.

## Pass 8.1: baseline regression metrics

- [ ] Establish named numerical baseline scenarios and metric thresholds.
- [ ] Add a regression command/check for the default ECEF INS/GNSS simulation and analysis pipeline.
- [ ] Add skip-first-N/time-window options for transient exclusion.

## Pass 8.2: innovation and covariance consistency

- [ ] Add innovation summary statistics: mean, variance, standard deviation, RMS, and sample count.
- [ ] Add NIS summary statistics: mean, expected mean, 95%/99% exceedance rates, and mean p-value.
- [ ] Overlay expected Gaussian distributions on innovation histograms.
- [ ] Add whitened-innovation time histories and autocorrelation checks.
- [ ] Add NEES for scenarios with known truth.
- [ ] Add confidence-bound exceedance statistics.
- [ ] Define pass/fail thresholds carefully; avoid treating a single stochastic run as proof of consistency.

## Pass 8.3: validation reports

- [ ] Produce machine-readable consistency summaries.
- [ ] Produce human-readable validation reports.

## Phase 8 completed foundation

- [x] Basic plots, innovation histories, NIS/p-value plots, histograms, ECEF/NED covariance/error plots, and dashboard plots exist.
- [x] Current scenario tooling can run simulations and generate analysis artifacts from one command.
