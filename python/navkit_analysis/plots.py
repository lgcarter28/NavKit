from __future__ import annotations

import argparse
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt


def plot_run(run_dir: Path) -> None:
    nav = pd.read_csv(run_dir / "nav.csv")
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(nav["time_s"], nav["err_p_e_x_m"], label="err p_e x")
    ax.plot(nav["time_s"], nav["err_p_e_y_m"], label="err p_e y")
    ax.plot(nav["time_s"], nav["err_p_e_z_m"], label="err p_e z")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("position error [m]")
    ax.grid(True)
    ax.legend()
    out = run_dir / "position_error.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(f"Wrote {out}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    plot_run(args.run_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
