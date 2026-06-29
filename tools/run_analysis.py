# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from __future__ import annotations

import sys
from pathlib import Path


def repo_root() -> Path:
    script_path = Path(__file__).resolve()

    if script_path.parent.name == "tools":
        return script_path.parent.parent

    return Path.cwd().resolve()


def main(argv: list[str] | None = None) -> int:
    root = repo_root()
    python_root = root / "python"

    if str(python_root) not in sys.path:
        sys.path.insert(0, str(python_root))

    from navkit_analysis.plots import main as plots_main

    return plots_main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
