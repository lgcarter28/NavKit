from __future__ import annotations

import subprocess
from pathlib import Path


def main() -> int:
    files = [str(p) for p in Path(".").rglob("*.hpp")] + [str(p) for p in Path(".").rglob("*.cpp")]
    if not files:
        return 0
    return subprocess.call(["clang-format", "-i", *files])


if __name__ == "__main__":
    raise SystemExit(main())
