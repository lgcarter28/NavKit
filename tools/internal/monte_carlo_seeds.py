# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

"""Deterministic Monte Carlo seed derivation shared by runners and validators."""

from __future__ import annotations

import hashlib


UINT32_MASK = (1 << 32) - 1


def derive_seed(master_seed: int, run_index: int, json_pointer: str) -> int:
    """Derive one nonzero 32-bit component seed from campaign identity."""
    message = f"{master_seed}:{run_index}:{json_pointer}".encode("utf-8")
    digest = hashlib.blake2b(message, digest_size=8).digest()
    seed = int.from_bytes(digest, byteorder="little", signed=False) & UINT32_MASK
    return 1 if seed == 0 else seed
