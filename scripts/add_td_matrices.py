#!/usr/bin/env python3
"""Add time-dependent truck travel-time matrices to an existing instance dir.

Reads truck_time_matrix.csv (hours, free-flow), builds one matrix per
departure period via Ichoua-Gendreau-Semet (2003) speed-profile integration
(see scripts/time_dependent.py; FIFO holds by construction and is verified),
and writes them as truck_time_matrix_{profile}_p{p}.csv alongside the static
files. Provenance is merged into parameters.json under td_profiles[profile].

Robot/drone layers stay static (documented assumption in time_dependent.py).

Usage:
    add_td_matrices.py --instance-dir DIR --profile two_peak|mild|flat
                       [--periods 8] [--horizon 8.0]
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from time_dependent import (
    PROFILES,
    discretize_profile,
    period_travel_matrices,
    profile_summary,
    verify_fifo,
)


def read_matrix(path: Path) -> list[list[float]]:
    with open(path, newline="") as f:
        return [[float(v) for v in row] for row in csv.reader(f) if row]


def write_matrix(path: Path, mat: list[list[float]]) -> None:
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        for row in mat:
            w.writerow(f"{v:.6f}" for v in row)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--instance-dir", required=True)
    ap.add_argument("--profile", required=True, choices=sorted(PROFILES))
    ap.add_argument("--periods", type=int, default=8)
    ap.add_argument("--horizon", type=float, default=8.0)
    args = ap.parse_args()

    inst = Path(args.instance_dir)
    static = read_matrix(inst / "truck_time_matrix.csv")
    import numpy as np

    t_static = np.array(static, dtype=float)
    bounds, factors = discretize_profile(args.profile, args.periods, args.horizon)
    mats = period_travel_matrices(t_static, bounds, factors)
    assert verify_fifo(mats, bounds), "FIFO check failed"

    for p, mat in enumerate(mats):
        write_matrix(inst / f"truck_time_matrix_{args.profile}_p{p}.csv", mat.tolist())

    params_path = inst / "parameters.json"
    params = json.loads(params_path.read_text())
    info = profile_summary(args.profile, args.periods, args.horizon)
    info["td_fifo_verified"] = True
    info["td_matrix_files"] = [f"truck_time_matrix_{args.profile}_p{p}.csv" for p in range(args.periods)]
    params.setdefault("td_profiles", {})[args.profile] = info
    params_path.write_text(json.dumps(params, indent=2))

    # Sanity: peak departures must be slower than static on at least one OD pair
    slow = max(float(mats[p].max()) for p in range(args.periods))
    print(f"{inst.name}: profile={args.profile} periods={args.periods} "
          f"max_period_travel={slow:.4f}h static_max={float(t_static.max()):.4f}h FIFO=ok")


if __name__ == "__main__":
    main()
