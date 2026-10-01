"""Validate all cases in data_processed/tdrp_tw_realistic_v1.

Checks per case:
  - required files present
  - matrix dimensions match NUM_NODES
  - zero diagonals, finite, nonnegative values
  - T_max >= every customer close time (no horizon penalty artifact)
  - provenance fields present (parameter_source, T_max_source, parameter_scenario)
  - capacities / endurance fields present
"""
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

REQUIRED_FILES = [
    "nodes.csv", "customers.csv", "arcs.csv",
    "distance_matrix.csv", "truck_time_matrix.csv",
    "drone_time_matrix.csv", "robot_distance_matrix.csv",
    "robot_time_matrix.csv", "parameters.json", "instance_summary.json",
]
MATRIX_FILES = [
    "distance_matrix.csv", "truck_time_matrix.csv", "drone_time_matrix.csv",
    "robot_distance_matrix.csv", "robot_time_matrix.csv",
]
REQUIRED_PARAM_KEYS = [
    "NUM_CUSTOMERS", "NUM_NODES", "NUM_TRUCKS",
    "Q_t", "Q_d", "Q_r", "E_d", "E_r",
    "truck_speed", "drone_speed", "robot_speed",
    "T_max", "T_max_source", "parameter_scenario", "parameter_source",
]


def read_matrix(path: Path) -> list[list[float]]:
    with path.open(encoding="utf-8") as handle:
        return [[float(v) for v in row] for row in csv.reader(handle) if row]


def validate_case(case_dir: Path) -> list[str]:
    errors: list[str] = []
    for fname in REQUIRED_FILES:
        if not (case_dir / fname).exists():
            errors.append(f"missing file {fname}")
    if errors:
        return errors
    params = json.loads((case_dir / "parameters.json").read_text(encoding="utf-8"))
    for key in REQUIRED_PARAM_KEYS:
        if key not in params:
            errors.append(f"parameters.json missing key {key}")
    n_nodes = params.get("NUM_NODES")
    n_cust = params.get("NUM_CUSTOMERS")
    for mfile in MATRIX_FILES:
        mat = read_matrix(case_dir / mfile)
        if len(mat) != n_nodes or any(len(row) != n_nodes for row in mat):
            errors.append(f"{mfile}: expected {n_nodes}x{n_nodes}, got {len(mat)}x{len(mat[0]) if mat else 0}")
            continue
        for i in range(n_nodes):
            if mat[i][i] != 0.0:
                errors.append(f"{mfile}: nonzero diagonal at ({i},{i})")
                break
        for i, row in enumerate(mat):
            for j, v in enumerate(row):
                if not math.isfinite(v) or v < 0:
                    errors.append(f"{mfile}: bad value {v} at ({i},{j})")
                    break
            else:
                continue
            break
    # Time-window / horizon consistency.
    t_max = params.get("T_max")
    with (case_dir / "customers.csv").open(encoding="utf-8") as handle:
        customers = list(csv.DictReader(handle))
    if len(customers) != n_cust:
        errors.append(f"customers.csv has {len(customers)} rows, NUM_CUSTOMERS={n_cust}")
    for c in customers:
        if float(c["close_time"]) > t_max + 1e-9:
            errors.append(f"customer {c['customer_id']}: close_time {c['close_time']} > T_max {t_max}")
            break
        if float(c["open_time"]) < 0 or float(c["demand"]) < 0:
            errors.append(f"customer {c['customer_id']}: negative open_time or demand")
            break
    return errors


def main() -> int:
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("data_processed/tdrp_tw_realistic_v1")
    case_dirs = sorted(d for d in root.iterdir() if d.is_dir())
    print(f"Validating {len(case_dirs)} cases in {root}")
    bad: dict[str, list[str]] = {}
    for case_dir in case_dirs:
        errs = validate_case(case_dir)
        if errs:
            bad[case_dir.name] = errs
    if bad:
        print(f"\n{len(bad)} cases FAILED:")
        for name, errs in bad.items():
            for e in errs:
                print(f"  {name}: {e}")
        return 1
    print(f"\nAll {len(case_dirs)} cases passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
