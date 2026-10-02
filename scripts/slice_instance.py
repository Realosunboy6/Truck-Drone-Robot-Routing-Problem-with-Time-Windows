#!/usr/bin/env python3
"""Slice the first K customers out of a large real-world instance into a
small pilot instance readable by the MILP model.

Usage:
    python3 scripts/slice_instance.py <src_dir> <dst_dir> <K> [--trucks T]

Customers are i.i.d. samples, so customers 1..K are representative. The depot
(centroid of all 300) is kept as-is. Matrices are sliced to node indices
[0..K] + old end-depot; arcs are filtered and remapped.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def read_csv_rows(p: Path):
    with open(p, newline="") as f:
        return list(csv.DictReader(f))


def read_matrix(p: Path):
    with open(p, newline="") as f:
        return [[float(x) for x in row] for row in csv.reader(f)]


def write_matrix(p: Path, m):
    with open(p, "w", newline="") as f:
        w = csv.writer(f)
        w.writerows([f"{v:.6f}" for v in row] for row in m)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("src_dir", type=Path)
    ap.add_argument("dst_dir", type=Path)
    ap.add_argument("k", type=int)
    ap.add_argument("--trucks", type=int, default=2)
    args = ap.parse_args()

    src, dst, k = args.src_dir, args.dst_dir, args.k
    params = json.loads((src / "parameters.json").read_text())
    n_src = params["NUM_CUSTOMERS"]
    old_end = n_src + 1
    assert 1 <= k < n_src

    nodes = read_csv_rows(src / "nodes.csv")
    customers = read_csv_rows(src / "customers.csv")
    arcs = read_csv_rows(src / "arcs.csv")

    new_end = k + 1
    keep_old = list(range(0, k + 1)) + [old_end]
    remap = {o: (o if o <= k else new_end) for o in keep_old}

    dst.mkdir(parents=True, exist_ok=True)

    # nodes.csv
    with open(dst / "nodes.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["node_id", "node_type", "x", "y"])
        w.writeheader()
        for row in nodes:
            oid = int(row["node_id"])
            if oid in remap:
                ntype = row["node_type"]
                if oid == old_end:
                    ntype = "end_depot"
                w.writerow({"node_id": remap[oid], "node_type": ntype,
                            "x": row["x"], "y": row["y"]})

    # customers.csv (first k)
    with open(dst / "customers.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["customer_id", "demand", "open_time",
                                          "close_time", "service_time"])
        w.writeheader()
        for row in customers[:k]:
            assert int(row["customer_id"]) <= k
            w.writerow(row)

    # matrices
    for name in ["distance_matrix", "truck_time_matrix", "drone_time_matrix",
                 "robot_distance_matrix", "robot_time_matrix"]:
        m = read_matrix(src / f"{name}.csv")
        write_matrix(dst / f"{name}.csv",
                     [[m[i][j] for j in keep_old] for i in keep_old])

    # arcs.csv
    with open(dst / "arcs.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["i", "j", "distance", "truck_time",
                                          "drone_time", "robot_distance", "robot_time"])
        w.writeheader()
        narcs = 0
        for row in arcs:
            oi, oj = int(row["i"]), int(row["j"])
            if oi not in remap or oj not in remap:
                continue
            i, j = remap[oi], remap[oj]
            if i == j or j == 0 or i == new_end:
                continue
            w.writerow({"i": i, "j": j, "distance": row["distance"],
                        "truck_time": row["truck_time"], "drone_time": row["drone_time"],
                        "robot_distance": row["robot_distance"],
                        "robot_time": row["robot_time"]})
            narcs += 1

    # parameters.json
    params = dict(params)
    params.update({
        "NUM_CUSTOMERS": k,
        "NUM_TRUCKS": args.trucks,
        "NUM_DRONES": 2 * args.trucks,
        "NUM_ROBOTS": 2 * args.trucks,
        "NUM_NODES": k + 2,
        "NUM_ARCS": narcs,
        "source_dataset": params.get("source_dataset", "") + f" (first-{k} slice)",
        "generator": "scripts/slice_instance.py",
    })
    (dst / "parameters.json").write_text(json.dumps(params, indent=2))

    # instance_summary.json
    summ = json.loads((src / "instance_summary.json").read_text())
    summ = dict(summ)
    summ.update({
        "number_of_customers": k,
        "number_of_nodes": k + 2,
        "number_of_arcs": narcs,
        "demand_total": round(sum(float(r["demand"]) for r in customers[:k]), 2),
    })
    (dst / "instance_summary.json").write_text(json.dumps(summ, indent=2))

    print(f"sliced {k} customers -> {dst} ({narcs} arcs)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
