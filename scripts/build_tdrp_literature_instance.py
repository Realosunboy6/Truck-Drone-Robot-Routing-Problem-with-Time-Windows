from __future__ import annotations

import argparse
import csv
import json
import math
import re
from datetime import datetime, timezone
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]

HEADER_FIELDS = [
    ("NUM_CUSTOMERS", int),
    ("NUM_TCS", int),
    ("NUM_TRUCKS", int),
    ("NUM_DRONES", int),
    ("DRONES_CARRIED_AT_DEPOT", int),
    ("MAX_DRONES_PER_TRUCK", int),
    ("Q_t", float),
    ("Q_d", float),
    ("truck_speed", float),
    ("drone_speed", float),
    ("T_max", float),
    ("E_d", float),
    ("truck_variable_cost", float),
    ("drone_departure_cost", float),
    ("waiting_penalty_weight", float),
    ("big_M", float),
]

LITERATURE_FIELDS = {
    "truck_speed": 45.0,
    "drone_speed": 75.0,
    "robot_speed": 25.0,
    "C_w": 0.0,
    "C_veh": 2.9,
    "C_w_drone": 0.0,
    "C_drone": 0.08,
    "C_w_r": 0.0,
    "C_rob": 0.06,
    "truck_fixed_cost": 30.0,
    "drone_fixed_cost": 10.0,
    "robot_fixed_cost": 8.0,
    "Q_d": 25.0,
    "Q_r": 20.0,
    "E_d": 20.0,
    "E_r": 15.0,
    "lambda_E_d": 1000.0,
    "lambda_E_r": 1000.0,
    "lambda_Q": 1000.0,
    "lambda_T": 1000.0,
    "lambda_W": 1000.0,
    "parameter_scenario": "literature_vrpdr_table3",
    "parameter_source": "Collaborative Last-Mile Delivery: A Multi-Platform Vehicle Routing Problem With En-route Charging, Table 3",
    "parameter_source_arxiv": "2505.23584v1",
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def numeric_row(line: str) -> list[float]:
    return [float(part) for part in re.split(r"\s+", line.strip()) if part]


def write_csv(path: Path, rows: list[dict], fieldnames: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_matrix(path: Path, matrix: list[list[float]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        for row in matrix:
            writer.writerow([f"{value:.10g}" for value in row])


def canonical_to_raw(node_id: int, end_depot: int) -> int:
    return 0 if node_id == end_depot else node_id


def matrix_value(raw_matrix: list[list[float]], i: int, j: int, end_depot: int) -> float:
    return raw_matrix[canonical_to_raw(i, end_depot)][canonical_to_raw(j, end_depot)]


def stats(values: list[float]) -> dict[str, float]:
    return {
        "min": min(values) if values else 0.0,
        "max": max(values) if values else 0.0,
        "mean": sum(values) / len(values) if values else 0.0,
    }


def parse_raw_instance(raw_path: Path) -> tuple[dict, list[float], list[float], list[float], list[float], list[list[float]]]:
    rows = [numeric_row(line) for line in raw_path.read_text(encoding="utf-8", errors="ignore").splitlines() if line.strip()]
    if len(rows) < 21:
        raise ValueError(f"{raw_path} has too few non-empty rows for the documented TDRP-TW format.")
    if any(len(row) != 1 for row in rows[:16]):
        raise ValueError(f"{raw_path} does not have the expected 16 one-value header rows.")

    params: dict = {
        "source_dataset": "TDRP-TW",
        "source_instance": raw_path.name,
        "selected_raw_file": str(raw_path.relative_to(PACKAGE_ROOT)) if raw_path.is_relative_to(PACKAGE_ROOT) else raw_path.name,
        "parsed_at": utc_now(),
        "format": "small_and_medium_single_file",
    }
    for (field, caster), row in zip(HEADER_FIELDS, rows[:16]):
        value = row[0]
        params[field] = caster(value)
    # NUM_TCS is a raw TDRP-TW type-count field. The strict model lets the MILP
    # choose truck/drone/robot assignment by constraints, so this metadata is not
    # carried into processed parameters.
    params.pop("NUM_TCS", None)

    n = int(params["NUM_CUSTOMERS"])
    expected = n + 1
    demand, service, open_time, close_time = rows[16:20]
    raw_matrix = rows[20:20 + expected]
    if any(len(vector) != expected for vector in (demand, service, open_time, close_time)):
        raise ValueError(f"{raw_path} has a demand/service/window vector with the wrong length.")
    if len(raw_matrix) != expected or any(len(row) != expected for row in raw_matrix):
        raise ValueError(f"{raw_path} distance matrix must be {expected} x {expected}.")
    return params, demand, service, open_time, close_time, raw_matrix


def build_instance(raw_path: Path, out_dir: Path) -> None:
    params, demand, service, open_time, close_time, raw_matrix = parse_raw_instance(raw_path)
    params.update(LITERATURE_FIELDS)
    params["NUM_ROBOTS"] = int(params["NUM_CUSTOMERS"]) + int(params["NUM_TRUCKS"])
    params["ROBOTS_CARRIED_AT_DEPOT"] = 1
    params["MAX_ROBOTS_PER_TRUCK"] = 2
    params["robot_fleet_assumption"] = (
        "Robot fleet parameters are stored separately. "
        "Current robot values are experimental placeholders because robot-specific benchmark data is unavailable."
    )

    n = int(params["NUM_CUSTOMERS"])
    end_depot = n + 1
    node_ids = list(range(n + 2))
    out_dir.mkdir(parents=True, exist_ok=True)

    nodes = [{"node_id": 0, "node_type": "start_depot", "x": "", "y": ""}]
    nodes.extend({"node_id": customer_id, "node_type": "customer", "x": "", "y": ""} for customer_id in range(1, n + 1))
    nodes.append({"node_id": end_depot, "node_type": "end_depot", "x": "", "y": ""})

    customers = []
    for customer_id in range(1, n + 1):
        customer_demand = float(demand[customer_id])
        customers.append(
            {
                "customer_id": customer_id,
                "demand": customer_demand,
                "open_time": float(open_time[customer_id]),
                "close_time": float(close_time[customer_id]),
                "service_time": float(service[customer_id]),
            }
        )

    size = n + 2
    distance_matrix = [[0.0 for _ in range(size)] for _ in range(size)]
    truck_time_matrix = [[0.0 for _ in range(size)] for _ in range(size)]
    drone_time_matrix = [[0.0 for _ in range(size)] for _ in range(size)]
    robot_distance_matrix = [[0.0 for _ in range(size)] for _ in range(size)]
    robot_time_matrix = [[0.0 for _ in range(size)] for _ in range(size)]

    arcs = []
    for i in node_ids:
        for j in node_ids:
            if i == j:
                continue
            distance = matrix_value(raw_matrix, i, j, end_depot)
            distance_matrix[i][j] = distance
            truck_time_matrix[i][j] = distance / float(params["truck_speed"])
            drone_time_matrix[i][j] = distance / float(params["drone_speed"])
            robot_distance_matrix[i][j] = distance
            robot_time_matrix[i][j] = distance / float(params["robot_speed"])
            if j != 0 and i != end_depot:
                arcs.append(
                    {
                        "i": i,
                        "j": j,
                        "distance": distance,
                        "truck_time": truck_time_matrix[i][j],
                        "drone_time": drone_time_matrix[i][j],
                        "robot_distance": distance,
                        "robot_time": robot_time_matrix[i][j],
                    }
                )

    params["NUM_NODES"] = size
    params["NUM_ARCS"] = len(arcs)
    params["assumptions"] = [
        "Canonical end depot n+1 duplicates raw depot 0.",
        "Robot layer uses the same benchmark distance matrix because TDRP-TW coordinates are unavailable.",
        "Literature platform speeds, costs, capacities, and endurance values are applied for this experiment.",
    ]
    params["warnings"] = [
        "This is a truck-drone benchmark converted to a truck-drone-robot experiment.",
        "Robot movement is a benchmark-distance surrogate, not a sidewalk-network shortest path.",
    ]

    write_csv(out_dir / "nodes.csv", nodes, ["node_id", "node_type", "x", "y"])
    write_csv(
        out_dir / "customers.csv",
        customers,
        ["customer_id", "demand", "open_time", "close_time", "service_time"],
    )
    write_csv(out_dir / "arcs.csv", arcs, ["i", "j", "distance", "truck_time", "drone_time", "robot_distance", "robot_time"])
    write_matrix(out_dir / "distance_matrix.csv", distance_matrix)
    write_matrix(out_dir / "truck_time_matrix.csv", truck_time_matrix)
    write_matrix(out_dir / "drone_time_matrix.csv", drone_time_matrix)
    write_matrix(out_dir / "robot_distance_matrix.csv", robot_distance_matrix)
    write_matrix(out_dir / "robot_time_matrix.csv", robot_time_matrix)
    (out_dir / "parameters.json").write_text(json.dumps(params, indent=2), encoding="utf-8")

    demand_values = [float(row["demand"]) for row in customers]
    open_values = [float(row["open_time"]) for row in customers]
    close_values = [float(row["close_time"]) for row in customers]
    arc_distances = [float(row["distance"]) for row in arcs]
    summary = {
        "source_dataset": "TDRP-TW",
        "selected_raw_file": params["selected_raw_file"],
        "number_of_customers": n,
        "number_of_nodes": size,
        "number_of_arcs": len(arcs),
        "demand_min": min(demand_values),
        "demand_max": max(demand_values),
        "demand_total": sum(demand_values),
        "time_window_open_min": min(open_values),
        "time_window_close_max": max(close_values),
        "distance_min": stats(arc_distances)["min"],
        "distance_max": stats(arc_distances)["max"],
        "distance_mean": stats(arc_distances)["mean"],
        "parameter_scenario": params["parameter_scenario"],
        "assumptions": params["assumptions"],
        "warnings": params["warnings"],
    }
    (out_dir / "instance_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Build one canonical literature-parameter instance from a raw TDRP-TW small/medium file.")
    parser.add_argument("--raw-file", required=True, type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    args = parser.parse_args()
    build_instance(args.raw_file, args.out_dir)
    print(f"Built {args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
