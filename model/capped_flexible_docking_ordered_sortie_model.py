from __future__ import annotations

import json
import math
import os
import time
from itertools import permutations
from pathlib import Path

import cplex  # noqa: F401
import numpy as np
import pandas as pd
from docplex.mp.model import Model


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = Path(os.environ.get("DRT_DATA_DIR", PACKAGE_ROOT / "data_processed" / "tdrp_tw_literature_params" / "6-25"))
RESULTS_DIR = Path(os.environ.get("DRT_RESULTS_DIR", PACKAGE_ROOT / "results"))
INSTANCE_TAG = os.environ.get("DRT_INSTANCE_TAG", "tdrp_6_25")
RUN_TAG = os.environ.get("DRT_RUN_TAG", "capped_flexible_docking")
EXCEL_PATH = RESULTS_DIR / f"{INSTANCE_TAG}_ordered_sorties_{RUN_TAG}_solution.xlsx"
LP_BASENAME = f"{INSTANCE_TAG}_ordered_sorties_{RUN_TAG}_model"
LP_PATH = RESULTS_DIR / f"{LP_BASENAME}.lp"
EXPORT_LP = os.environ.get("DRT_EXPORT_LP", "1").strip().lower() in {"1", "true", "yes", "y"}
NOTES_PATH = RESULTS_DIR / f"{INSTANCE_TAG}_ordered_sorties_{RUN_TAG}_model_notes.md"
INFEASIBILITY_NOTES_PATH = RESULTS_DIR / f"{INSTANCE_TAG}_ordered_sorties_{RUN_TAG}_infeasibility_notes.md"
MAX_CUSTOMERS_PER_SORTIE = int(os.environ.get("DRT_MAX_CUSTOMERS_PER_SORTIE", "4"))
TOP_SORTIES_PER_TRUCK_LEG = int(os.environ.get("DRT_TOP_SORTIES_PER_TRUCK_LEG", "25"))
TIME_LIMIT_SECONDS = float(os.environ.get("DRT_TIME_LIMIT_SECONDS", "1800"))
# Truck-only baseline mode: DRT_TRUCK_ONLY=1 fixes all drone and robot
# sortie variables to zero so the same model file can produce a truck-only baseline.
TRUCK_ONLY = os.environ.get("DRT_TRUCK_ONLY", "0").strip() == "1"
USE_TRUCK_WARM_START = os.environ.get("DRT_USE_TRUCK_WARM_START", "0").strip().lower() in {"1", "true", "yes", "y"}


def read_inputs() -> dict:
    nodes = pd.read_csv(DATA_DIR / "nodes.csv")
    customers = pd.read_csv(DATA_DIR / "customers.csv")
    arcs = pd.read_csv(DATA_DIR / "arcs.csv")
    return {
        "nodes": nodes,
        "customers": customers,
        "arcs": arcs,
        "distance_matrix": pd.read_csv(DATA_DIR / "distance_matrix.csv", header=None).to_numpy(dtype=float),
        "truck_time_matrix": pd.read_csv(DATA_DIR / "truck_time_matrix.csv", header=None).to_numpy(dtype=float),
        "drone_time_matrix": pd.read_csv(DATA_DIR / "drone_time_matrix.csv", header=None).to_numpy(dtype=float),
        "robot_distance_matrix": pd.read_csv(DATA_DIR / "robot_distance_matrix.csv", header=None).to_numpy(dtype=float),
        "robot_time_matrix": pd.read_csv(DATA_DIR / "robot_time_matrix.csv", header=None).to_numpy(dtype=float),
        "parameters": json.loads((DATA_DIR / "parameters.json").read_text(encoding="utf-8-sig")),
        "instance_summary": json.loads((DATA_DIR / "instance_summary.json").read_text(encoding="utf-8-sig")),
    }


def validate_inputs(data: dict) -> None:
    nodes = data["nodes"]
    customers = data["customers"]
    arcs = data["arcs"]
    required_node_cols = {"node_id", "node_type", "x", "y"}
    required_customer_cols = {
        "customer_id", "demand", "open_time", "close_time", "service_time",
    }
    required_arc_cols = {"i", "j", "distance", "truck_time", "drone_time", "robot_distance", "robot_time"}
    errors: list[str] = []
    if not required_node_cols.issubset(nodes.columns):
        errors.append(f"nodes.csv missing {sorted(required_node_cols - set(nodes.columns))}")
    if not required_customer_cols.issubset(customers.columns):
        errors.append(f"customers.csv missing {sorted(required_customer_cols - set(customers.columns))}")
    if not required_arc_cols.issubset(arcs.columns):
        errors.append(f"arcs.csv missing {sorted(required_arc_cols - set(arcs.columns))}")
    n = len(customers)
    node_ids = sorted(nodes["node_id"].astype(int).tolist())
    if node_ids != list(range(n + 2)):
        errors.append(f"Expected nodes 0..{n + 1}, found {node_ids}")
    if sorted(customers["customer_id"].astype(int).tolist()) != list(range(1, n + 1)):
        errors.append("Customers must be exactly 1..n")
    for name in ["distance_matrix", "truck_time_matrix", "drone_time_matrix", "robot_distance_matrix", "robot_time_matrix"]:
        matrix = data[name]
        if matrix.shape != (n + 2, n + 2):
            errors.append(f"{name} has shape {matrix.shape}, expected {(n + 2, n + 2)}")
        if not np.isfinite(matrix).all():
            errors.append(f"{name} has non-finite values")
        if (matrix < -1e-9).any():
            errors.append(f"{name} has negative values")
    if errors:
        raise ValueError("Input validation failed:\n" + "\n".join(f"- {e}" for e in errors))


def flatten_json(prefix: str, value, rows: list[dict]) -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            flatten_json(f"{prefix}.{key}" if prefix else str(key), child, rows)
    elif isinstance(value, list):
        rows.append({"parameter": prefix, "value": json.dumps(value)})
    else:
        rows.append({"parameter": prefix, "value": value})


def value_or_zero(solution, obj) -> float:
    if solution is None:
        return 0.0
    try:
        return float(solution.get_value(obj))
    except Exception:
        return 0.0


def selected_rows(solution, var_dict: dict, columns: list[str], threshold: float = 0.5) -> pd.DataFrame:
    rows = []
    if solution is None:
        return pd.DataFrame(columns=columns + ["value"])
    for key, var in var_dict.items():
        val = value_or_zero(solution, var)
        if val > threshold:
            row = dict(zip(columns, key if isinstance(key, tuple) else (key,)))
            row["value"] = val
            rows.append(row)
    return pd.DataFrame(rows, columns=columns + ["value"])


def combine_platform_sortie_selections(
    solution,
    launch_vars: dict,
    recovery_vars: dict,
    platform_col: str,
    sortie_df: pd.DataFrame,
) -> pd.DataFrame:
    launch_df = selected_rows(solution, launch_vars, ["launch_truck", platform_col, "sortie_id"])
    recovery_df = selected_rows(solution, recovery_vars, ["recovery_truck", platform_col, "sortie_id"])
    key_cols = [platform_col, "sortie_id"]
    output_cols = [
        "launch_truck",
        "recovery_truck",
        platform_col,
        "sortie_id",
        "value_launch",
        "value_recovery",
        "truck",
    ]

    if launch_df.empty and recovery_df.empty:
        return pd.DataFrame(columns=output_cols + [col for col in sortie_df.columns if col not in output_cols])

    selected = pd.merge(
        launch_df,
        recovery_df,
        on=key_cols,
        how="outer",
        suffixes=("_launch", "_recovery"),
    )
    selected = selected.merge(sortie_df, on="sortie_id", how="left")
    selected["truck"] = selected.apply(
        lambda row: f"{int(row.launch_truck)}->{int(row.recovery_truck)}"
        if pd.notna(row.launch_truck) and pd.notna(row.recovery_truck)
        else "",
        axis=1,
    )
    leading_cols = [col for col in output_cols if col in selected.columns]
    trailing_cols = [col for col in selected.columns if col not in leading_cols]
    return selected[leading_cols + trailing_cols]


def route_edges(start: int, sequence: tuple[int, ...], end: int) -> list[tuple[int, int]]:
    path = (start,) + sequence + (end,)
    return list(zip(path[:-1], path[1:]))


def route_label(start: int, sequence: tuple[int, ...], end: int) -> str:
    return " -> ".join(map(str, (start,) + sequence + (end,)))


def add_truck_only_mip_start(
    mdl: Model,
    T: list[int],
    C: list[int],
    V: list[int],
    A_set: set[tuple[int, int]],
    start_depot: int,
    end_depot: int,
    demand: dict[int, float],
    truck_capacity: float,
    truck_time: np.ndarray,
    open_time: dict[int, float],
    close_time: dict[int, float],
    service_time: dict[int, float],
    T_max: float,
    x: dict,
    used_truck: dict,
    visit: dict,
    u: dict,
    a_t: dict,
    late_t: dict,
    route_late_t: dict,
    route_late_global,
    y: dict,
    z: dict,
    drone_truck: dict,
    robot_truck: dict,
) -> bool:
    """Give CPLEX a simple real truck-only incumbent.

    The start is intentionally conservative: customers are sorted by time
    window and split across trucks by capacity. Time-window and route-duration
    violations are allowed through the model's existing soft penalty variables.
    """

    ordered_customers = sorted(C, key=lambda i: (open_time[i], close_time[i], i))
    routes: dict[int, list[int]] = {v: [] for v in T}
    loads = {v: 0.0 for v in T}
    truck_index = 0

    for customer in ordered_customers:
        if demand[customer] > truck_capacity + 1e-9:
            return False
        while truck_index < len(T) and loads[T[truck_index]] + demand[customer] > truck_capacity + 1e-9:
            truck_index += 1
        if truck_index >= len(T):
            return False
        v = T[truck_index]
        routes[v].append(customer)
        loads[v] += demand[customer]

    mip_start = mdl.new_solution()

    max_route_late = 0.0

    for v in T:
        route = routes[v]
        if not route:
            continue

        mip_start.add_var_value(used_truck[v], 1)
        path = [start_depot] + route + [end_depot]

        current_time = 0.0
        mip_start.add_var_value(a_t[v, start_depot], 0)
        for order, customer in enumerate(route, start=1):
            mip_start.add_var_value(visit[v, customer], 1)
            mip_start.add_var_value(u[v, customer], order)

        for i, j in zip(path[:-1], path[1:]):
            if (i, j) not in A_set:
                return False
            mip_start.add_var_value(x[v, i, j], 1)
            if i in C:
                current_time += float(service_time[i])
            current_time += float(truck_time[i, j])
            if j in C:
                current_time = max(current_time, float(open_time[j]))
                mip_start.add_var_value(a_t[v, j], current_time)
                customer_late = max(0.0, current_time - float(close_time[j]))
                if customer_late > 1e-9:
                    mip_start.add_var_value(late_t[j], customer_late)
            elif j == end_depot:
                mip_start.add_var_value(a_t[v, end_depot], current_time)

        route_late = max(0.0, current_time - T_max)
        if route_late > 1e-9:
            mip_start.add_var_value(route_late_t[v], route_late)
        max_route_late = max(max_route_late, route_late)

    if max_route_late > 1e-9:
        mip_start.add_var_value(route_late_global, max_route_late)
    mdl.add_mip_start(mip_start, complete_vars=False)
    return True


def generate_ordered_sorties(
    arcs: list[tuple[int, int]],
    customers: list[int],
    demand: dict[int, float],
    capacity: float,
    endurance: float,
    time_matrix: np.ndarray,
    distance_matrix: np.ndarray,
    cost_matrix: dict[tuple[int, int], float],
    max_customers_per_sortie: int = MAX_CUSTOMERS_PER_SORTIE,
    top_sorties_per_truck_leg: int = TOP_SORTIES_PER_TRUCK_LEG,
) -> pd.DataFrame:
    rows: list[dict] = []
    sortie_id = 1
    for launch, recover in arcs:
        leg_rows: list[dict] = []
        candidates = [
            customer
            for customer in customers
            if customer not in (launch, recover)
        ]
        max_length = min(max_customers_per_sortie, len(candidates))
        for length in range(1, max_length + 1):
            for sequence in permutations(candidates, length):
                payload = sum(demand[customer] for customer in sequence)
                if payload > capacity + 1e-9:
                    continue
                edges = route_edges(launch, sequence, recover)
                duration = sum(float(time_matrix[u, v]) for u, v in edges)
                route_distance = sum(float(distance_matrix[u, v]) for u, v in edges)
                if route_distance > endurance + 1e-9:
                    continue
                leg_rows.append(
                    {
                        "i": launch,
                        "k": recover,
                        "sequence": sequence,
                        "sequence_text": ",".join(map(str, sequence)),
                        "route": route_label(launch, sequence, recover),
                        "customer_count": len(sequence),
                        "payload": payload,
                        "travel_distance": route_distance,
                        "duration": duration,
                        "cost": sum(cost_matrix[u, v] for u, v in edges),
                    }
                )
        leg_rows.sort(key=lambda row: (-row["customer_count"], row["cost"], row["duration"], row["sequence_text"]))
        for row in leg_rows[:top_sorties_per_truck_leg]:
            row["sortie_id"] = sortie_id
            rows.append(row)
            sortie_id += 1
    return pd.DataFrame(rows)


def build_truck_routes(selected_truck: pd.DataFrame, T: list[int], start: int, end: int) -> pd.DataFrame:
    rows = []
    for v in T:
        arcs_v = selected_truck[selected_truck["truck"] == v] if not selected_truck.empty else pd.DataFrame()
        if arcs_v.empty:
            rows.append(
                {
                    "truck": v,
                    "route": "unused",
                    "arc_count": 0,
                    "starts_at_depot": False,
                    "ends_at_end_depot": False,
                    "status": "unused",
                }
            )
            continue
        succ = {int(row.i): int(row.j) for row in arcs_v.itertuples(index=False)}
        route = [start]
        current = start
        seen = {start}
        while current in succ:
            nxt = succ[current]
            route.append(nxt)
            if nxt == end:
                break
            if nxt in seen:
                route.append("cycle_detected")
                break
            seen.add(nxt)
            current = nxt
        rows.append(
            {
                "truck": v,
                "route": " -> ".join(map(str, route)),
                "arc_count": len(arcs_v),
                "starts_at_depot": bool(route and route[0] == start),
                "ends_at_end_depot": bool(route and route[-1] == end),
                "status": "used" if route and route[-1] == end else "incomplete",
            }
        )
    return pd.DataFrame(rows)


def build_truck_physical_timing(
    selected_truck: pd.DataFrame,
    T: list[int],
    start: int,
    end: int,
    solution,
    truck_arrival_vars: dict,
    truck_time_matrix: np.ndarray,
    open_time: dict[int, float],
    service_time: dict[int, float],
    recovery_events: dict[tuple[int, int], float],
) -> pd.DataFrame:
    """Recompute the earliest physical truck timing along each selected route
    and compare it with the model arrival variables. Arrival variables are
    only lower-bounded in the MILP, so harmless upward slack is possible at
    nodes without objective pressure; this sheet makes any slack explicit and
    labels the legitimate waiting causes (time windows, platform recovery)."""
    rows: list[dict] = []
    for v in T:
        arcs_v = selected_truck[selected_truck["truck"] == v] if not selected_truck.empty else pd.DataFrame()
        if arcs_v.empty:
            continue
        succ = {int(r.i): int(r.j) for r in arcs_v.itertuples(index=False)}
        current = start
        depart_prev = 0.0
        visited = {start}
        while current in succ:
            nxt = succ[current]
            travel = float(truck_time_matrix[current, nxt])
            arrive = depart_prev + travel
            window_open = float(open_time.get(nxt, 0.0)) if nxt != end else 0.0
            recovery_ready = float(recovery_events.get((v, nxt), 0.0))
            earliest_required = max(arrive, window_open, recovery_ready)
            model_time = value_or_zero(solution, truck_arrival_vars[v, nxt])
            if recovery_ready > max(arrive, window_open):
                note = "waits for platform recovery"
            elif window_open > arrive:
                note = "waits for time window"
            else:
                note = ""
            rows.append(
                {
                    "truck": v,
                    "from_node": current,
                    "node_id": nxt,
                    "travel_time_hr": travel,
                    "earliest_physical_arrival_hr": arrive,
                    "window_open_hr": window_open if nxt != end else "",
                    "platform_recovery_ready_hr": recovery_ready if recovery_ready > 0 else "",
                    "earliest_required_time_hr": earliest_required,
                    "model_arrival_time_hr": model_time,
                    "slack_hr": model_time - earliest_required,
                    "note": note,
                }
            )
            depart_prev = earliest_required + (float(service_time.get(nxt, 0.0)) if nxt != end else 0.0)
            if nxt == end or nxt in visited:
                break
            visited.add(nxt)
            current = nxt
    return pd.DataFrame(rows)


def audit_selected_ordered_sorties(
    selected_sorties: pd.DataFrame,
    platform_label: str,
    selected_truck: pd.DataFrame,
    solution,
    truck_arrival_vars: dict,
    travel_time_matrix: np.ndarray,
    distance_matrix: np.ndarray,
    service_time: dict[int, float],
    open_time: dict[int, float],
    close_time: dict[int, float],
    tol: float = 1e-4,
) -> tuple[pd.DataFrame, pd.DataFrame]:
    summary_rows: list[dict] = []
    detail_rows: list[dict] = []
    if selected_sorties.empty:
        return selected_sorties, pd.DataFrame()

    selected_sorties = selected_sorties.copy()
    for row in selected_sorties.itertuples(index=False):
        vehicle_id = int(getattr(row, platform_label))
        sortie_id = int(row.sortie_id)
        launch = int(row.i)
        recovery = int(row.k)
        sequence = tuple(int(customer) for customer in row.sequence)
        route = (launch,) + sequence + (recovery,)
        launch_truck = int(row.launch_truck)
        recovery_truck = int(row.recovery_truck)
        truck_pair = f"{launch_truck}->{recovery_truck}"

        if launch != 0:
            matching_launch = selected_truck[
                (selected_truck["truck"].astype(int) == launch_truck)
                & (selected_truck["j"].astype(int) == launch)
            ]
            if matching_launch.empty:
                raise ValueError(
                    f"Selected {platform_label} sortie {sortie_id} launches from node {launch} "
                    f"with truck {launch_truck}, but that truck does not visit the launch node."
                )
        matching_recovery = selected_truck[
            (selected_truck["truck"].astype(int) == recovery_truck)
            & (selected_truck["j"].astype(int) == recovery)
        ]
        if matching_recovery.empty:
            raise ValueError(
                f"Selected {platform_label} sortie {sortie_id} recovers at node {recovery} "
                f"with truck {recovery_truck}, but that truck does not visit the recovery node."
            )

        truck_launch_time = value_or_zero(solution, truck_arrival_vars[launch_truck, launch])
        truck_recovery_time = value_or_zero(solution, truck_arrival_vars[recovery_truck, recovery])

        current_time = truck_launch_time
        total_distance = 0.0
        total_travel = 0.0
        total_service = 0.0
        total_wait = 0.0
        for from_node, to_node in zip(route[:-1], route[1:]):
            distance = float(distance_matrix[from_node, to_node])
            travel = float(travel_time_matrix[from_node, to_node])
            total_distance += distance
            total_travel += travel
            arrival_before_wait = current_time + travel
            if to_node == recovery:
                wait = 0.0
                service = 0.0
                service_start = arrival_before_wait
                service_finish = service_start
            else:
                service_start = max(arrival_before_wait, float(open_time.get(to_node, 0.0)))
                wait = service_start - arrival_before_wait
                service = float(service_time.get(to_node, 0.0))
                service_finish = service_start + service
                total_wait += wait
                total_service += service
                latest = float(close_time.get(to_node, math.inf))
                if service_start > latest + tol:
                    raise ValueError(
                        f"Time-window violation in {platform_label} sortie {sortie_id} "
                        f"at node {to_node}: service_start={service_start}, latest={latest}"
                    )

            detail_rows.append(
                {
                    "platform": platform_label,
                    "vehicle": vehicle_id,
                    "sortie_id": sortie_id,
                    "truck": truck_pair,
                    "launch_truck": launch_truck,
                    "recovery_truck": recovery_truck,
                    "route": row.route,
                    "leg": f"{from_node} -> {to_node}",
                    "from_node": from_node,
                    "to_node": to_node,
                    "travel_distance": distance,
                    "travel_duration_hr": travel,
                    "arrival_before_wait_hr": arrival_before_wait,
                    "wait_hr": wait,
                    "service_start_hr": service_start,
                    "service_duration_hr": service,
                    "service_finish_hr": service_finish,
                }
            )
            current_time = service_finish

        platform_finish_time = current_time
        active_duration = total_travel + total_service
        elapsed_schedule_time = truck_recovery_time - truck_launch_time
        slack_at_recovery = truck_recovery_time - platform_finish_time
        if platform_finish_time > truck_recovery_time + tol:
            raise ValueError(
                f"Synchronization violation in {platform_label} sortie {sortie_id}: "
                f"finish={platform_finish_time}, truck_recovery={truck_recovery_time}"
            )

        summary_rows.append(
            {
                "platform": platform_label,
                "vehicle": vehicle_id,
                "sortie_id": sortie_id,
                "truck": truck_pair,
                "launch_truck": launch_truck,
                "recovery_truck": recovery_truck,
                "launch_node": launch,
                "recovery_node": recovery,
                "route": row.route,
                "truck_launch_time_hr": truck_launch_time,
                "truck_recovery_time_hr": truck_recovery_time,
                "platform_finish_time_hr": platform_finish_time,
                "recovery_slack_hr": slack_at_recovery,
                "travel_distance": total_distance,
                "travel_duration_hr": total_travel,
                "service_duration_hr": total_service,
                "active_duration_hr": active_duration,
                "elapsed_schedule_time_hr": elapsed_schedule_time,
                "waiting_time_hr": total_wait,
                "launch_time_assumption": "truck_arrival_at_launch_node",
            }
        )

    summary_df = pd.DataFrame(summary_rows)
    detail_df = pd.DataFrame(detail_rows)
    summary_lookup = summary_df.set_index(["vehicle", "sortie_id"])
    selected_keys = list(zip(selected_sorties[platform_label].astype(int), selected_sorties["sortie_id"].astype(int)))
    for col in [
        "truck",
        "launch_truck",
        "recovery_truck",
        "truck_launch_time_hr",
        "truck_recovery_time_hr",
        "platform_finish_time_hr",
        "recovery_slack_hr",
        "travel_distance",
        "travel_duration_hr",
        "service_duration_hr",
        "active_duration_hr",
        "elapsed_schedule_time_hr",
        "waiting_time_hr",
    ]:
        selected_sorties[col] = [summary_lookup.loc[key, col] for key in selected_keys]
    selected_sorties = selected_sorties.rename(columns={"duration": "travel_duration_precomputed_hr"})
    return selected_sorties, detail_df


def build_platform_node_timing(selected_sorties: pd.DataFrame, detail_df: pd.DataFrame, platform_label: str) -> pd.DataFrame:
    rows: list[dict] = []
    if selected_sorties.empty:
        return pd.DataFrame(rows)

    for sortie in selected_sorties.itertuples(index=False):
        vehicle_id = int(getattr(sortie, platform_label))
        launch_node = int(sortie.i)
        recovery_node = int(sortie.k)
        launch_truck = int(sortie.launch_truck)
        recovery_truck = int(sortie.recovery_truck)
        truck_launch_time = float(sortie.truck_launch_time_hr)
        truck_recovery_time = float(sortie.truck_recovery_time_hr)
        rows.append(
            {
                "platform": platform_label,
                "vehicle": vehicle_id,
                "sortie_id": int(sortie.sortie_id),
                "truck": sortie.truck,
                "launch_truck": launch_truck,
                "recovery_truck": recovery_truck,
                "route": sortie.route,
                "node_id": launch_node,
                "node_role": "launch",
                "previous_node": "",
                "next_node": "",
                "arrival_before_wait_hr": truck_launch_time,
                "wait_hr": 0.0,
                "service_start_hr": truck_launch_time,
                "service_duration_hr": 0.0,
                "service_finish_hr": truck_launch_time,
                "truck_launch_time_hr": truck_launch_time,
                "truck_recovery_time_hr": truck_recovery_time,
                "recovery_slack_hr": float(sortie.recovery_slack_hr),
            }
        )

    if not detail_df.empty and "platform" in detail_df.columns:
        platform_detail = detail_df[detail_df["platform"] == platform_label].copy()
        for detail in platform_detail.itertuples(index=False):
            node_id = int(detail.to_node)
            recovery_node = int(selected_sorties.loc[selected_sorties["sortie_id"] == detail.sortie_id, "k"].iloc[0])
            role = "recovery" if node_id == recovery_node else "customer"
            rows.append(
                {
                    "platform": platform_label,
                    "vehicle": int(detail.vehicle),
                    "sortie_id": int(detail.sortie_id),
                    "truck": detail.truck,
                    "launch_truck": int(detail.launch_truck),
                    "recovery_truck": int(detail.recovery_truck),
                    "route": detail.route,
                    "node_id": node_id,
                    "node_role": role,
                    "previous_node": int(detail.from_node),
                    "next_node": "",
                    "arrival_before_wait_hr": float(detail.arrival_before_wait_hr),
                    "wait_hr": float(detail.wait_hr),
                    "service_start_hr": float(detail.service_start_hr),
                    "service_duration_hr": float(detail.service_duration_hr),
                    "service_finish_hr": float(detail.service_finish_hr),
                    "truck_launch_time_hr": "",
                    "truck_recovery_time_hr": "",
                    "recovery_slack_hr": "",
                }
            )

    return pd.DataFrame(rows).sort_values(["platform", "vehicle", "sortie_id", "service_start_hr", "node_role"])


def write_notes(big_m: float, operational_result: dict | None = None) -> None:
    lines = [
        "# Strict Math Model with Ordered Multi-Customer Sortie Improvement",
        "",
        "This file documents `drone_robot_truck_team_model_ordered_sorties_strict_math_improved.py`.",
        "",
        "## Paper Parameter Scenario",
        "This runner uses `data_processed/tdrp_tw_literature_params/<case>`, which stores the VRP-DR paper Table 3 values: truck speed 45 km/h, drone speed 75 km/h, robot speed 25 km/h, drone payload 25 kg, robot payload 20 kg, drone distance limit 20 km, and robot distance limit 15 km.",
        f"To keep the MILP solvable, ordered sortie generation is capped at {MAX_CUSTOMERS_PER_SORTIE} customers per sortie and the best {TOP_SORTIES_PER_TRUCK_LEG} routes per launch/recovery node pair.",
        "",
        "## Main Correction",
        "The model follows the math-model decision structure: truck arc variables `x`, drone sortie variables `y`, and robot sortie variables `z` decide the platform assignment.",
        "Customer assignment is not pre-filtered by eligibility columns.",
        "Assignment is determined by `truck_service[j] + drone_service[j] + robot_service[j] == 1`, while capacity, endurance, synchronization, and timing constraints control feasibility.",
        "",
        "## Ordered-Route Improvement",
        "The drone and robot variables use complete ordered sortie paths instead of single-customer sortie variables.",
        "A selected sortie can represent a path such as `6 -> 3 -> 8 -> 4`, so the route sequence is directly recoverable from the decision variable.",
        "",
        "## Retained Model Structure",
        "- Customer service exactly once.",
        "- Truck, drone, and robot capacity constraints (hard).",
        "- Drone and robot endurance enforced per sortie (hard): candidate generation excludes routes beyond the distance limit, and per-sortie constraints restate it in the MILP.",
        "- Each physical drone or robot performs at most one sortie, so no inter-sortie sequencing or onboard platform tracking is required. Fleet sizes in the processed cases exceed the number of useful sorties, so this does not restrict the solution space.",
        "- Flexible truck-platform synchronization for drone/robot launch and recovery.",
        "- Drone launch variable `h_launch[v,d,s]` identifies which truck launches each selected drone sortie.",
        "- Drone recovery variable `h_recover[v,d,s]` identifies which truck recovers each selected drone sortie.",
        "- Distinct drones interacting with each truck are limited by `MAX_DRONES_PER_TRUCK` using aggregate truck-drone pairing constraints.",
        "- Drone launches from the depot are limited by `DRONES_CARRIED_AT_DEPOT`; launches from any truck stop are limited by `MAX_DRONES_PER_TRUCK`.",
        "- Drone recoveries at truck stops and the end depot are limited by `MAX_DRONES_PER_TRUCK`.",
        "- Robot launch variable `g_launch[v,r,s]` identifies which truck launches each selected robot sortie.",
        "- Robot recovery variable `g_recover[v,r,s]` identifies which truck recovers each selected robot sortie.",
        "- Distinct robots interacting with each truck are limited by `MAX_ROBOTS_PER_TRUCK` using aggregate truck-robot pairing constraints.",
        "- Robot launches from the depot are limited by `ROBOTS_CARRIED_AT_DEPOT`; launches from any truck stop are limited by `MAX_ROBOTS_PER_TRUCK`.",
        "- Robot recoveries at truck stops and the end depot are limited by `MAX_ROBOTS_PER_TRUCK`.",
        "- Ordered sortie timing linked to truck launch and recovery times.",
        "- Time-window penalty constraints (soft).",
        "- Route-duration penalty (soft): truck depot departure is fixed at time 0, and each truck's end-depot arrival may exceed `T_max` only at a penalty. The global excess equals the maximum per-truck excess.",
        "- Operating cost = variable travel cost + fixed activation costs (truck 30 per used truck, drone 10 per selected sortie, robot 8 per selected sortie; Malik et al., VRP-DR, arXiv:2505.23584, Table 3). Fixed costs are written in parameters.json as truck_fixed_cost / drone_fixed_cost / robot_fixed_cost; the same values remain model defaults as a safeguard.",
        "- Operating cost plus the two soft penalties form the objective. Capacity and endurance are hard constraints with no penalty terms.",
        "- Symmetry breaking: platform k+1 may fly a sortie only if platform k does, valid because platforms are identical and limited to one sortie each.",
        "- Lossless fleet trimming: the model instantiates at most MAX_DRONES_PER_TRUCK x NUM_TRUCKS drones and MAX_ROBOTS_PER_TRUCK x NUM_TRUCKS robots, because the distinct-platform caps make any additional identical platforms provably unusable. This shrinks the MILP without changing the solution space.",
        "- Optional truck-only baseline: setting DRT_TRUCK_ONLY=1 fixes all drone and robot sortie variables to zero.",
        "- Makespan is computed after solving from the reported truck and platform finish times. It is not a decision variable and is not part of the objective, matching the PDF formulation (Z plus route-duration and time-window penalty terms).",
        "- The `Truck Physical Timing` sheet compares each model arrival time with the earliest physically required time (travel chain, time windows, platform recovery), making any remaining harmless slack in arrival variables explicit.",
        "",
        "## Added Operational Routing Completeness Constraints",
        "- Binary `used_truck[v]`.",
        "- Depot departure equals `used_truck[v]`.",
        "- Depot return equals `used_truck[v]`.",
        "- Truck flow conservation at customer nodes.",
        "- Binary `visit[v,i]` linked to inbound truck arcs.",
        "- Truck service relation `truck_service[i] == sum_v visit[v,i]`.",
        "- Truck-use activation constraints.",
        "- Used trucks must serve at least one customer.",
        "- Truck-indexed timing propagation includes truck service at the departure customer before traveling to the next node.",
        "- Truck time-window constraints linked to visits.",
        "- Truck route-duration diagnostics per truck.",
        "- MTZ subtour-elimination constraints for customer-customer arcs.",
        "",
        "## Ordered Sortie Generation",
        "- Each feasible drone/robot sortie is an ordered customer sequence between a launch node and a recovery node.",
        f"- Candidate generation keeps at most {TOP_SORTIES_PER_TRUCK_LEG} high-coverage, low-cost ordered routes per launch/recovery node pair.",
        "- A sequence is generated only if its total demand is within platform capacity.",
        "- A sequence is generated only if its full route distance is within platform endurance.",
        "- The full route cost and duration use every consecutive arc in the ordered path.",
        "- The selected route timing propagates through the ordered customer sequence and includes customer service time.",
        "- Output sheets distinguish precomputed travel duration from service duration, active duration, elapsed schedule time, waiting time, and recovery slack.",
        "- Output sheets also report selected sortie travel distance by summing the platform-specific distance on every route leg.",
        "",
        "## Synchronization Interpretation",
        "The model uses flexible truck-platform synchronization: an ordered sortie from `i` to `k` has one launch truck and one recovery truck.",
        "If `h_launch[v,d,s] = 1`, drone `d` may launch only after truck `v` arrives at launch node `i`.",
        "If `h_recover[v,d,s] = 1`, drone `d` must recover no later than truck `v` arrives at recovery node `k`.",
        "The same interpretation applies to robot variables `g_launch[v,r,s]` and `g_recover[v,r,s]`.",
        "",
        f"## Big-M",
        f"Chosen M = `{big_m}`.",
        "",
    ]
    if operational_result:
        lines.extend(
            [
                "",
                "## Ordered-Sortie Result",
                f"- Solve status: {operational_result.get('solve_status')}",
                f"- Objective: {operational_result.get('objective')}",
                f"- Truck arcs: {operational_result.get('truck_arcs')}",
                f"- Drone sorties: {operational_result.get('drone_sorties')}",
                f"- Robot sorties: {operational_result.get('robot_sorties')}",
                f"- Feasible drone sequences generated: {operational_result.get('feasible_drone_sequences')}",
                f"- Feasible robot sequences generated: {operational_result.get('feasible_robot_sequences')}",
                f"- Reported makespan: {operational_result.get('reported_makespan_hr')}",
                f"- Route duration excess: {operational_result.get('route_duration_excess')}",
                f"- Total penalty: {operational_result.get('total_penalty')}",
            ]
        )
    lines.extend(
        [
            "",
            "## Professor's Route-Confirmation Concern",
            "The selected sortie sheet prints the complete ordered path, so a route like `6 -> 3 -> 8 -> 4` is confirmed by one selected variable rather than inferred from multiple independent single-customer sorties.",
        ]
    )
    NOTES_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    start_clock = time.perf_counter()
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    data = read_inputs()
    validate_inputs(data)
    nodes = data["nodes"].copy()
    customers = data["customers"].copy()
    arcs = data["arcs"].copy()
    params = data["parameters"]
    instance_summary = data["instance_summary"]

    V = sorted(nodes["node_id"].astype(int).tolist())
    C = sorted(customers["customer_id"].astype(int).tolist())
    n = len(C)
    start_depot = 0
    end_depot = n + 1
    P = [start_depot, end_depot]
    T = list(range(1, int(params["NUM_TRUCKS"]) + 1))
    D = list(range(1, int(params["NUM_DRONES"]) + 1))
    R = list(range(1, int(params["NUM_ROBOTS"]) + 1))
    A = [(int(row.i), int(row.j)) for row in arcs.itertuples(index=False)]
    A_set = set(A)
    sortie_triplets = [(i, j, k) for (i, k) in A for j in C if i != j and j != k]

    q = customers.set_index("customer_id")["demand"].astype(float).to_dict()
    open_time = customers.set_index("customer_id")["open_time"].astype(float).to_dict()
    close_time = customers.set_index("customer_id")["close_time"].astype(float).to_dict()
    service_time = customers.set_index("customer_id")["service_time"].astype(float).to_dict()
    distance = data["distance_matrix"]
    truck_time = data["truck_time_matrix"]
    drone_time = data["drone_time_matrix"]
    robot_distance = data["robot_distance_matrix"]
    robot_time = data["robot_time_matrix"]

    Q_t = float(params["Q_t"])
    Q_d = float(params["Q_d"])
    Q_r = float(params["Q_r"])
    E_d = float(params["E_d"])
    E_r = float(params["E_r"])
    drones_carried_at_depot = int(params.get("DRONES_CARRIED_AT_DEPOT", len(D)))
    max_drones_per_truck = int(params.get("MAX_DRONES_PER_TRUCK", len(D)))
    robots_carried_at_depot = int(params.get("ROBOTS_CARRIED_AT_DEPOT", len(R)))
    max_robots_per_truck = int(params.get("MAX_ROBOTS_PER_TRUCK", len(R)))
    # Lossless fleet trimming. The distinct-platform cap allows at most
    # MAX_*_PER_TRUCK platforms per truck, so no more than
    # max_per_truck * NUM_TRUCKS platforms of each type can ever be used in
    # any feasible solution. Platforms beyond that bound are provably
    # unusable; dropping them shrinks the model (variables scale linearly in
    # fleet size) without changing the solution space, because platforms are
    # identical and symmetry-broken.
    # Two independent lossless bounds: (a) the distinct-platform caps allow at
    # most max_per_truck * NUM_TRUCKS platforms per type; (b) each used
    # platform flies exactly one sortie (one-sortie-per-platform) serving at
    # least one customer, and each customer is served at most once, so at most
    # n platforms of each type can be used in any feasible solution.
    max_usable_drones = max(1, min(max_drones_per_truck * len(T), n))
    max_usable_robots = max(1, min(max_robots_per_truck * len(T), n))
    if len(D) > max_usable_drones:
        D = D[:max_usable_drones]
    if len(R) > max_usable_robots:
        R = R[:max_usable_robots]
    T_max = float(params["T_max"])
    C_w = float(params["C_w"])
    C_veh = float(params["C_veh"])
    C_drone = float(params.get("C_drone", C_veh))
    C_w_drone = float(params.get("C_w_drone", C_w))
    C_w_r = float(params["C_w_r"])
    C_rob = float(params["C_rob"])
    # Fixed activation costs (Malik et al., VRP-DR, arXiv:2505.23584, Table 3).
    # Fixed costs are explicit in processed parameters; defaults are safeguards.
    f_t = float(params.get("truck_fixed_cost", 30.0))
    f_d = float(params.get("drone_fixed_cost", 10.0))
    f_r = float(params.get("robot_fixed_cost", 8.0))
    lambda_T = float(params["lambda_T"])
    lambda_W = float(params["lambda_W"])
    max_time_value = max(float(np.max(truck_time)), float(np.max(drone_time)), float(np.max(robot_time)))
    big_m = max(T_max, max_time_value * len(V) * 10, 10000.0)
    write_notes(big_m)

    truck_cost = {(i, j): truck_time[i, j] * C_w + distance[i, j] * C_veh for (i, j) in A}
    drone_cost = {(i, j): drone_time[i, j] * C_w_drone + distance[i, j] * C_drone for i in V for j in V}
    robot_cost = {(i, j): robot_time[i, j] * C_w_r + robot_distance[i, j] * C_rob for i in V for j in V}
    drone_sorties = generate_ordered_sorties(A, C, q, Q_d, E_d, drone_time, distance, drone_cost)
    robot_sorties = generate_ordered_sorties(A, C, q, Q_r, E_r, robot_time, robot_distance, robot_cost)
    drone_sortie_ids = drone_sorties["sortie_id"].astype(int).tolist()
    robot_sortie_ids = robot_sorties["sortie_id"].astype(int).tolist()
    drone_sortie_by_id = drone_sorties.set_index("sortie_id").to_dict(orient="index")
    robot_sortie_by_id = robot_sorties.set_index("sortie_id").to_dict(orient="index")

    # DRT_FAST_BUILD=1 discards constraint/variable names and disables the
    # docplex type checker during model construction. This substantially
    # reduces Python-side build memory for very large cases (e.g. 11-25 with
    # 8 trucks) at the cost of less readable LP exports and harder
    # infeasibility debugging. The MILP itself is identical.
    fast_build = os.environ.get("DRT_FAST_BUILD", "0").strip() == "1"
    mdl = Model(
        name="drone_robot_truck_ordered_sorties_first_instance",
        ignore_names=fast_build,
        checker="off" if fast_build else "default",
    )
    mdl.parameters.timelimit = TIME_LIMIT_SECONDS
    mdl.parameters.threads = 2
    # Memory hygiene: prefer disk over RAM for the branch-and-bound tree so
    # large cases degrade gracefully instead of failing with CPLEX Error 1001.
    mdl.parameters.emphasis.memory = 1
    mdl.parameters.workmem = 4096
    mdl.parameters.mip.strategy.file = 3

    x = mdl.binary_var_dict(((v, i, j) for v in T for (i, j) in A), name="x")
    # Ordered sortie variables. Each sortie id represents a full path such as
    # 6 -> 3 -> 8 -> 4, not a single-customer shorthand.
    y = mdl.binary_var_dict(((d, sid) for d in D for sid in drone_sortie_ids), name="y")
    h_launch = mdl.binary_var_dict(((v, d, sid) for v in T for d in D for sid in drone_sortie_ids), name="h_launch")
    h_recover = mdl.binary_var_dict(((v, d, sid) for v in T for d in D for sid in drone_sortie_ids), name="h_recover")
    drone_truck = mdl.binary_var_dict(((v, d) for v in T for d in D), name="drone_truck")
    z = mdl.binary_var_dict(((r, sid) for r in R for sid in robot_sortie_ids), name="z")
    g_launch = mdl.binary_var_dict(((v, r, sid) for v in T for r in R for sid in robot_sortie_ids), name="g_launch")
    g_recover = mdl.binary_var_dict(((v, r, sid) for v in T for r in R for sid in robot_sortie_ids), name="g_recover")
    robot_truck = mdl.binary_var_dict(((v, r) for v in T for r in R), name="robot_truck")
    used_truck = mdl.binary_var_dict(T, name="used_truck")
    visit = mdl.binary_var_dict(((v, i) for v in T for i in C), name="visit")
    u = mdl.continuous_var_dict(((v, i) for v in T for i in C), lb=0, ub=n, name="u")

    if TRUCK_ONLY:
        # Truck-only baseline: fix every drone and robot sortie variable to zero.
        # The h/g assignment variables follow automatically through
        # sum_v h_launch = y and sum_v g_launch = z.
        for var in y.values():
            var.ub = 0
        for var in z.values():
            var.ub = 0

    a_t = mdl.continuous_var_dict(((v, i) for v in T for i in V), lb=0, name="a_t")
    a_d = mdl.continuous_var_dict(V, lb=0, name="a_d")
    a_r = mdl.continuous_var_dict(V, lb=0, name="a_r")
    late_t = mdl.continuous_var_dict(C, lb=0, name="late_t")
    late_d = mdl.continuous_var_dict(C, lb=0, name="late_d")
    late_r = mdl.continuous_var_dict(C, lb=0, name="late_r")
    route_late_t = mdl.continuous_var_dict(T, lb=0, name="route_late_t")
    route_late_global = mdl.continuous_var(lb=0, name="route_late_global")

    truck_service = {i: mdl.sum(visit[v, i] for v in T) for i in C}
    drone_service = {
        j: mdl.sum(y[d, sid] for d in D for sid, row in drone_sortie_by_id.items() if j in row["sequence"])
        for j in C
    }
    robot_service = {
        j: mdl.sum(z[r, sid] for r in R for sid, row in robot_sortie_by_id.items() if j in row["sequence"])
        for j in C
    }

    # PDF-based customer service.
    for i in C:
        mdl.add_constraint(truck_service[i] + drone_service[i] + robot_service[i] == 1, ctname=f"customer_service_{i}")

    # PDF-based capacity constraints.
    for v in T:
        load_v = mdl.sum(q[i] * visit[v, i] for i in C)
        mdl.add_constraint(load_v <= Q_t, ctname=f"truck_capacity_{v}")
    for d, sid in y:
        mdl.add_constraint(drone_sortie_by_id[sid]["payload"] * y[d, sid] <= Q_d, ctname=f"drone_capacity_{d}_{sid}")
    for r, sid in z:
        mdl.add_constraint(robot_sortie_by_id[sid]["payload"] * z[r, sid] <= Q_r, ctname=f"robot_capacity_{r}_{sid}")

    # PDF-based endurance constraints. In this paper-parameter scenario,
    # E_d and E_r are distance limits that apply per sortie: the battery is
    # swapped or recharged when the platform docks with a truck. Candidate
    # generation already excludes sorties whose full route distance exceeds
    # the limit; these per-sortie constraints restate the limit explicitly
    # so the model remains correct if the generation filter changes.
    for d, sid in y:
        mdl.add_constraint(
            drone_sortie_by_id[sid]["travel_distance"] * y[d, sid] <= E_d,
            ctname=f"drone_endurance_{d}_{sid}",
        )
    for r, sid in z:
        mdl.add_constraint(
            robot_sortie_by_id[sid]["travel_distance"] * z[r, sid] <= E_r,
            ctname=f"robot_endurance_{r}_{sid}",
        )

    # Flexible docking synchronization. The launch truck and recovery truck
    # can be different vehicles, as long as each truck visits the proper node.
    for d in D:
        for sid in drone_sortie_ids:
            row = drone_sortie_by_id[sid]
            launch = int(row["i"])
            recover = int(row["k"])
            mdl.add_constraint(
                mdl.sum(h_launch[v, d, sid] for v in T) == y[d, sid],
                ctname=f"assign_selected_drone_sortie_launch_truck_{d}_{sid}",
            )
            mdl.add_constraint(
                mdl.sum(h_recover[v, d, sid] for v in T) == y[d, sid],
                ctname=f"assign_selected_drone_sortie_recovery_truck_{d}_{sid}",
            )
            for v in T:
                if launch == start_depot:
                    mdl.add_constraint(h_launch[v, d, sid] <= used_truck[v], ctname=f"sync_drone_launch_depot_{v}_{d}_{sid}")
                elif launch in C:
                    mdl.add_constraint(h_launch[v, d, sid] <= visit[v, launch], ctname=f"sync_drone_launch_visit_{v}_{d}_{sid}")
                else:
                    mdl.add_constraint(h_launch[v, d, sid] <= 0, ctname=f"sync_drone_launch_invalid_{v}_{d}_{sid}")

                if recover == end_depot:
                    mdl.add_constraint(h_recover[v, d, sid] <= used_truck[v], ctname=f"sync_drone_recover_depot_{v}_{d}_{sid}")
                elif recover in C:
                    mdl.add_constraint(h_recover[v, d, sid] <= visit[v, recover], ctname=f"sync_drone_recover_visit_{v}_{d}_{sid}")
                else:
                    mdl.add_constraint(h_recover[v, d, sid] <= 0, ctname=f"sync_drone_recover_invalid_{v}_{d}_{sid}")
        # Physical-platform consistency: each drone performs at most one sortie.
        # This dominates the older per-node departure/recovery exclusivity and
        # removes the need for inter-sortie sequencing or onboard drone tracking.
        # The processed cases carry more drones than useful sorties, so this
        # constraint does not restrict the solution space in these experiments.
        mdl.add_constraint(
            mdl.sum(y[d, sid] for sid in drone_sortie_ids) <= 1,
            ctname=f"one_sortie_per_drone_{d}",
        )
    for v in T:
        mdl.add_constraint(
            mdl.sum(drone_truck[v, d] for d in D) <= max_drones_per_truck * used_truck[v],
            ctname=f"max_distinct_drones_assigned_to_truck_{v}",
        )
        for d in D:
            drone_use_by_truck = mdl.sum(h_launch[v, d, sid] + h_recover[v, d, sid] for sid in drone_sortie_ids)
            # With at most one sortie per drone, a drone does at most one launch
            # and one recovery in total, so 2 is a valid tight coefficient.
            mdl.add_constraint(drone_use_by_truck <= 2 * drone_truck[v, d], ctname=f"drone_truck_pair_upper_{v}_{d}")
            mdl.add_constraint(drone_truck[v, d] <= drone_use_by_truck, ctname=f"drone_truck_pair_lower_{v}_{d}")
        for launch in V:
            launch_sorties = [sid for sid in drone_sortie_ids if int(drone_sortie_by_id[sid]["i"]) == launch]
            if not launch_sorties:
                continue
            launch_count = mdl.sum(h_launch[v, d, sid] for d in D for sid in launch_sorties)
            if launch == start_depot:
                mdl.add_constraint(
                    launch_count <= drones_carried_at_depot * used_truck[v],
                    ctname=f"depot_drone_launch_limit_{v}",
                )
            else:
                mdl.add_constraint(
                    launch_count <= max_drones_per_truck * visit[v, launch],
                    ctname=f"node_drone_launch_limit_{v}_{launch}",
                )
        for recover in V:
            recovery_sorties = [sid for sid in drone_sortie_ids if int(drone_sortie_by_id[sid]["k"]) == recover]
            if not recovery_sorties:
                continue
            recovery_count = mdl.sum(h_recover[v, d, sid] for d in D for sid in recovery_sorties)
            if recover == end_depot:
                mdl.add_constraint(
                    recovery_count <= max_drones_per_truck * used_truck[v],
                    ctname=f"depot_drone_recovery_limit_{v}",
                )
            elif recover in C:
                mdl.add_constraint(
                    recovery_count <= max_drones_per_truck * visit[v, recover],
                    ctname=f"node_drone_recovery_limit_{v}_{recover}",
                )

    for r in R:
        for sid in robot_sortie_ids:
            row = robot_sortie_by_id[sid]
            launch = int(row["i"])
            recover = int(row["k"])
            mdl.add_constraint(
                mdl.sum(g_launch[v, r, sid] for v in T) == z[r, sid],
                ctname=f"assign_selected_robot_sortie_launch_truck_{r}_{sid}",
            )
            mdl.add_constraint(
                mdl.sum(g_recover[v, r, sid] for v in T) == z[r, sid],
                ctname=f"assign_selected_robot_sortie_recovery_truck_{r}_{sid}",
            )
            for v in T:
                if launch == start_depot:
                    mdl.add_constraint(g_launch[v, r, sid] <= used_truck[v], ctname=f"sync_robot_launch_depot_{v}_{r}_{sid}")
                elif launch in C:
                    mdl.add_constraint(g_launch[v, r, sid] <= visit[v, launch], ctname=f"sync_robot_launch_visit_{v}_{r}_{sid}")
                else:
                    mdl.add_constraint(g_launch[v, r, sid] <= 0, ctname=f"sync_robot_launch_invalid_{v}_{r}_{sid}")

                if recover == end_depot:
                    mdl.add_constraint(g_recover[v, r, sid] <= used_truck[v], ctname=f"sync_robot_recover_depot_{v}_{r}_{sid}")
                elif recover in C:
                    mdl.add_constraint(g_recover[v, r, sid] <= visit[v, recover], ctname=f"sync_robot_recover_visit_{v}_{r}_{sid}")
                else:
                    mdl.add_constraint(g_recover[v, r, sid] <= 0, ctname=f"sync_robot_recover_invalid_{v}_{r}_{sid}")
        # Physical-platform consistency: each robot performs at most one sortie.
        # Same reasoning as the drone constraint above.
        mdl.add_constraint(
            mdl.sum(z[r, sid] for sid in robot_sortie_ids) <= 1,
            ctname=f"one_sortie_per_robot_{r}",
        )
    for v in T:
        mdl.add_constraint(
            mdl.sum(robot_truck[v, r] for r in R) <= max_robots_per_truck * used_truck[v],
            ctname=f"max_distinct_robots_assigned_to_truck_{v}",
        )
        for r in R:
            robot_use_by_truck = mdl.sum(g_launch[v, r, sid] + g_recover[v, r, sid] for sid in robot_sortie_ids)
            mdl.add_constraint(robot_use_by_truck <= 2 * robot_truck[v, r], ctname=f"robot_truck_pair_upper_{v}_{r}")
            mdl.add_constraint(robot_truck[v, r] <= robot_use_by_truck, ctname=f"robot_truck_pair_lower_{v}_{r}")
        for launch in V:
            launch_sorties = [sid for sid in robot_sortie_ids if int(robot_sortie_by_id[sid]["i"]) == launch]
            if not launch_sorties:
                continue
            launch_count = mdl.sum(g_launch[v, r, sid] for r in R for sid in launch_sorties)
            if launch == start_depot:
                mdl.add_constraint(
                    launch_count <= robots_carried_at_depot * used_truck[v],
                    ctname=f"depot_robot_launch_limit_{v}",
                )
            else:
                mdl.add_constraint(
                    launch_count <= max_robots_per_truck * visit[v, launch],
                    ctname=f"node_robot_launch_limit_{v}_{launch}",
                )
        for recover in V:
            recovery_sorties = [sid for sid in robot_sortie_ids if int(robot_sortie_by_id[sid]["k"]) == recover]
            if not recovery_sorties:
                continue
            recovery_count = mdl.sum(g_recover[v, r, sid] for r in R for sid in recovery_sorties)
            if recover == end_depot:
                mdl.add_constraint(
                    recovery_count <= max_robots_per_truck * used_truck[v],
                    ctname=f"depot_robot_recovery_limit_{v}",
                )
            elif recover in C:
                mdl.add_constraint(
                    recovery_count <= max_robots_per_truck * visit[v, recover],
                    ctname=f"node_robot_recovery_limit_{v}_{recover}",
                )

    # Symmetry breaking. All drones (and all robots) are identical and each
    # performs at most one sortie, so platform k+1 may fly only if platform k
    # flies. This removes permutation-equivalent solutions without cutting off
    # any distinct physical plan.
    for d_prev, d_next in zip(D, D[1:]):
        mdl.add_constraint(
            mdl.sum(y[d_next, sid] for sid in drone_sortie_ids)
            <= mdl.sum(y[d_prev, sid] for sid in drone_sortie_ids),
            ctname=f"symmetry_drone_{d_prev}_{d_next}",
        )
    for r_prev, r_next in zip(R, R[1:]):
        mdl.add_constraint(
            mdl.sum(z[r_next, sid] for sid in robot_sortie_ids)
            <= mdl.sum(z[r_prev, sid] for sid in robot_sortie_ids),
            ctname=f"symmetry_robot_{r_prev}_{r_next}",
        )

    # PDF-based time windows for drone/robot modes; truck mode is integrated below with visit[v,i].
    mdl.add_constraint(a_d[start_depot] == 0, ctname="start_time_d")
    mdl.add_constraint(a_r[start_depot] == 0, ctname="start_time_r")
    for i in C:
        mdl.add_constraint(a_d[i] >= open_time[i] * drone_service[i], ctname=f"open_drone_{i}")
        mdl.add_constraint(a_r[i] >= open_time[i] * robot_service[i], ctname=f"open_robot_{i}")
        mdl.add_constraint(late_d[i] >= a_d[i] - close_time[i], ctname=f"late_drone_def_{i}")
        mdl.add_constraint(late_r[i] >= a_r[i] - close_time[i], ctname=f"late_robot_def_{i}")
        mdl.add_constraint(a_d[i] <= close_time[i] + late_d[i] + big_m * (1 - drone_service[i]), ctname=f"time_window_drone_{i}")
        mdl.add_constraint(a_r[i] <= close_time[i] + late_r[i] + big_m * (1 - robot_service[i]), ctname=f"time_window_robot_{i}")

    # Ordered sortie timing. These constraints force the selected ordered
    # drone/robot path to occur between its launch-truck and recovery-truck times.
    for d in D:
        for sid in drone_sortie_ids:
            row = drone_sortie_by_id[sid]
            launch = int(row["i"])
            recover = int(row["k"])
            sequence = tuple(int(customer) for customer in row["sequence"])
            first = sequence[0]
            last = sequence[-1]
            for v in T:
                mdl.add_constraint(
                    a_d[first] >= a_t[v, launch] + drone_time[launch, first] - big_m * (1 - h_launch[v, d, sid]),
                    ctname=f"sortie_time_drone_launch_{d}_{sid}_{v}",
                )
                mdl.add_constraint(
                    a_t[v, recover] >= a_d[last] + service_time[last] + drone_time[last, recover] - big_m * (1 - h_recover[v, d, sid]),
                    ctname=f"sortie_time_drone_recover_{d}_{sid}_{v}",
                )
            for prev, nxt in zip(sequence[:-1], sequence[1:]):
                mdl.add_constraint(
                    a_d[nxt] >= a_d[prev] + service_time[prev] + drone_time[prev, nxt] - big_m * (1 - y[d, sid]),
                    ctname=f"sortie_time_drone_seq_{d}_{sid}_{prev}_{nxt}",
                )

    for r in R:
        for sid in robot_sortie_ids:
            row = robot_sortie_by_id[sid]
            launch = int(row["i"])
            recover = int(row["k"])
            sequence = tuple(int(customer) for customer in row["sequence"])
            first = sequence[0]
            last = sequence[-1]
            for v in T:
                mdl.add_constraint(
                    a_r[first] >= a_t[v, launch] + robot_time[launch, first] - big_m * (1 - g_launch[v, r, sid]),
                    ctname=f"sortie_time_robot_launch_{r}_{sid}_{v}",
                )
                mdl.add_constraint(
                    a_t[v, recover] >= a_r[last] + service_time[last] + robot_time[last, recover] - big_m * (1 - g_recover[v, r, sid]),
                    ctname=f"sortie_time_robot_recover_{r}_{sid}_{v}",
                )
            for prev, nxt in zip(sequence[:-1], sequence[1:]):
                mdl.add_constraint(
                    a_r[nxt] >= a_r[prev] + service_time[prev] + robot_time[prev, nxt] - big_m * (1 - z[r, sid]),
                    ctname=f"sortie_time_robot_seq_{r}_{sid}_{prev}_{nxt}",
                )

    # ============================================================
    # OPERATIONAL ROUTING COMPLETENESS CONSTRAINTS
    # These constraints are added because the PDF defines truck arc
    # variables but does not explicitly write full vehicle-route
    # continuity constraints. They are standard routing constraints
    # needed to make x[v,i,j] form valid physical truck routes.
    # ============================================================

    # COMPUTATIONAL NOTE: used_truck[v] controls whether truck v leaves the start depot.
    for v in T:
        mdl.add_constraint(mdl.sum(x[v, start_depot, j] for j in V if (start_depot, j) in A_set) == used_truck[v], ctname=f"op_depot_departure_{v}")
        mdl.add_constraint(mdl.sum(x[v, i, end_depot] for i in V if (i, end_depot) in A_set) == used_truck[v], ctname=f"op_depot_return_{v}")

    # COMPUTATIONAL NOTE: flow conservation makes a visited customer have matching inbound/outbound truck arcs.
    for v in T:
        for node in C:
            mdl.add_constraint(
                mdl.sum(x[v, i, node] for i in V if (i, node) in A_set)
                == mdl.sum(x[v, node, j] for j in V if (node, j) in A_set),
                ctname=f"op_flow_{v}_{node}",
            )

    # COMPUTATIONAL NOTE: visit[v,i] indicates whether truck v physically visits customer i.
    for v in T:
        for i in C:
            mdl.add_constraint(visit[v, i] == mdl.sum(x[v, h, i] for h in V if (h, i) in A_set), ctname=f"op_visit_inbound_{v}_{i}")

    # COMPUTATIONAL NOTE: truck service remains the PDF service term, now tied to physical visits.
    for i in C:
        mdl.add_constraint(truck_service[i] == mdl.sum(visit[v, i] for v in T), ctname=f"op_truck_service_visit_{i}")

    # COMPUTATIONAL NOTE: unused trucks cannot carry customer visits or arcs.
    for v in T:
        mdl.add_constraint(mdl.sum(visit[v, i] for i in C) <= n * used_truck[v], ctname=f"op_used_visit_activation_{v}")
        mdl.add_constraint(used_truck[v] <= mdl.sum(visit[v, i] for i in C), ctname=f"op_used_requires_service_{v}")
        for i, j in A:
            mdl.add_constraint(x[v, i, j] <= used_truck[v], ctname=f"op_arc_activation_{v}_{i}_{j}")

    # COMPUTATIONAL NOTE: truck-indexed arrival times propagate along selected physical arcs.
    # Truck depot departure is fixed at clock time 0 by modeling choice.
    for v in T:
        mdl.add_constraint(a_t[v, start_depot] == 0, ctname=f"op_start_time_truck_{v}")

    for v in T:
        for i, j in A:
            departure_service = float(service_time.get(i, 0.0))
            mdl.add_constraint(
                a_t[v, j] >= a_t[v, i] + departure_service + truck_time[i, j] - big_m * (1 - x[v, i, j]),
                ctname=f"op_time_propagation_{v}_{i}_{j}",
            )

    # COMPUTATIONAL NOTE: truck time-window penalties apply only when truck v visits customer i.
    for v in T:
        for i in C:
            mdl.add_constraint(a_t[v, i] >= open_time[i] - big_m * (1 - visit[v, i]), ctname=f"op_open_truck_{v}_{i}")
            mdl.add_constraint(late_t[i] >= a_t[v, i] - close_time[i] - big_m * (1 - visit[v, i]), ctname=f"op_late_truck_{v}_{i}")

    # COMPUTATIONAL NOTE: truck route duration is enforced per used truck.
    # Since truck departure is fixed at time 0, duration equals end-depot arrival.
    for v in T:
        mdl.add_constraint(
            route_late_t[v] >= a_t[v, end_depot] - T_max - big_m * (1 - used_truck[v]),
            ctname=f"op_route_duration_late_truck_{v}",
        )
        mdl.add_constraint(
            a_t[v, end_depot] <= T_max + route_late_t[v] + big_m * (1 - used_truck[v]),
            ctname=f"op_route_duration_truck_{v}",
        )
        mdl.add_constraint(route_late_t[v] <= big_m * used_truck[v], ctname=f"op_route_duration_late_truck_activation_{v}")
        mdl.add_constraint(route_late_t[v] <= route_late_global + big_m * (1 - used_truck[v]), ctname=f"op_route_duration_late_truck_global_bound_{v}")

    # route_late_global equals the maximum per-truck route-duration excess via
    # the per-truck bounds above.

    # COMPUTATIONAL NOTE:
    # MTZ constraints are added to prevent disconnected truck subtours,
    # which are not explicitly written in the PDF but are necessary for
    # operational route validity.
    for v in T:
        for i in C:
            mdl.add_constraint(u[v, i] <= n * visit[v, i], ctname=f"op_mtz_visit_upper_{v}_{i}")
            mdl.add_constraint(u[v, i] >= visit[v, i], ctname=f"op_mtz_visit_lower_{v}_{i}")
        for i in C:
            for j in C:
                if i != j and (i, j) in A_set:
                    mdl.add_constraint(u[v, i] - u[v, j] + n * x[v, i, j] <= n - 1, ctname=f"op_mtz_{v}_{i}_{j}")

    # Variable (distance/time) operating costs.
    truck_variable_cost = mdl.sum(truck_cost[i, j] * x[v, i, j] for v in T for (i, j) in A)
    drone_variable_cost = mdl.sum(drone_sortie_by_id[sid]["cost"] * y[d, sid] for d in D for sid in drone_sortie_ids)
    robot_variable_cost = mdl.sum(robot_sortie_by_id[sid]["cost"] * z[r, sid] for r in R for sid in robot_sortie_ids)
    total_variable_cost = truck_variable_cost + drone_variable_cost + robot_variable_cost
    # Fixed activation costs (Malik Table 3): per used truck, per selected
    # drone sortie, per selected robot sortie. With the one-sortie-per-platform
    # constraint, a selected sortie equals one platform deployment.
    truck_fixed_cost = f_t * mdl.sum(used_truck[v] for v in T)
    drone_fixed_cost = f_d * mdl.sum(y[d, sid] for d in D for sid in drone_sortie_ids)
    robot_fixed_cost = f_r * mdl.sum(z[r, sid] for r in R for sid in robot_sortie_ids)
    total_fixed_cost = truck_fixed_cost + drone_fixed_cost + robot_fixed_cost
    operating_cost = total_variable_cost + total_fixed_cost
    # Capacity and endurance are hard constraints (endurance is additionally
    # enforced at candidate generation), so they carry no soft penalty terms.
    # Only route-duration and time-window violations are penalized.
    route_duration_penalty = lambda_T * route_late_global
    time_window_penalty = lambda_W * (mdl.sum(late_t[i] for i in C) + mdl.sum(late_d[i] for i in C) + mdl.sum(late_r[i] for i in C))
    total_penalty = route_duration_penalty + time_window_penalty
    # Objective matches the PDF formulation (P = Z + penalty terms, Eq. 12):
    # operating cost plus route-duration and time-window penalties.
    objective = operating_cost + total_penalty
    mdl.minimize(objective)

    kpi_exprs = {
        "Objective": objective,
        "Operating Cost": operating_cost,
        "Route Duration Penalty": route_duration_penalty,
        "Route Duration Excess": route_late_global,
        "Time Window Penalty": time_window_penalty,
        "Total Penalty": total_penalty,
        "Used Trucks": mdl.sum(used_truck[v] for v in T),
        # Cost breakdown (snake_case fields for report tables).
        "truck_variable_cost": truck_variable_cost,
        "drone_variable_cost": drone_variable_cost,
        "robot_variable_cost": robot_variable_cost,
        "truck_fixed_cost": truck_fixed_cost,
        "drone_fixed_cost": drone_fixed_cost,
        "robot_fixed_cost": robot_fixed_cost,
        "total_variable_cost": total_variable_cost,
        "total_fixed_cost": total_fixed_cost,
        "operating_cost": operating_cost,
        "route_duration_penalty": route_duration_penalty,
        "time_window_penalty": time_window_penalty,
        "total_penalty": total_penalty,
        "objective_value": objective,
    }
    for name, expr in kpi_exprs.items():
        mdl.add_kpi(expr, publish_name=name)

    truck_warm_start_added = False
    if USE_TRUCK_WARM_START:
        truck_warm_start_added = add_truck_only_mip_start(
            mdl=mdl,
            T=T,
            C=C,
            V=V,
            A_set=A_set,
            start_depot=start_depot,
            end_depot=end_depot,
            demand=q,
            truck_capacity=Q_t,
            truck_time=truck_time,
            open_time=open_time,
            close_time=close_time,
            service_time=service_time,
            T_max=T_max,
            x=x,
            used_truck=used_truck,
            visit=visit,
            u=u,
            a_t=a_t,
            late_t=late_t,
            route_late_t=route_late_t,
            route_late_global=route_late_global,
            y=y,
            z=z,
            drone_truck=drone_truck,
            robot_truck=robot_truck,
        )

    print("\nOperational model information")
    print("-" * 60)
    mdl.print_information()
    print(f"Truck-only warm start added: {truck_warm_start_added}")
    exported_lp = None
    if EXPORT_LP:
        exported_lp = mdl.export_as_lp(path=str(RESULTS_DIR), basename=LP_BASENAME)
        print(f"LP exported to: {exported_lp}")
    else:
        print("LP export skipped; set DRT_EXPORT_LP=1 to write the .lp file.")

    solution = None
    solve_exception = None
    try:
        solution = mdl.solve(log_output=True)
    except Exception as exc:
        solve_exception = exc

    runtime = time.perf_counter() - start_clock
    solve_status = str(mdl.solve_details.status) if mdl.solve_details else "not solved"
    objective_value = value_or_zero(solution, objective) if solution is not None else math.nan
    selected_truck = selected_rows(solution, x, ["truck", "i", "j"])
    selected_drone = combine_platform_sortie_selections(solution, h_launch, h_recover, "drone", drone_sorties)
    selected_robot = combine_platform_sortie_selections(solution, g_launch, g_recover, "robot", robot_sorties)
    truck_routes = build_truck_routes(selected_truck, T, start_depot, end_depot)
    selected_drone, drone_audit_rows = audit_selected_ordered_sorties(
        selected_drone,
        "drone",
        selected_truck,
        solution,
        a_t,
        drone_time,
        distance,
        service_time,
        open_time,
        close_time,
    )
    selected_robot, robot_audit_rows = audit_selected_ordered_sorties(
        selected_robot,
        "robot",
        selected_truck,
        solution,
        a_t,
        robot_time,
        robot_distance,
        service_time,
        open_time,
        close_time,
    )
    sortie_timing_audit = pd.concat([drone_audit_rows, robot_audit_rows], ignore_index=True)
    platform_node_timing = pd.concat(
        [
            build_platform_node_timing(selected_drone, sortie_timing_audit, "drone"),
            build_platform_node_timing(selected_robot, sortie_timing_audit, "robot"),
        ],
        ignore_index=True,
    )
    if sortie_timing_audit.empty:
        sortie_timing_audit = pd.DataFrame(
            [
                {
                    "platform": "(none)",
                    "vehicle": "",
                    "sortie_id": "",
                    "truck": "",
                    "launch_truck": "",
                    "recovery_truck": "",
                    "route": "",
                    "leg": "",
                    "travel_duration_hr": 0.0,
                    "arrival_before_wait_hr": 0.0,
                    "wait_hr": 0.0,
                    "service_start_hr": 0.0,
                    "service_duration_hr": 0.0,
                    "service_finish_hr": 0.0,
                }
            ]
        )
    if platform_node_timing.empty:
        platform_node_timing = pd.DataFrame(
            [
                {
                    "platform": "(none)",
                    "vehicle": "",
                    "sortie_id": "",
                    "truck": "",
                    "launch_truck": "",
                    "recovery_truck": "",
                    "route": "",
                    "node_id": "",
                    "node_role": "",
                    "previous_node": "",
                    "next_node": "",
                    "arrival_before_wait_hr": 0.0,
                    "wait_hr": 0.0,
                    "service_start_hr": 0.0,
                    "service_duration_hr": 0.0,
                    "service_finish_hr": 0.0,
                    "truck_launch_time_hr": 0.0,
                    "truck_recovery_time_hr": 0.0,
                    "recovery_slack_hr": 0.0,
                }
            ]
        )

    # Physical timing audit: model arrival vs earliest physically required time.
    recovery_events: dict[tuple[int, int], float] = {}
    for platform_df in (selected_drone, selected_robot):
        if not platform_df.empty and "recovery_truck" in platform_df.columns and "platform_finish_time_hr" in platform_df.columns:
            for prow in platform_df.itertuples(index=False):
                key = (int(prow.recovery_truck), int(prow.k))
                finish = float(prow.platform_finish_time_hr)
                recovery_events[key] = max(recovery_events.get(key, 0.0), finish)
    truck_physical_timing = build_truck_physical_timing(
        selected_truck, T, start_depot, end_depot, solution, a_t,
        truck_time, open_time, service_time, recovery_events,
    )
    if truck_physical_timing.empty:
        truck_physical_timing = pd.DataFrame(
            [{"truck": "(none)", "from_node": "", "node_id": "", "travel_time_hr": "",
              "earliest_physical_arrival_hr": "", "window_open_hr": "",
              "platform_recovery_ready_hr": "", "earliest_required_time_hr": "",
              "model_arrival_time_hr": "", "slack_hr": "", "note": ""}]
        )

    reported_makespan_hr = 0.0
    if solution is not None:
        for v in T:
            if value_or_zero(solution, used_truck[v]) > 0.5:
                reported_makespan_hr = max(reported_makespan_hr, value_or_zero(solution, a_t[v, end_depot]))
        for platform_df in (selected_drone, selected_robot):
            if not platform_df.empty and "platform_finish_time_hr" in platform_df.columns:
                reported_makespan_hr = max(
                    reported_makespan_hr,
                    max(float(value) for value in platform_df["platform_finish_time_hr"].fillna(0.0)),
                )

    assignment_rows = []
    for i in C:
        truck_hits = [
            f"truck {row.truck}: visit {i}"
            for row in selected_rows(solution, visit, ["truck", "customer"]).itertuples(index=False)
            if int(row.customer) == i
        ]
        drone_hits = [
            f"drone {row.drone}: {row.route} (truck {row.launch_truck}->{row.recovery_truck})"
            for row in selected_drone.itertuples(index=False)
            if i in row.sequence
        ]
        robot_hits = [
            f"robot {row.robot}: {row.route} (truck {row.launch_truck}->{row.recovery_truck})"
            for row in selected_robot.itertuples(index=False)
            if i in row.sequence
        ]
        if truck_hits:
            mode = "truck"
            detail = "; ".join(truck_hits)
        elif drone_hits:
            mode = "drone"
            detail = "; ".join(drone_hits)
        elif robot_hits:
            mode = "robot"
            detail = "; ".join(robot_hits)
        else:
            mode = "unassigned"
            detail = ""
        assignment_rows.append({"customer_id": i, "assigned_mode": mode, "assignment_detail": detail})
    customer_assignment = pd.DataFrame(assignment_rows)

    # Arrival-time report. Arrival variables of unused trucks (and of nodes a
    # used truck never visits) are unconstrained in the MILP, so the solver may
    # leave arbitrary values there (e.g. 10010). Those values are meaningless;
    # report them as blank with an explicit status instead of confusing numbers.
    arrival_rows = []
    truck_is_used = {v: value_or_zero(solution, used_truck[v]) > 0.5 for v in T}
    truck_visits = {(v, i): value_or_zero(solution, visit[v, i]) > 0.5 for v in T for i in C}
    for v in T:
        for i in V:
            if not truck_is_used[v]:
                status = "unused_truck"
            elif i in (start_depot, end_depot) or truck_visits.get((v, i), False):
                status = "visited"
            else:
                status = "not_visited"
            arrival_rows.append(
                {
                    "mode": "truck",
                    "vehicle": v,
                    "node_id": i,
                    "status": status,
                    "arrival_time": value_or_zero(solution, a_t[v, i]) if status == "visited" else "",
                }
            )
    # Drone/robot arrival variables for a customer the platform never serves are
    # only loosely bounded (>= 0, <= close_time + late + big_M), so the solver
    # can park them at an arbitrary value in that range (often open_time or
    # close_time). Flag those rows so they are not read as real visit times,
    # mirroring the truck "status" column above.
    for i in V:
        if i in (start_depot, end_depot):
            drone_status, robot_status = "depot", "depot"
        else:
            drone_status = "served" if value_or_zero(solution, drone_service[i]) > 0.5 else "not_served_value_arbitrary"
            robot_status = "served" if value_or_zero(solution, robot_service[i]) > 0.5 else "not_served_value_arbitrary"
        arrival_rows.append({"mode": "drone", "vehicle": "", "node_id": i, "status": drone_status, "arrival_time": value_or_zero(solution, a_d[i])})
        arrival_rows.append({"mode": "robot", "vehicle": "", "node_id": i, "status": robot_status, "arrival_time": value_or_zero(solution, a_r[i])})

    penalty_vars = {}
    for label, var_dict in [
        ("late_t", late_t),
        ("late_d", late_d),
        ("late_r", late_r),
        ("route_late_t", route_late_t),
        ("route_late_global", {"global": route_late_global}),
    ]:
        for key, var in var_dict.items():
            val = value_or_zero(solution, var)
            if val > 1e-6:
                penalty_vars[f"{label}[{key}]"] = val

    kpi_rows = [
        {"kpi": "Solve Status", "value": solve_status},
        {"kpi": "Objective Value", "value": objective_value},
        {"kpi": "Runtime Seconds", "value": runtime},
        {"kpi": "Truck Only Mode", "value": int(TRUCK_ONLY)},
        {"kpi": "Truck Warm Start", "value": int(truck_warm_start_added)},
        {"kpi": "Drones In Model", "value": len(D)},
        {"kpi": "Robots In Model", "value": len(R)},
        {"kpi": "Reported Makespan", "value": reported_makespan_hr},
    ]
    if solve_exception:
        kpi_rows.append({"kpi": "Solve Exception", "value": repr(solve_exception)})
    kpi_rows.extend({"kpi": name, "value": value_or_zero(solution, expr)} for name, expr in kpi_exprs.items())

    operational_result = {
        "model_version": "ordered_multi_customer_sorties",
        "solve_status": solve_status,
        "objective": objective_value,
        "truck_arcs": len(selected_truck),
        "drone_sorties": len(selected_drone),
        "robot_sorties": len(selected_robot),
        "truck_customers": int((customer_assignment["assigned_mode"] == "truck").sum()),
        "drone_customers": int((customer_assignment["assigned_mode"] == "drone").sum()),
        "robot_customers": int((customer_assignment["assigned_mode"] == "robot").sum()),
        "feasible_drone_sequences": len(drone_sorties),
        "feasible_robot_sequences": len(robot_sorties),
        "reported_makespan_hr": reported_makespan_hr,
        "route_duration_excess": value_or_zero(solution, route_late_global),
        "total_penalty": value_or_zero(solution, total_penalty),
        "runtime": runtime,
    }
    write_notes(big_m, operational_result)

    parameter_rows: list[dict] = []
    flatten_json("", params, parameter_rows)
    summary_rows: list[dict] = []
    flatten_json("", instance_summary, summary_rows)
    notes_rows = [{"note": line} for line in NOTES_PATH.read_text(encoding="utf-8").splitlines()]
    penalties_df = pd.DataFrame([{"penalty_variable": key, "value": val} for key, val in penalty_vars.items()])
    if penalties_df.empty:
        penalties_df = pd.DataFrame([{"penalty_variable": "(none)", "value": 0.0}])

    with pd.ExcelWriter(EXCEL_PATH, engine="openpyxl") as writer:
        pd.DataFrame(kpi_rows).to_excel(writer, sheet_name="KPIs", index=False)
        selected_truck.to_excel(writer, sheet_name="Truck Arcs", index=False)
        truck_routes.to_excel(writer, sheet_name="Truck Routes", index=False)
        selected_drone.to_excel(writer, sheet_name="Drone Sorties", index=False)
        selected_robot.to_excel(writer, sheet_name="Robot Sorties", index=False)
        sortie_timing_audit.to_excel(writer, sheet_name="Sortie Timing Audit", index=False)
        platform_node_timing.to_excel(writer, sheet_name="Platform Node Timing", index=False)
        customer_assignment.to_excel(writer, sheet_name="Customer Assignment", index=False)
        pd.DataFrame(arrival_rows).to_excel(writer, sheet_name="Arrival Times", index=False)
        truck_physical_timing.to_excel(writer, sheet_name="Truck Physical Timing", index=False)
        penalties_df.to_excel(writer, sheet_name="Penalties", index=False)
        customers.to_excel(writer, sheet_name="Input Customers", index=False)
        arcs.to_excel(writer, sheet_name="Input Arcs", index=False)
        pd.DataFrame(parameter_rows).to_excel(writer, sheet_name="Parameters", index=False)
        pd.DataFrame(summary_rows).to_excel(writer, sheet_name="Instance Summary", index=False)
        pd.DataFrame(notes_rows).to_excel(writer, sheet_name="Model Notes", index=False)

    print("\nOperational solve summary")
    print("-" * 60)
    print(f"Solve status: {solve_status}")
    if solve_exception:
        print(f"Solve exception: {solve_exception!r}")
    print(f"Objective value: {objective_value}")
    print(f"Runtime seconds: {runtime:.4f}")
    print("\nKPI values")
    for row in kpi_rows:
        print(f"{row['kpi']}: {row['value']}")
    print("\nSelected truck route arcs")
    print(selected_truck.to_string(index=False) if not selected_truck.empty else "(none)")
    print("\nReconstructed truck routes")
    print(truck_routes.to_string(index=False))
    print("\nSelected drone sorties")
    print(selected_drone.to_string(index=False) if not selected_drone.empty else "(none)")
    print("\nSelected robot sorties")
    print(selected_robot.to_string(index=False) if not selected_robot.empty else "(none)")
    print("\nSortie timing audit")
    print(sortie_timing_audit.to_string(index=False))
    print("\nCustomer assignment")
    print(customer_assignment.to_string(index=False))
    print("\nArrival times")
    print(pd.DataFrame(arrival_rows).to_string(index=False))
    print("\nPositive penalty variables")
    if penalty_vars:
        for key, val in penalty_vars.items():
            print(f"{key}: {val:.6f}")
    else:
        print("(none)")
    print("\nWarnings / modeling notes")
    print(f"- Operational constraints are route-completeness constraints, not explicit PDF constraints.")
    print(f"- Flexible docking uses h_launch/h_recover and g_launch/g_recover.")
    print(f"- Big-M used: {big_m}")
    print(f"\nExcel exported to: {EXCEL_PATH}")
    print(f"LP file: {exported_lp if exported_lp else 'not exported'}")
    print(f"Operational notes: {NOTES_PATH}")

    if solution is None:
        INFEASIBILITY_NOTES_PATH.write_text(
            "Operational model did not produce a feasible solution.\n"
            f"Solve status: {solve_status}\n"
            f"LP file: {exported_lp if exported_lp else 'not exported'}\\n"
            f"Solve exception: {repr(solve_exception)}\n",
            encoding="utf-8",
        )
        print(f"Infeasibility notes: {INFEASIBILITY_NOTES_PATH}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
