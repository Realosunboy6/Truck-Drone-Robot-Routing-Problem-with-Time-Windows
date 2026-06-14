from __future__ import annotations

import json
import os
import re
from pathlib import Path

import pandas as pd


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
RESULTS_ROOT = PACKAGE_ROOT / "results" / os.environ.get("DRT_BATCH_RESULTS_SUBDIR", "rerun_30min")
DATA_ROOT = PACKAGE_ROOT / "data_processed" / "tdrp_tw_literature_params"
RUN_TAG = os.environ.get("DRT_RUN_TAG", "github_final_30min_3_10_no_cmax_arrival_status")


def case_key(path: Path) -> tuple[int, int]:
    match = re.search(r"tdrp_(\d+)_(\d+)_", path.name)
    if not match:
        return (10**9, 10**9)
    return int(match.group(1)), int(match.group(2))


def read_kpis(xl: pd.ExcelFile) -> dict[str, object]:
    df = pd.read_excel(xl, "KPIs")
    return {str(row["kpi"]): row.get("value") for _, row in df.iterrows()}


def selected_routes(xl: pd.ExcelFile, sheet_name: str, vehicle_col: str) -> str:
    df = pd.read_excel(xl, sheet_name)
    if df.empty or "route" not in df.columns:
        return ""
    if "value" in df.columns:
        df = df[df["value"].fillna(0) > 0.5]
    routes = []
    for row in df.itertuples(index=False):
        truck_part = ""
        if hasattr(row, "launch_truck") and hasattr(row, "recovery_truck"):
            truck_part = f"truck={int(row.launch_truck)}->{int(row.recovery_truck)}"
        parts = [
            f"{vehicle_col} {getattr(row, vehicle_col)}: {row.route}",
            truck_part,
            f"customers={getattr(row, 'customer_count', '')}",
            f"payload={getattr(row, 'payload', '')}",
            f"distance={getattr(row, 'travel_distance', '')}",
            f"finish_hr={getattr(row, 'platform_finish_time_hr', '')}",
            f"slack_hr={getattr(row, 'recovery_slack_hr', '')}",
        ]
        parts = [part for part in parts if part]
        routes.append(" (".join([parts[0], ", ".join(parts[1:]) + ")"]))
    return " | ".join(routes)


def audit_platform_truck_limits(
    issues: list[dict[str, object]],
    df: pd.DataFrame,
    case: str,
    path: Path,
    platform: str,
    vehicle_col: str,
    max_per_truck: int,
    carried_at_depot: int,
) -> None:
    if {"launch_truck", "recovery_truck", vehicle_col, "i", "k"}.issubset(df.columns):
        launch_events = df[["launch_truck", vehicle_col, "i"]].rename(
            columns={"launch_truck": "truck", "i": "node"}
        )
        launch_events["event"] = "launch"
        recovery_events = df[["recovery_truck", vehicle_col, "k"]].rename(
            columns={"recovery_truck": "truck", "k": "node"}
        )
        recovery_events["event"] = "recovery"
        interactions = pd.concat([launch_events, recovery_events], ignore_index=True)
        distinct_by_truck = interactions.groupby("truck")[vehicle_col].nunique().reset_index(name=f"distinct_{platform}s")
        for row in distinct_by_truck[distinct_by_truck[f"distinct_{platform}s"] > max_per_truck].itertuples(index=False):
            issues.append(
                {
                    "case": case,
                    "issue": f"truck_interacts_with_more_distinct_{platform}s_than_max",
                    "value": int(getattr(row, f"distinct_{platform}s")),
                    "limit": max_per_truck,
                    "truck": int(row.truck),
                    "file": str(path),
                }
            )
        for row in launch_events.groupby(["truck", "node"])[vehicle_col].nunique().reset_index(name="count").itertuples(index=False):
            limit = carried_at_depot if int(row.node) == 0 else max_per_truck
            if int(row.count) > limit:
                issues.append(
                    {
                        "case": case,
                        "issue": f"truck_launches_more_{platform}s_from_node_than_limit",
                        "value": int(row.count),
                        "limit": limit,
                        "truck": int(row.truck),
                        "node": int(row.node),
                        "file": str(path),
                    }
                )
        for row in recovery_events.groupby(["truck", "node"])[vehicle_col].nunique().reset_index(name="count").itertuples(index=False):
            if int(row.count) > max_per_truck:
                issues.append(
                    {
                        "case": case,
                        "issue": f"truck_recovers_more_{platform}s_at_node_than_limit",
                        "value": int(row.count),
                        "limit": max_per_truck,
                        "truck": int(row.truck),
                        "node": int(row.node),
                        "file": str(path),
                    }
                )
        return

    if {"truck", vehicle_col, "i"}.issubset(df.columns):
        distinct_by_truck = df.groupby("truck")[vehicle_col].nunique().reset_index(name=f"distinct_{platform}s")
        for row in distinct_by_truck[distinct_by_truck[f"distinct_{platform}s"] > max_per_truck].itertuples(index=False):
            issues.append(
                {
                    "case": case,
                    "issue": f"truck_uses_more_distinct_{platform}s_than_max",
                    "value": int(getattr(row, f"distinct_{platform}s")),
                    "limit": max_per_truck,
                    "truck": int(row.truck),
                    "file": str(path),
                }
            )
        launches_by_node = df.groupby(["truck", "i"])[vehicle_col].nunique().reset_index(name="count")
        for row in launches_by_node.itertuples(index=False):
            limit = carried_at_depot if int(row.i) == 0 else max_per_truck
            if int(row.count) > limit:
                issues.append(
                    {
                        "case": case,
                        "issue": f"truck_launches_more_{platform}s_from_node_than_limit",
                        "value": int(row.count),
                        "limit": limit,
                        "truck": int(row.truck),
                        "node": int(row.i),
                        "file": str(path),
                    }
                )


def audit_solution(path: Path, xl: pd.ExcelFile, case: str) -> list[dict[str, object]]:
    issues: list[dict[str, object]] = []
    params_path = DATA_ROOT / case / "parameters.json"
    params = json.loads(params_path.read_text(encoding="utf-8-sig")) if params_path.exists() else {}
    max_drones_per_truck = int(params.get("MAX_DRONES_PER_TRUCK", 10**9))
    drones_carried_at_depot = int(params.get("DRONES_CARRIED_AT_DEPOT", 10**9))
    max_robots_per_truck = int(params.get("MAX_ROBOTS_PER_TRUCK", 10**9))
    robots_carried_at_depot = int(params.get("ROBOTS_CARRIED_AT_DEPOT", 10**9))
    limits = {
        "drone": {"payload": 25.0, "distance": 20.0, "sheet": "Drone Sorties", "vehicle": "drone"},
        "robot": {"payload": 20.0, "distance": 15.0, "sheet": "Robot Sorties", "vehicle": "robot"},
    }
    for platform, cfg in limits.items():
        df = pd.read_excel(xl, cfg["sheet"])
        if df.empty or cfg["vehicle"] not in df.columns:
            continue
        if "value" in df.columns:
            df = df[df["value"].fillna(0) > 0.5]
        if df.empty:
            continue
        if "payload" in df.columns:
            for row in df[df["payload"] > cfg["payload"] + 1e-6].itertuples(index=False):
                issues.append({"case": case, "issue": f"{platform}_payload", "value": row.payload, "file": str(path)})
        if "travel_distance" in df.columns:
            for row in df[df["travel_distance"] > cfg["distance"] + 1e-6].itertuples(index=False):
                issues.append({"case": case, "issue": f"{platform}_distance", "value": row.travel_distance, "file": str(path)})
        if {"i", "k"}.issubset(df.columns):
            grouped = df.groupby([cfg["vehicle"], "i", "k"]).size().reset_index(name="count")
            for row in grouped[grouped["count"] > 1].itertuples(index=False):
                issues.append(
                    {
                        "case": case,
                        "issue": f"{platform}_duplicate_same_vehicle_same_leg",
                        "value": int(row.count),
                        "file": str(path),
                    }
                )
        if platform == "drone":
            audit_platform_truck_limits(
                issues, df, case, path, "drone", "drone", max_drones_per_truck, drones_carried_at_depot
            )
        if platform == "robot":
            audit_platform_truck_limits(
                issues, df, case, path, "robot", "robot", max_robots_per_truck, robots_carried_at_depot
            )
    return issues


def main() -> int:
    rows: list[dict[str, object]] = []
    issues: list[dict[str, object]] = []
    for path in sorted(RESULTS_ROOT.glob("*_solution.xlsx"), key=case_key):
        n, pct = case_key(path)
        case = f"{n}-{pct}"
        xl = pd.ExcelFile(path)
        kpis = read_kpis(xl)
        truck_routes = pd.read_excel(xl, "Truck Routes")
        used_trucks = truck_routes[truck_routes["status"].astype(str) == "used"]
        rows.append(
            {
                "case": case,
                "solve_status": kpis.get("Solve Status", ""),
                "objective": kpis.get("Objective Value", ""),
                "runtime_seconds": kpis.get("Runtime Seconds", ""),
                "used_trucks": kpis.get("Used Trucks", ""),
                "operating_cost": kpis.get("Operating Cost", ""),
                "reported_makespan": kpis.get("Reported Makespan", ""),
                "route_duration_excess": kpis.get("Route Duration Excess", ""),
                "total_penalty": kpis.get("Total Penalty", ""),
                "route_duration_penalty": kpis.get("Route Duration Penalty", ""),
                "time_window_penalty": kpis.get("Time Window Penalty", ""),
                "truck_routes": " | ".join(f"truck {int(row.truck)}: {row.route}" for row in used_trucks.itertuples(index=False)),
                "drone_routes": selected_routes(xl, "Drone Sorties", "drone"),
                "robot_routes": selected_routes(xl, "Robot Sorties", "robot"),
            }
        )
        issues.extend(audit_solution(path, xl, case))

    pd.DataFrame(rows).to_csv(RESULTS_ROOT / f"{RUN_TAG}_detailed_summary.csv", index=False)
    pd.DataFrame(issues).to_csv(RESULTS_ROOT / f"{RUN_TAG}_constraint_audit.csv", index=False)
    print(f"Saved {RESULTS_ROOT / f'{RUN_TAG}_detailed_summary.csv'}")
    print(f"Saved {RESULTS_ROOT / f'{RUN_TAG}_constraint_audit.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
