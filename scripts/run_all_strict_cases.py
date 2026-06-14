from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pandas as pd


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DATA_ROOT = PACKAGE_ROOT / "data_processed" / "tdrp_tw_literature_params"
RESULTS_ROOT = PACKAGE_ROOT / "results" / os.environ.get("DRT_BATCH_RESULTS_SUBDIR", "rerun_30min")
MODEL_PATH = PACKAGE_ROOT / "model" / "capped_flexible_docking_ordered_sortie_model.py"
RUN_TAG = os.environ.get("DRT_RUN_TAG", "github_final_30min_3_10_no_cmax_arrival_status")
TIME_LIMIT_SECONDS = os.environ.get("DRT_TIME_LIMIT_SECONDS", "1800")
MAX_CUSTOMERS_PER_SORTIE = os.environ.get("DRT_MAX_CUSTOMERS_PER_SORTIE", "3")
TOP_SORTIES_PER_TRUCK_LEG = os.environ.get("DRT_TOP_SORTIES_PER_TRUCK_LEG", "10")
EXPORT_LP = os.environ.get("DRT_EXPORT_LP", "1")
FAST_BUILD = os.environ.get("DRT_FAST_BUILD", "0")
USE_TRUCK_WARM_START = os.environ.get("DRT_USE_TRUCK_WARM_START", "1")


def case_sort_key(path: Path) -> tuple[int, int]:
    n, pct = path.name.split("-")
    return int(n), int(pct)


def read_kpis(excel_path: Path) -> dict[str, object]:
    if not excel_path.exists():
        return {}
    df = pd.read_excel(excel_path, "KPIs")
    return {str(row["kpi"]): row.get("value") for _, row in df.iterrows()}


def audit_duplicate_platform_sorties(excel_path: Path) -> list[dict[str, object]]:
    issues: list[dict[str, object]] = []
    if not excel_path.exists():
        return issues
    xl = pd.ExcelFile(excel_path)
    for sheet, vehicle_col in [("Drone Sorties", "drone"), ("Robot Sorties", "robot")]:
        df = pd.read_excel(xl, sheet)
        if df.empty or vehicle_col not in df.columns or "i" not in df.columns or "k" not in df.columns:
            continue
        if "value" in df.columns:
            df = df[df["value"].fillna(0) > 0.5]
        grouped = df.groupby([vehicle_col, "i", "k"]).size().reset_index(name="count")
        for row in grouped[grouped["count"] > 1].itertuples(index=False):
            issues.append(
                {
                    "platform": sheet.replace(" Sorties", "").lower(),
                    "vehicle": getattr(row, vehicle_col),
                    "launch_node": int(row.i),
                    "recovery_node": int(row.k),
                    "selected_sorties_on_same_leg": int(row.count),
                }
            )
    return issues


def main() -> int:
    RESULTS_ROOT.mkdir(parents=True, exist_ok=True)
    cases = sorted([p for p in DATA_ROOT.iterdir() if p.is_dir()], key=case_sort_key)
    rows: list[dict[str, object]] = []
    duplicate_issues: list[dict[str, object]] = []

    for case_dir in cases:
        case = case_dir.name
        instance_tag = f"tdrp_{case.replace('-', '_')}"
        excel_path = RESULTS_ROOT / f"{instance_tag}_ordered_sorties_{RUN_TAG}_solution.xlsx"

        env = os.environ.copy()
        env["DRT_DATA_DIR"] = str(case_dir)
        env["DRT_RESULTS_DIR"] = str(RESULTS_ROOT)
        env["DRT_INSTANCE_TAG"] = instance_tag
        env["DRT_RUN_TAG"] = RUN_TAG
        env["DRT_TIME_LIMIT_SECONDS"] = TIME_LIMIT_SECONDS
        env["DRT_MAX_CUSTOMERS_PER_SORTIE"] = MAX_CUSTOMERS_PER_SORTIE
        env["DRT_TOP_SORTIES_PER_TRUCK_LEG"] = TOP_SORTIES_PER_TRUCK_LEG
        env["DRT_EXPORT_LP"] = EXPORT_LP
        env["DRT_FAST_BUILD"] = FAST_BUILD
        env["DRT_USE_TRUCK_WARM_START"] = USE_TRUCK_WARM_START

        print(f"\n=== Running capped flexible-docking model case {case} ===", flush=True)
        print(
            f"results={RESULTS_ROOT} tag={RUN_TAG} time_limit={TIME_LIMIT_SECONDS}s "
            f"pool={MAX_CUSTOMERS_PER_SORTIE}/{TOP_SORTIES_PER_TRUCK_LEG} "
            f"lp_export={EXPORT_LP} fast_build={FAST_BUILD} warm_start={USE_TRUCK_WARM_START}",
            flush=True,
        )
        completed = subprocess.run([sys.executable, str(MODEL_PATH)], env=env, cwd=str(PACKAGE_ROOT))

        kpis = read_kpis(excel_path)
        rows.append(
            {
                "case": case,
                "return_code": completed.returncode,
                "excel_path": str(excel_path),
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
            }
        )

        for issue in audit_duplicate_platform_sorties(excel_path):
            issue["case"] = case
            duplicate_issues.append(issue)

    summary = pd.DataFrame(rows)
    summary.to_csv(RESULTS_ROOT / f"{RUN_TAG}_run_summary.csv", index=False)
    pd.DataFrame(duplicate_issues).to_csv(RESULTS_ROOT / f"{RUN_TAG}_duplicate_sortie_audit.csv", index=False)
    print("\nSaved:")
    print(RESULTS_ROOT / f"{RUN_TAG}_run_summary.csv")
    print(RESULTS_ROOT / f"{RUN_TAG}_duplicate_sortie_audit.csv")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
