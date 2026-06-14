# Truck-Drone-Robot Routing Problem with Time Windows

This folder contains the report-facing version of the strict truck-drone-robot model.

## Folder Structure

- `data_raw/tdrp_tw`: raw TDRP-TW benchmark files and source metadata.
- `data_processed/tdrp_tw_literature_params`: processed small benchmark cases using the literature-based platform parameters.
- `model/capped_flexible_docking_ordered_sortie_model.py`: capped flexible-docking ordered-sortie model with environment-variable inputs for one case or batch runs.
- `scripts/build_tdrp_literature_instance.py`: script used to build processed case data.
- `scripts/run_all_strict_cases.py`: reruns all processed strict-model cases and saves outputs under `results`.
- `scripts/summarize_strict_results.py`: summarizes the saved Excel workbooks and checks payload, distance, and duplicate sortie issues.
- `pdf`: final code-faithful formulation PDF/TEX and the reference math-model PDF used for traceability.
- `results`: organized output folder containing the official 5-minute and 30-minute capped-model runs.

## Model Version

The report model is the capped flexible-docking ordered-sortie MILP. A selected drone or robot variable represents the full path, for example:

```text
5 -> 2 -> 6 -> 7
```

This directly answers the route-confirmation question: the output identifies the launch node, each served customer in order, and the recovery node.

The processed customer files contain only:

```text
customer_id, demand, open_time, close_time, service_time
```

They do not contain truck/drone/robot eligibility columns. Customer assignment is decided by the optimization model using capacity, endurance, timing, and synchronization constraints.

## Physical-Platform Consistency

Each physical drone or robot performs at most one sortie per solution (`one_sortie_per_drone` / `one_sortie_per_robot`). This guarantees that no platform appears in two places at once and removes the need for inter-sortie sequencing or onboard platform-flow tracking. The processed cases carry more drones and robots than useful sorties, so this constraint does not restrict the solution space in these experiments.

The model instantiates at most `MAX_DRONES_PER_TRUCK x NUM_TRUCKS` drones and `MAX_ROBOTS_PER_TRUCK x NUM_TRUCKS` robots. The distinct-platform caps make any additional identical platforms provably unusable, so this trimming shrinks the MILP without changing any optimal solution. The KPIs sheet reports the trimmed fleet sizes (`Drones In Model`, `Robots In Model`).

Endurance is enforced per sortie: candidate generation excludes any ordered route whose total distance exceeds the platform limit, and the MILP restates the per-sortie limit explicitly. Truck capacity and platform endurance are hard constraints with no penalty terms; only time-window and route-duration violations are soft penalties in the objective.

Because sortie generation is capped, an `integer optimal solution` status means optimal over the retained candidate-sortie pool, not over every possible ordered sortie in the unrestricted problem. The reported runs use `MAX_CUSTOMERS_PER_SORTIE=3` and `TOP_SORTIES_PER_TRUCK_LEG=10`.

## Objective Costs

The operating cost combines variable travel costs with fixed activation costs from the VRP-DR paper (Malik et al., arXiv:2505.23584, Table 3): 30 per used truck, 10 per selected drone sortie, and 8 per selected robot sortie. Fixed costs are read from `parameters.json` keys `truck_fixed_cost`, `drone_fixed_cost`, and `robot_fixed_cost`, with the paper defaults applied when the keys are missing. The KPIs sheet in each solution workbook reports the full breakdown: variable and fixed cost per platform, totals, penalties, and objective value.

The model also includes symmetry-breaking constraints (platform k+1 may fly a sortie only if platform k does), which are valid because platforms are identical and each is limited to one sortie.

## Parameter Usage Audit

The raw TDRP-TW files contain fields that are kept for traceability or used only during preprocessing. The solve model does not use customer-type counts to preassign service modes; truck, drone, and robot assignment is decided by the MILP through capacity, endurance, timing, synchronization, and cost.

The file `parameter_usage_audit.csv` lists every key in each case's `parameters.json` and labels it as active in the solve model, builder-only, metadata-only, or unused raw/PDF data. Reproduced below for the report:

| Status | Parameters | Why |
| --- | --- | --- |
| Active in model | `C_veh`, `C_drone`, `C_rob`, `C_w`, `C_w_drone`, `C_w_r`, `Q_t`, `Q_d`, `Q_r`, `E_d`, `E_r`, `T_max`, `lambda_T`, `lambda_W`, `NUM_TRUCKS`, `NUM_DRONES`, `NUM_ROBOTS`, `MAX_DRONES_PER_TRUCK`, `MAX_ROBOTS_PER_TRUCK`, `DRONES_CARRIED_AT_DEPOT`, `ROBOTS_CARRIED_AT_DEPOT`, `truck_fixed_cost`, `drone_fixed_cost`, `robot_fixed_cost` | Read directly by the solve model; each affects a constraint, cost term, objective term, or fleet limit. |
| Unused raw/PDF fields | `waiting_penalty_weight`, `big_M`, `truck_variable_cost`, `drone_departure_cost`, `lambda_E_d`, `lambda_E_r`, `lambda_Q` | Retained only for traceability. `waiting_penalty_weight` is a raw TDRP-TW waiting-cost field and is not `lambda_W`; `lambda_W` is the active time-window lateness penalty. `big_M` is the raw source value, while the model computes its own safe internal Big-M. `truck_variable_cost` and `drone_departure_cost` are raw source costs superseded by the literature cost coefficients and fixed costs. `lambda_E_d`, `lambda_E_r`, and `lambda_Q` are unused because endurance and capacity are hard constraints. |
| Builder-only | `truck_speed`, `drone_speed`, `robot_speed` | Used by `build_tdrp_literature_instance.py` to build the time matrices; the solve model reads the matrices, not these keys. |
| Metadata only | `NUM_ARCS`, `NUM_CUSTOMERS`, `NUM_NODES`, `format`, `parameter_scenario`, `parameter_source`, `parameter_source_arxiv`, `parsed_at`, `selected_raw_file`, `source_dataset`, `source_instance`, `robot_fleet_assumption`, `assumptions`, `warnings` | Traceability/reporting fields only; do not affect optimization. |

The active soft penalties are `lambda_T` (route-duration excess against `T_max`) and `lambda_W` (time-window lateness). Capacity and endurance are hard constraints.

**Fix (2026-06-13):** `C_w_drone` was previously misclassified as unused. The drone time/waiting cost term used `C_w` (the truck coefficient) instead. The cost expressions now read `C_w`, `C_w_drone`, and `C_w_r` separately for truck, drone, and robot respectively:

```text
truck cost = truck_time * C_w   + distance        * C_veh
drone cost = drone_time * C_w_drone + distance    * C_drone
robot cost = robot_time * C_w_r + robot_distance  * C_rob
```

Both `C_w` and `C_w_drone` are `0.0` in the current literature-parameter data, so this fix does not change any objective values already reported. It only removes a latent inconsistency that would matter if a future scenario sets `C_w_drone != C_w`.

## Flexible Truck-Platform Synchronization

The final model explicitly separates launch-truck and recovery-truck decisions:

```text
h_launch[v,d,s] = 1 if truck v launches drone d for drone sortie s
h_recover[v,d,s] = 1 if truck v recovers drone d for drone sortie s
g_launch[v,r,s] = 1 if truck v launches robot r for robot sortie s
g_recover[v,r,s] = 1 if truck v recovers robot r for robot sortie s
```

The launch truck and recovery truck can be the same truck or different trucks. This lets a drone or robot synchronize with any truck in the system, as long as the launch truck reaches the launch node and the recovery truck reaches the recovery node on time.

The processed data keeps robot fleet parameters separate from drone parameters:

```text
NUM_ROBOTS
ROBOTS_CARRIED_AT_DEPOT
MAX_ROBOTS_PER_TRUCK
```

For the current experiments, robot-specific benchmark data is not available, so the robot fleet values are set separately as experimental placeholders:

```text
NUM_ROBOTS: NUM_CUSTOMERS + NUM_TRUCKS
ROBOTS_CARRIED_AT_DEPOT: 1
MAX_ROBOTS_PER_TRUCK: 2
```

They are intentionally separate fields so real robot-specific data can be inserted later without changing the formulation.

## Truck Route Duration

`T_max` (10 h in the TDRP-TW data) is applied as a truck route-duration limit from the fixed depot departure time. In this package, every truck departs the start depot at clock time 0. Route-duration penalties therefore apply when a used truck reaches the end depot after `T_max`.

Some TDRP-TW customer time windows open much later than 10 h, with several small cases having service windows in the 20-40 h horizon. Under the fixed-departure interpretation, those late windows can make the route-duration penalty dominate the objective even when all service time windows and synchronization constraints are satisfied. This is a data/formulation interaction, not a capacity or synchronization violation; report tables should therefore show operating cost, route-duration penalty, and total penalty separately.

The original formulation writes the route-duration condition for trucks, drones, and robots. In this implementation, the penalty is applied through the truck end-depot arrival because drone/robot recovery is synchronized with a truck: if a platform finishes late, the recovering truck must wait for it, and that delay propagates to the truck's end-depot time and route-duration penalty.

## Timing Verification

Each solution workbook includes:

- `Sortie Timing Audit`: leg-by-leg travel, waiting, service start, service finish.
- `Platform Node Timing`: node-by-node drone/robot timing for launch, customer service, and recovery.
- `Arrival Times`: truck and platform arrival variables. For drone/robot rows, only `served` and `depot` statuses are real timing records; `not_served_value_arbitrary` means the platform did not serve that customer and the displayed variable value should be ignored.

Use `Platform Node Timing` to explain synchronization:

```text
platform_finish_time_hr <= truck_recovery_time_hr
```

The `recovery_slack_hr` column shows how much time remains when the drone or robot reaches the recovery node before the truck.

## Results Included in This Package

The `results` folder is organized as:

```text
results/
  README.md
  final_5min/
    workbooks/
    summaries/
    lp_models/
    logs/
    model_notes/
    batch_status/
  final_30min/
    workbooks/
    summaries/
    lp_models/
    logs/
    model_notes/
    infeasibility_notes/
    batch_status/
```

The 5-minute run used a 300-second limit per case. The 30-minute run used an 1800-second limit per case. Both official runs used:

```text
max customers per sortie = 3
top sorties per truck leg = 10
LP export = on
fast build = off
truck-only warm start = on
```

The most useful summary files are:

- `results/final_5min/summaries/customer_windows_service_assignment_audit.csv`
- `results/final_5min/summaries/case_status_cost_summary.csv`
- `results/final_30min/summaries/case_status_cost_summary_30min.csv`
- `results/final_30min/summaries/customer_windows_service_assignment_audit_30min.csv`

The exported `.lp` files are included under each run's `lp_models/` folder when they are small enough for GitHub. The two `11-25` LP exports exceed GitHub's 100 MB file-size limit, so they are kept local-only and ignored by Git.

## Running All Capped Flexible-Docking Cases

From this folder:

```powershell
.\run_all_strict_cases.ps1
```

By default, reruns are written to `results/rerun_30min` so the official `final_5min` and `final_30min` results are not overwritten.

## Requirements

Python 3.10 with CPLEX installed. Python packages are listed in `requirements.txt`:

```powershell
py -3.10 -m pip install -r requirements.txt
```

The model needs a working IBM CPLEX installation (the `cplex` and `docplex` packages must match your CPLEX version).

## Benchmark Data Attribution

The raw benchmark instances in `data_raw/tdrp_tw` come from the TDRP-TW dataset:

> Li, Hongqi (2021). "TDRP-TW instances." Mendeley Data, V1. DOI: [10.17632/tn4hkfrn9w.1](https://doi.org/10.17632/tn4hkfrn9w.1). Licensed under [CC BY 4.0](http://creativecommons.org/licenses/by/4.0).

The processed cases in `data_processed/tdrp_tw_literature_params` are derived from this dataset with literature-based platform parameters (VRP-DR paper Table 3, arXiv:2505.23584) applied. Robot fleet values are experimental placeholders as described above.
