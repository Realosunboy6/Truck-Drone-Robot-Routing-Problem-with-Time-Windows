# Truck-Drone-Robot Routing Problem with Time Windows

This folder contains the report-facing version of the strict truck-drone-robot model.

## Folder Structure

- `data_raw/tdrp_tw`: raw TDRP-TW benchmark files and source metadata.
- `data_processed/tdrp_tw_literature_params`: processed small benchmark cases using the literature-based platform parameters.
- `data_processed/tdrp_tw_realistic_v1`: repaired benchmark set — all 75 TDRP-TW cases (25 instances x 3 drone-eligibility levels) rebuilt with realistic hardware parameters and a per-instance `T_max` set to the raw depot close time, so route-duration penalties only reflect genuine horizon violations. See "Realistic-Parameter Dataset (v1)" below.
- `model/capped_flexible_docking_ordered_sortie_model.py`: capped flexible-docking ordered-sortie model with environment-variable inputs for one case or batch runs.
- `scripts/build_tdrp_literature_instance.py`: script used to build processed case data (`--scenario literature` reproduces the original set; `--scenario realistic` builds the repaired v1 set).
- `scripts/build_real_world_instance.py`: builds brand-new instances from OpenStreetMap — truck travel on the OSM drive network, robot travel on the OSM walk network, drone travel as great-circle distance. See "Real-World Instance Generator" below.
- `scripts/validate_realistic_v1.py`: validates every case in a processed set (matrix dimensions, zero diagonals, finite/nonnegative values, `T_max` vs all time windows, provenance fields).
- `scripts/run_all_strict_cases.py`: reruns all processed strict-model cases and saves outputs under `results` (`DRT_DATA_ROOT` selects the processed set; failed cases are recorded with an `error` field instead of aborting the batch).
- `scripts/summarize_strict_results.py`: summarizes the saved Excel workbooks and checks payload, distance, and duplicate sortie issues (`DRT_DATA_ROOT` selects the processed set; audit limits are read from each case's `parameters.json`).
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

The cost expressions now read `C_w`, `C_w_drone`, and `C_w_r` separately for truck, drone, and robot respectively:

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

They are intentionally separate fields so real robot-specific data can be inserted later without changing the formulation. In the realistic v1 dataset and the real-world generator, robot physical parameters (speed, payload, endurance) are literature-based (see below); the placeholder status that remains is the robot travel matrix in the TDRP-derived set, which still reuses truck road distances until pedestrian-network data is wired in. The real-world generator already uses the OSM walk network for robots.

## Truck Route Duration

`T_max` is applied as a truck route-duration limit from the fixed depot departure time. In this package, every truck departs the start depot at clock time 0. Route-duration penalties therefore apply when a used truck reaches the end depot after `T_max`.

The original TDRP-TW processed set used a flat `T_max = 10 h` while several small cases have service windows in the 20-40 h horizon. Under the fixed-departure interpretation, those late windows can make the route-duration penalty dominate the objective even when all service time windows and synchronization constraints are satisfied. This is a data/formulation interaction, not a capacity or synchronization violation; report tables should therefore show operating cost, route-duration penalty, and total penalty separately.

The repaired `tdrp_tw_realistic_v1` dataset (see below) sets `T_max` per instance to the raw instance's depot close time, so route-duration penalties only trigger on genuine horizon violations.

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

## Realistic-Parameter Dataset (v1)

`data_processed/tdrp_tw_realistic_v1` repairs the two biggest data distortions in the original processed set:

1. **Per-instance `T_max`.** The flat 10 h horizon is replaced by each raw instance's depot close time (e.g. 20.6 h for case 11-25), so the `lambda_T = 1000` route-duration penalty no longer dominates the objective on late-window cases.
2. **Literature-based hardware.** Drone: 80 km/h, 5 kg payload, 40 km sortie range (Sacramento et al., 2019 — 50 mph, 30-minute endurance). Robot: 6 km/h, 10 kg payload, 12 km sortie range (Starship Gen 3; Ostermeier, 2021). Costs remain the VRP-DR Table 3 structure (Malik et al., arXiv:2505.23584).

The set covers all 75 TDRP-TW cases (25 instances x 25/50/75% drone-eligibility levels), each tagged with `parameter_scenario`, `parameter_source`, `T_max_source`, and per-case `assumptions`/`warnings` in `parameters.json`. Build it with:

```bash
python scripts/build_tdrp_literature_instance.py --scenario realistic
```

Validate it with:

```bash
python scripts/validate_realistic_v1.py data_processed/tdrp_tw_realistic_v1
```

Known remaining placeholder: the robot travel matrix still reuses truck road distances (the source TDRP-TW data has no robot layer). This is flagged in every case's `warnings` until pedestrian-network data is wired in — which is exactly what the real-world generator below does.

## Real-World Instance Generator

`scripts/build_real_world_instance.py` builds brand-new instances from OpenStreetMap instead of repairing benchmark data, so every travel layer is mode-realistic:

- **Truck:** shortest-path distances on the OSM *drive* network (45 km/h).
- **Robot:** shortest-path distances on the OSM *walk* (pedestrian) network (6 km/h).
- **Drone:** haversine great-circle distances (80 km/h, unrestricted airspace).

Customers are sampled from the pedestrian network with a minimum separation; the depot is the road-network node nearest the customer centroid. Demands follow the Amazon parcel rule used across the literature (~86% of parcels under 5 lbs / 2.27 kg, drone/robot eligible; the rest heavier truck-leaning parcels). Time windows sit inside an 8-hour working day (each customer gets a 2-hour window), and `T_max` equals the 8-hour horizon. Output matches the model's CSV/JSON schema exactly, with full provenance (`source_place`, `generator_seed`, network sizes, assumptions, warnings) in `parameters.json` and `instance_summary.json`.

Requires `osmnx` (`pip install osmnx`) and network access to OpenStreetMap's geocoder/download servers.

```bash
python scripts/build_real_world_instance.py \
    --place "DeKalb, Illinois, USA" --n-customers 25 --seed 7 \
    --out-dir data_processed/realworld_dekalb_il_25
```

(`--bbox north,south,east,west` and `--depot-latlon lat,lon` are also supported.)

### 300-customer DeKalb instance (offline pedestrian routing)

`data_processed/realworld_dekalb_il_300` covers the whole of DeKalb, IL (300 customers,
302 nodes, 90,301 arcs, 10 trucks, 8 h horizon). At this scale both public pedestrian
routers failed (Valhalla dropped connections; the OSRM foot profile throttled the build),
so robot distances are computed **fully offline** by `scripts/local_pedestrian.py`
(`--robot-engine local --osm-pbf`): it parses a local Geofabrik Illinois PBF extract with
pyrosm, keeps the largest connected walk component, and runs in-process Dijkstra for
all-pairs pedestrian shortest paths. Truck matrices still use the public OSRM driving
server. The 360 MB PBF is local infrastructure and is intentionally not committed.
Reproduce with:

```bash
python -u scripts/build_real_world_instance.py \
    --bbox "41.960,41.900,-88.715,-88.790" \
    --n-customers 300 --seed 7 --min-sep-km 0.10 \
    --robot-engine local --osm-pbf /path/to/illinois-YYMMDD.osm.pbf \
    --out-dir data_processed/realworld_dekalb_il_300
```

`data_processed/realworld_dekalb_il_300_slice6` (and `_slice8`) are small pilots cut from
the 300 for solver experiments — see `scripts/slice_instance.py` and `docs/LAPTOP_SETUP.md`.

### Time-dependent truck travel times (paper's t_ij(tau))

`scripts/time_dependent.py` implements the Ichoua–Gendreau–Semet (2003)
speed-profile construction: the working day is split into P departure periods,
each with a speed factor relative to free-flow speed; travel times are computed
by *integrating speeds* through period boundaries, so the FIFO property holds by
construction (asserted via `verify_fifo` on every build and recorded in
`parameters.json` with the profile, period bounds, factors, and citations).
Magnitudes follow Figliozzi (2012)'s 2.5:1 max slowdown bound. Robot (no public
pedestrian-congestion data) and drone (uncongested airspace) layers stay static.
The exact MILP remains the static free-flow baseline for gap measurement;
heuristics consume t_ij(tau) directly by looking up the departure period.

Three profiles: `two_peak` (two-peak urban day, max 2:1 slowdown), `mild`
(small-city, e.g. DeKalb, slowest factor 0.80), `flat` (all factors 1.0, the
static control). Speed factors are defined as resolution-independent
day-fraction segments and discretized at period midpoints, so any P works.

Two ways to generate the matrices:
- **At build time:** `--time-periods P --td-profile {two_peak,mild,flat}` writes
  `truck_time_matrix_p0.csv` … `truck_time_matrix_p{P-1}.csv` into the new
  instance directory.
- **Retrofitting an existing instance:**
  `scripts/add_td_matrices.py --instance-dir DIR --profile {two_peak,mild,flat} --periods P`
  writes `truck_time_matrix_{profile}_p{p}.csv` (profile in the filename so
  several profiles can coexist) and merges provenance under `td_profiles` in
  `parameters.json`.

The four small DeKalb instances (`realworld_dekalb_il_300_slice6`,
`realworld_dekalb_il_300_slice8`, `realworld_dekalb_il_10`,
`realworld_dekalb_il_25`) already ship 8-period × 3-profile TD matrix sets. The
mathematics is documented in `pdf/model_formulation.tex`, Section "Time-Dependent
Travel Times".

## Benchmark Data Attribution

The raw benchmark instances in `data_raw/tdrp_tw` come from the TDRP-TW dataset:

> Li, Hongqi (2021). "TDRP-TW instances." Mendeley Data, V1. DOI: [10.17632/tn4hkfrn9w.1](https://doi.org/10.17632/tn4hkfrn9w.1). Licensed under [CC BY 4.0](http://creativecommons.org/licenses/by/4.0).

The processed cases in `data_processed/tdrp_tw_literature_params` are derived from this dataset with literature-based platform parameters (VRP-DR paper Table 3, arXiv:2505.23584) applied. Robot fleet values are experimental placeholders as described above.
