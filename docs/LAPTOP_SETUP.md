# Laptop Setup Guide

Reproduce the truck-drone-robot routing experiments on your own machine.
No API keys are required for anything below.

## 1. Clone

```bash
git clone https://github.com/Realosunboy6/Truck-Drone-Robot-Routing-Problem-with-Time-Windows.git
cd Truck-Drone-Robot-Routing-Problem-with-Time-Windows
```

All 98 instances ship in `data_processed/` — nothing else to download:

- `tdrp_tw_literature_params/` — 18 literature-parameter benchmark cases
- `tdrp_tw_realistic_v1/` — 75 repaired benchmark cases (per-instance horizons, literature hardware)
- `realworld_dekalb_il_10/`, `realworld_dekalb_il_25/`, `realworld_dekalb_il_300/` — real DeKalb, IL geography
- `realworld_dekalb_il_300_slice6/`, `realworld_dekalb_il_300_slice8/` — small solver pilots cut from the 300

## 2. Python environment (free solvers)

Python 3.12 recommended.

```bash
python3 -m venv venv
source venv/bin/activate   # Windows: venv\Scripts\activate
pip install osmnx==2.1.1 pyrosm==0.14.0 PuLP==2.8.0 highspy==1.15.1 \
    pandas==3.0.6 openpyxl==3.1.5 geopandas==1.2.0 networkx==3.7 \
    numpy==2.5.3 shapely==2.1.2 pyproj==3.8.0
```

This gives you: OSM data tools, the PuLP/CBC free MILP stack (`model/run_with_pulp.py`
runs the docplex model through a compatibility shim — no CPLEX needed).

## 3. Reproduce the optimal multimodal pilot (6 customers, DeKalb)

```bash
DRT_DATA_DIR=$PWD/data_processed/realworld_dekalb_il_300_slice6 \
DRT_RESULTS_DIR=$PWD/results/pulp_pilot \
DRT_INSTANCE_TAG=dekalb300_slice6 \
DRT_RUN_TAG=pulp_cbc_multimodal \
DRT_MAX_CUSTOMERS_PER_SORTIE=3 \
DRT_TOP_SORTIES_PER_TRUCK_LEG=8 \
DRT_USE_TRUCK_WARM_START=1 \
DRT_FAST_BUILD=1 \
DRT_TIME_LIMIT_SECONDS=600 \
python model/run_with_pulp.py
```

Expected result: **proven optimal, objective 67.1552** —
truck serves customers 4, 5, 3; robot flies depot → 2 → 6 → 1 → 3;
drones unused (outcompeted on cost). Workbook lands in `results/pulp_pilot/`.

## 4. Reproduce the truck-only baseline (10 customers, repaired benchmark)

```bash
DRT_DATA_DIR=$PWD/data_processed/tdrp_tw_realistic_v1/10-25 \
DRT_RESULTS_DIR=$PWD/results/pulp_pilot \
DRT_INSTANCE_TAG=tdrp_10_25_realistic \
DRT_RUN_TAG=pulp_cbc_truckonly \
DRT_TRUCK_ONLY=1 \
DRT_TIME_LIMIT_SECONDS=600 \
python model/run_with_pulp.py
```

Expected: optimal, objective **296.8**, one truck, zero penalties.

## 5. Cut your own pilot slices

```bash
python scripts/slice_instance.py \
    data_processed/realworld_dekalb_il_300 \
    data_processed/my_slice 8 --trucks 2
```

Then tighten `MAX_DRONES_PER_TRUCK` / `MAX_ROBOTS_PER_TRUCK` to 1 in the
slice's `parameters.json` to keep the MILP small for CBC.

## 6. Regenerating the 300-customer instance (optional)

The instance is already built and validated in the repo. To rebuild it you need
the Illinois map extract (360 MB, not in the repo — too large):

- Download from Geofabrik: https://download.geofabrik.de/north-america/us/illinois.html
  (a dated `illinois-YYMMDD.osm.pbf` works; record the filename in `parameters.json`)

```bash
python -u scripts/build_real_world_instance.py \
  --bbox "41.960,41.900,-88.715,-88.790" \
  --n-customers 300 --seed 7 --min-sep-km 0.10 \
  --robot-engine local --osm-pbf /path/to/illinois-YYMMDD.osm.pbf \
  --out-dir data_processed/realworld_dekalb_il_300
```

Truck matrices use the public OSRM demo server (no key); robot matrices are
computed fully offline from the PBF.

## 7. Solver upgrade path (recommended)

CBC caps out around 6–10 customers on the full multimodal model. For your
NP-hard study — optimals first, then heuristics — get a commercial solver:

- **Gurobi** — free academic license at gurobi.com/academia; with it, run the
  model directly through docplex (no PuLP shim needed) and expect 10–100× speedups.
- **CPLEX** — also free for academics (IBM Academic Initiative).
- **OR-Tools CP-SAT** — free, strong on routing-flavored models.

## 8. Which result files to trust

Valid, independently re-verified:

- `results/pulp_pilot/tdrp_10_25_realistic_ordered_sorties_pulp_cbc_truckonly_solution.xlsx`
- `results/pulp_pilot/dekalb300_slice6_ordered_sorties_pulp_cbc_multimodal_solution.xlsx`

Stale / invalid — do not use:

- `*_pilot_warmstart_*` and `*_pulp_cbc_pilot_solution.xlsx` (superseded, infeasible runs)
