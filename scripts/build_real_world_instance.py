"""Build a real-world truck-drone-robot instance from OpenStreetMap.

Generates one processed case folder in the exact format consumed by
model/capped_flexible_docking_ordered_sortie_model.py:

    nodes.csv, customers.csv, arcs.csv,
    distance_matrix.csv, truck_time_matrix.csv,
    drone_time_matrix.csv, robot_distance_matrix.csv, robot_time_matrix.csv,
    parameters.json, instance_summary.json

Travel data is mode-realistic per platform:
  - truck: shortest-path driving distances on the OSM road network,
    via the OSRM table service (router.project-osrm.org demo server)
  - robot: shortest-path pedestrian distances on the OSM walk network,
    via Valhalla sources_to_targets with pedestrian costing
    (valhalla1.openstreetmap.de, FOSSGIS)
  - drone: haversine (great-circle) distances, i.e. straight-line flight

Physical parameters are the realistic-v1 hardware set (see
REALISTIC_PARAMS): delivery drone 80 km/h / 5 kg / 40 km sortie
(Sacramento et al. 2019), ground robot 6 km/h / 10 kg / 12 km sortie
(Starship Gen 3; Ostermeier 2021), cost structure from Malik et al.,
VRP-DR, arXiv:2505.23584 Table 3.

Demands follow the Amazon parcel rule used across the literature:
~86% of parcels weigh under 5 lbs (2.27 kg) and are drone/robot eligible;
the rest are heavier truck-leaning parcels.

Time windows: an 8-hour working day (t in [0, 8] hours). Each customer gets
a 2-hour window opening uniformly in [0, 6]; the depot closes at t=8, which
is also T_max, so route-duration penalties only trigger on genuine horizon
violations (the flat-T_max distortion of the v0 benchmark data is gone).

Requires: Python 3.10+ standard library only, plus network access to the
OSRM demo server, the FOSSGIS Valhalla instance, and Nominatim (only when
a place name is given instead of --bbox).

Example:
    python scripts/build_real_world_instance.py \
        --place "DeKalb, Illinois, USA" --n-customers 25 --seed 7 \
        --out-dir data_processed/realworld_dekalb_il_25
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import random
from datetime import datetime, timezone
from pathlib import Path


# Realistic-v1 hardware parameters (shared with build_tdrp_literature_instance.py).
REALISTIC_PARAMS = {
    "truck_speed": 45.0,   # km/h, urban delivery truck
    "drone_speed": 80.0,    # km/h, Sacramento et al. 2019 (50 mph)
    "robot_speed": 6.0,     # km/h, Starship Gen 3
    "Q_t": 1000.0,          # kg truck capacity
    "Q_d": 5.0,             # kg, Sacramento et al. 2019
    "Q_r": 10.0,            # kg, Starship Gen 3
    "E_d": 40.0,            # km max sortie distance (30-min endurance at 80 km/h)
    "E_r": 12.0,            # km max sortie distance (~6.4 km radius at 6 km/h)
    "C_w": 0.0,
    "C_veh": 2.9,
    "C_w_drone": 0.0,
    "C_drone": 0.08,
    "C_w_r": 0.0,
    "C_rob": 0.06,
    "truck_fixed_cost": 30.0,
    "drone_fixed_cost": 10.0,
    "robot_fixed_cost": 8.0,
    "lambda_E_d": 1000.0,
    "lambda_E_r": 1000.0,
    "lambda_Q": 1000.0,
    "lambda_T": 1000.0,
    "lambda_W": 1000.0,
    "parameter_scenario": "realworld_osm_v1",
}

HORIZON_HOURS = 8.0        # working day 08:00-16:00 mapped to t in [0, 8]
WINDOW_WIDTH = 2.0         # hours per customer time window
AMAZON_SMALL_SHARE = 0.86  # share of parcels under 5 lbs (2.27 kg)
SMALL_PARCEL_MAX_KG = 2.27
HEAVY_PARCEL_MAX_KG = 20.0

# Routing engines (used instead of raw Overpass graph downloads: this network's
# egress proxy truncates Overpass responses mid-stream, while these small,
# fast routing-API calls pass through reliably).
#   - truck: OSRM public demo server, table service, driving profile
#            (real OSM road-network shortest paths)
#   - robot: FOSSGIS Valhalla instance, sources_to_targets, pedestrian costing
#            (real OSM pedestrian-network shortest paths)
#   - drone: haversine great-circle distance (unrestricted airspace)
OSRM_TABLE_URL = "https://router.project-osrm.org/table/v1/driving"
VALHALLA_MATRIX_URL = "https://valhalla1.openstreetmap.de/sources_to_targets"
OSRM_BLOCK = 50       # coordinates per OSRM table request (server cap is 100)
VALHALLA_BLOCK = 10   # coordinates per Valhalla matrix request (pairs per request <= 100)
ENGINE_RETRIES = 8
ENGINE_TIMEOUT = 120


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


def haversine_km(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371.0088
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def _http_get_json(url: str, timeout: int = ENGINE_TIMEOUT) -> dict:
    """GET a JSON document with retries and exponential backoff."""
    import json
    import time
    import urllib.request

    last: Exception | None = None
    for attempt in range(1, ENGINE_RETRIES + 1):
        try:
            req = urllib.request.Request(
                url, headers={"User-Agent": "tdrp-instance-generator/1.0"})
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                return json.load(resp)
        except Exception as exc:  # noqa: BLE001 - transient network failures
            last = exc
            print(f"GET attempt {attempt} failed ({type(exc).__name__}); retrying...")
            time.sleep(2 ** attempt)
    raise RuntimeError(f"GET {url[:80]}... failed after {ENGINE_RETRIES} attempts: {last}")


def _http_post_json(url: str, payload: dict, timeout: int = ENGINE_TIMEOUT) -> dict:
    """POST a JSON document with retries and exponential backoff."""
    import json
    import random
    import time
    import urllib.request

    data = json.dumps(payload).encode()
    last: Exception | None = None
    for attempt in range(1, ENGINE_RETRIES + 1):
        try:
            req = urllib.request.Request(
                url, data=data,
                headers={"Content-Type": "application/json",
                         "User-Agent": "tdrp-instance-generator/1.0"},
                method="POST")
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                return json.load(resp)
        except Exception as exc:  # noqa: BLE001 - transient network failures
            last = exc
            wait = 2 ** attempt + random.uniform(0, 2)
            print(f"POST attempt {attempt} failed ({type(exc).__name__}); waiting {wait:.0f}s...")
            time.sleep(wait)
    raise RuntimeError(f"POST {url} failed after {ENGINE_RETRIES} attempts: {last}")


def osrm_driving_matrix(latlon: list[tuple[float, float]]
                        ) -> tuple[list[list[float | None]], list[tuple[float, float]]]:
    """Truck road-network distances (km) via the OSRM table service (driving).

    Returns (dist_km, snapped_latlon); entries are None when the engine
    cannot route between a pair.
    """
    n = len(latlon)
    dist_km: list[list[float | None]] = [[None] * n for _ in range(n)]
    snapped: list[tuple[float, float] | None] = [None] * n
    for a in range(0, n, OSRM_BLOCK):
        for b in range(0, n, OSRM_BLOCK):
            a_idx = list(range(a, min(a + OSRM_BLOCK, n)))
            b_idx = list(range(b, min(b + OSRM_BLOCK, n)))
            coords = [latlon[i] for i in a_idx] + [latlon[i] for i in b_idx]
            coord_str = ";".join(f"{lon:.6f},{lat:.6f}" for lat, lon in coords)
            src = ";".join(str(k) for k in range(len(a_idx)))
            dst = ";".join(str(len(a_idx) + k) for k in range(len(b_idx)))
            url = (f"{OSRM_TABLE_URL}/{coord_str}"
                   f"?annotations=distance&sources={src}&destinations={dst}")
            doc = _http_get_json(url)
            if doc.get("code") != "Ok":
                raise RuntimeError(f"OSRM table error: {doc.get('code')}: {doc.get('message')}")
            table = doc.get("distances") or doc["durations"]
            for ri, gi in enumerate(a_idx):
                for cj, gj in enumerate(b_idx):
                    val = table[ri][cj]
                    dist_km[gi][gj] = (val / 1000.0) if val is not None else None
            for k, gj in enumerate(b_idx):
                loc = doc["destinations"][k]["location"]  # [lon, lat], snapped
                snapped[gj] = (loc[1], loc[0])
    assert all(s is not None for s in snapped)
    return dist_km, [s for s in snapped if s is not None]


def valhalla_pedestrian_matrix(latlon: list[tuple[float, float]]
                               ) -> tuple[list[list[float | None]], list[tuple[float, float]]]:
    """Robot pedestrian-network distances (km) via Valhalla.

    Returns (dist_km, snapped_latlon); entries are None when the engine
    cannot route between a pair.
    """
    n = len(latlon)
    dist_km: list[list[float | None]] = [[None] * n for _ in range(n)]
    snapped: list[tuple[float, float] | None] = [None] * n
    for a in range(0, n, VALHALLA_BLOCK):
        for b in range(0, n, VALHALLA_BLOCK):
            a_idx = list(range(a, min(a + VALHALLA_BLOCK, n)))
            b_idx = list(range(b, min(b + VALHALLA_BLOCK, n)))
            payload = {
                "sources": [{"lat": latlon[i][0], "lon": latlon[i][1]} for i in a_idx],
                "targets": [{"lat": latlon[i][0], "lon": latlon[i][1]} for i in b_idx],
                "costing": "pedestrian",
            }
            doc = _http_post_json(VALHALLA_MATRIX_URL, payload)
            rows = doc["sources_to_targets"]
            for ri, gi in enumerate(a_idx):
                for cj, gj in enumerate(b_idx):
                    cell = rows[ri][cj]
                    dist_km[gi][gj] = cell["distance"] if cell["distance"] is not None else None
            for k, gj in enumerate(b_idx):
                t = doc["targets"][k]
                snapped[gj] = (t["lat"], t["lon"])
    assert all(s is not None for s in snapped)
    return dist_km, [s for s in snapped if s is not None]


def _geocode_bbox(place: str) -> tuple[float, float, float, float]:
    """Resolve a place name to a (north, south, east, west) bbox via Nominatim."""
    import urllib.parse

    q = urllib.parse.quote(place)
    url = (f"https://nominatim.openstreetmap.org/search?q={q}"
           f"&format=json&limit=1")
    doc = _http_get_json(url)
    if not doc:
        raise RuntimeError(f"Nominatim found no result for place {place!r}; pass --bbox instead")
    south, north, west, east = (float(v) for v in doc[0]["boundingbox"])
    print(f"geocoded {place!r} to bbox({north},{south},{east},{west})")
    return north, south, east, west

def build_instance(
    place: str | None,
    bbox: tuple[float, float, float, float] | None,
    n_customers: int,
    seed: int,
    out_dir: Path,
    depot_latlon: tuple[float, float] | None = None,
) -> None:
    rng = random.Random(seed)

    if bbox is not None:
        north, south, east, west = bbox
        place_label = f"bbox({north},{south},{east},{west})"
    else:
        north, south, east, west = _geocode_bbox(place)
        place_label = place
    assert south < north and west < east

    # Sample customer points spread across the bbox (min separation avoids
    # clusters of near-duplicate delivery points). The routing engines snap
    # each point to the road/pedestrian networks when the matrices are built.
    min_sep_km = 0.15
    cust_latlon: list[tuple[float, float]] = []
    attempts = 0
    while len(cust_latlon) < n_customers and attempts < 20000:
        attempts += 1
        lat = rng.uniform(south, north)
        lon = rng.uniform(west, east)
        if all(haversine_km(lat, lon, la, lo) >= min_sep_km for la, lo in cust_latlon):
            cust_latlon.append((lat, lon))
    if len(cust_latlon) < n_customers:
        raise RuntimeError("could not sample enough separated customer points; widen the area")

    # Depot: customer centroid (or explicit override); the engines snap it to
    # the networks like any other point.
    if depot_latlon is not None:
        depot = depot_latlon
    else:
        depot = (sum(la for la, _ in cust_latlon) / n_customers,
                 sum(lo for _, lo in cust_latlon) / n_customers)

    points: list[tuple[float, float]] = [depot] + cust_latlon  # index 0 = depot

    # Route every pair on the real networks. Sampled points the engines cannot
    # route are replaced (bounded rounds); a depot that cannot route is fatal.
    def _rowcol_ok(m: list[list[float | None]], i: int) -> bool:
        n_ = len(m)
        return all(m[i][j] is not None and m[j][i] is not None for j in range(n_))

    for _round in range(5):
        truck_mat, snapped = osrm_driving_matrix(points)
        robot_mat, _snapped_walk = valhalla_pedestrian_matrix(points)
        if not _rowcol_ok(truck_mat, 0) or not _rowcol_ok(robot_mat, 0):
            raise RuntimeError("depot location is not routable on the road/pedestrian network")
        bad = [i for i in range(1, len(points))
               if not (_rowcol_ok(truck_mat, i) and _rowcol_ok(robot_mat, i))]
        if not bad:
            break
        print(f"resampling {len(bad)} unroutable customer point(s)...")
        for i in bad:
            for _ in range(20000):
                lat = rng.uniform(south, north)
                lon = rng.uniform(west, east)
                others = [p for k, p in enumerate(points) if k != i]
                if all(haversine_km(lat, lon, la, lo) >= min_sep_km for la, lo in others):
                    points[i] = (lat, lon)
                    break
            else:
                raise RuntimeError("could not resample a routable customer point")
    else:
        raise RuntimeError("could not find routable customer points after 5 rounds")

    # Canonical coordinates: engine-snapped positions.
    depot_lat, depot_lon = snapped[0]
    cust_latlon = snapped[1:]

    def drive_km(a_idx: int, b_idx: int) -> float:
        val = truck_mat[a_idx][b_idx]
        assert val is not None
        return val

    def walk_km(a_idx: int, b_idx: int) -> float:
        val = robot_mat[a_idx][b_idx]
        assert val is not None
        return val
    # Demands: Amazon parcel rule.
    demands: list[float] = []
    for _ in range(n_customers):
        if rng.random() < AMAZON_SMALL_SHARE:
            demands.append(round(rng.uniform(0.5, SMALL_PARCEL_MAX_KG), 2))
        else:
            demands.append(round(rng.uniform(SMALL_PARCEL_MAX_KG, HEAVY_PARCEL_MAX_KG), 2))

    # Time windows over the 8-hour working day.
    windows: list[tuple[float, float]] = []
    for _ in range(n_customers):
        open_t = round(rng.uniform(0, HORIZON_HOURS - WINDOW_WIDTH), 2)
        windows.append((open_t, round(open_t + WINDOW_WIDTH, 2)))
    service_times = [round(rng.choice([0.08, 0.15, 0.25]), 2) for _ in range(n_customers)]

    n = n_customers
    end_depot = n + 1
    size = n + 2
    node_ids = list(range(size))

    params = dict(REALISTIC_PARAMS)
    params.update(
        {
            "source_dataset": "OSM real-world",
            "source_place": place_label,
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "generator": "scripts/build_real_world_instance.py",
            "generator_seed": seed,
            "NUM_CUSTOMERS": n,
            "NUM_TRUCKS": 2 if n <= 12 else 3,
            "NUM_DRONES": n,
            "DRONES_CARRIED_AT_DEPOT": 1,
            "MAX_DRONES_PER_TRUCK": 2,
            "NUM_ROBOTS": n,
            "ROBOTS_CARRIED_AT_DEPOT": 1,
            "MAX_ROBOTS_PER_TRUCK": 2,
            "T_max": HORIZON_HOURS,
            "T_max_source": "working-day horizon 08:00-16:00 mapped to t in [0, 8]",
            "parameter_source": (
                "Drone: Sacramento et al. 2019 (50 mph / 80 km/h, 30-min endurance, 5 kg payload); "
                "Robot: Starship Gen 3 (6 km/h, 10 kg payload, ~6.4 km delivery radius) and "
                "Ostermeier 2021 (5 km/h robot); "
                "Costs: Malik et al., VRP-DR, arXiv:2505.23584, Table 3; "
                f"Truck road distances: OSRM table service (driving profile) on the OSM road network, "
                f"via the router.project-osrm.org public demo server ({place_label}); "
                f"robot pedestrian distances: Valhalla sources_to_targets (pedestrian costing) on the OSM "
                f"walk network, via valhalla1.openstreetmap.de (FOSSGIS); "
                "queried " + datetime.now(timezone.utc).date().isoformat()
            ),
            "demand_rule": (
                f"Amazon parcel rule: {AMAZON_SMALL_SHARE:.0%} of parcels U[0.5, {SMALL_PARCEL_MAX_KG}] kg "
                f"(drone/robot eligible), remainder U[{SMALL_PARCEL_MAX_KG}, {HEAVY_PARCEL_MAX_KG}] kg"
            ),
            "time_window_rule": (
                f"8-hour working day t in [0, {HORIZON_HOURS}]; each customer gets a {WINDOW_WIDTH} h window "
                f"opening uniformly in [0, {HORIZON_HOURS - WINDOW_WIDTH}]"
            ),
            "assumptions": [
                "Truck travel: OSRM shortest-path driving distances on the OSM road network at a constant 45 km/h.",
                "Robot travel: Valhalla shortest-path pedestrian distances on the OSM walk network at 6 km/h.",
                "Drone travel: haversine great-circle distance at 80 km/h (unrestricted airspace).",
                "Canonical end depot n+1 duplicates depot 0.",
            ],
            "warnings": [
                "Truck speed is constant; OSM maxspeed-based travel times are a future refinement.",
                "Drone flight is straight-line; no-fly zones and wind are not modeled.",
                "Routing data comes from public demo servers (OSRM, Valhalla); for publication-grade "
                "instances use a self-hosted engine or a local OSM extract.",
            ],
        }
    )

    nodes = [{"node_id": 0, "node_type": "start_depot", "x": depot_lon, "y": depot_lat}]
    nodes.extend(
        {"node_id": i + 1, "node_type": "customer", "x": lon, "y": lat}
        for i, (lat, lon) in enumerate(cust_latlon)
    )
    nodes.append({"node_id": end_depot, "node_type": "end_depot", "x": depot_lon, "y": depot_lat})

    customers = [
        {
            "customer_id": i + 1,
            "demand": demands[i],
            "open_time": windows[i][0],
            "close_time": windows[i][1],
            "service_time": service_times[i],
        }
        for i in range(n)
    ]

    distance_matrix = [[0.0] * size for _ in range(size)]
    truck_time_matrix = [[0.0] * size for _ in range(size)]
    drone_time_matrix = [[0.0] * size for _ in range(size)]
    robot_distance_matrix = [[0.0] * size for _ in range(size)]
    robot_time_matrix = [[0.0] * size for _ in range(size)]

    def canon(idx: int) -> int:
        return 0 if idx == end_depot else idx

    arcs = []
    for i in node_ids:
        for j in node_ids:
            if i == j:
                continue
            ci, cj = canon(i), canon(j)
            truck_km = drive_km(ci, cj)
            robot_km = walk_km(ci, cj)
            lat_i, lon_i = (depot_lat, depot_lon) if ci == 0 else cust_latlon[ci - 1]
            lat_j, lon_j = (depot_lat, depot_lon) if cj == 0 else cust_latlon[cj - 1]
            drone_km = haversine_km(lat_i, lon_i, lat_j, lon_j)
            truck_h = truck_km / params["truck_speed"]
            robot_h = robot_km / params["robot_speed"]
            drone_h = drone_km / params["drone_speed"]
            distance_matrix[i][j] = truck_km
            truck_time_matrix[i][j] = truck_h
            drone_time_matrix[i][j] = drone_h
            robot_distance_matrix[i][j] = robot_km
            robot_time_matrix[i][j] = robot_h
            if j != 0 and i != end_depot:
                arcs.append(
                    {
                        "i": i,
                        "j": j,
                        "distance": truck_km,
                        "truck_time": truck_h,
                        "drone_time": drone_h,
                        "robot_distance": robot_km,
                        "robot_time": robot_h,
                    }
                )

    params["NUM_NODES"] = size
    params["NUM_ARCS"] = len(arcs)

    out_dir.mkdir(parents=True, exist_ok=True)
    write_csv(out_dir / "nodes.csv", nodes, ["node_id", "node_type", "x", "y"])
    write_csv(out_dir / "customers.csv", customers,
              ["customer_id", "demand", "open_time", "close_time", "service_time"])
    write_csv(out_dir / "arcs.csv", arcs,
              ["i", "j", "distance", "truck_time", "drone_time", "robot_distance", "robot_time"])
    write_matrix(out_dir / "distance_matrix.csv", distance_matrix)
    write_matrix(out_dir / "truck_time_matrix.csv", truck_time_matrix)
    write_matrix(out_dir / "drone_time_matrix.csv", drone_time_matrix)
    write_matrix(out_dir / "robot_distance_matrix.csv", robot_distance_matrix)
    write_matrix(out_dir / "robot_time_matrix.csv", robot_time_matrix)
    (out_dir / "parameters.json").write_text(json.dumps(params, indent=2), encoding="utf-8")

    summary = {
        "source_dataset": "OSM real-world",
        "source_place": place_label,
        "generator_seed": seed,
        "number_of_customers": n,
        "number_of_nodes": size,
        "number_of_arcs": len(arcs),
        "demand_min": min(demands),
        "demand_max": max(demands),
        "demand_total": round(sum(demands), 2),
        "time_window_open_min": min(w[0] for w in windows),
        "time_window_close_max": max(w[1] for w in windows),
        "truck_routing_source": "OSRM table v1/driving via router.project-osrm.org (OSM road network)",
        "robot_routing_source": "Valhalla sources_to_targets, costing=pedestrian via valhalla1.openstreetmap.de (OSM walk network)",
        "drone_distance_source": "haversine great-circle on WGS84",
        "depot_lat": depot_lat,
        "depot_lon": depot_lon,
        "parameter_scenario": params["parameter_scenario"],
        "assumptions": params["assumptions"],
        "warnings": params["warnings"],
    }
    (out_dir / "instance_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"Built real-world instance: {out_dir} ({n} customers, {place_label})")


def main() -> int:
    parser = argparse.ArgumentParser(description="Build a real-world OSM-based truck-drone-robot instance.")
    parser.add_argument("--place", default=None, help='OSM place query, e.g. "DeKalb, Illinois, USA"')
    parser.add_argument("--bbox", default=None,
                        help="north,south,east,west bounding box (overrides --place)")
    parser.add_argument("--n-customers", type=int, required=True)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--depot-latlon", default=None, help="lat,lon of depot (default: customer centroid)")
    args = parser.parse_args()
    if args.place is None and args.bbox is None:
        parser.error("one of --place or --bbox is required")
    bbox = tuple(float(v) for v in args.bbox.split(",")) if args.bbox else None
    depot_latlon = tuple(float(v) for v in args.depot_latlon.split(",")) if args.depot_latlon else None
    build_instance(args.place, bbox, args.n_customers, args.seed, args.out_dir, depot_latlon)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
