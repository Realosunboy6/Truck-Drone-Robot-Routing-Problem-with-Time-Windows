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
#            (valhalla1.openstreetmap.de) -- or, for large instances, the
#            FOSSGIS OSRM foot-profile table service (routing.openstreetmap.de),
#            which answers 50x50 blocks reliably where Valhalla drops connections
#   - drone: haversine great-circle distance (unrestricted airspace)
OSRM_TABLE_URL = "https://router.project-osrm.org/table/v1/driving"
OSRM_FOOT_TABLE_URL = "https://routing.openstreetmap.de/routed-foot/table/v1/driving"
VALHALLA_MATRIX_URL = "https://valhalla1.openstreetmap.de/sources_to_targets"
OSRM_BLOCK = 100      # coordinates per OSRM driving-table request (server cap is 100; 100x100 tested OK)
FOOT_BLOCK = 50       # coordinates per OSRM foot-table request (100x100 responses get truncated ~80KB)
VALHALLA_BLOCK = 10   # coordinates per Valhalla matrix request (pairs per request <= 100;
                      # the public server drops larger requests, so keep this conservative)
VALHALLA_DELAY = 0.25  # politeness pause (s) between Valhalla block requests
SNAP_COLLIDE_KM = 0.05  # snapped points closer than this are resampled (degenerate pairs)
OSRM_REQUEST_DELAY = 1.0  # politeness pause (s) between OSRM table block requests
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


def _osrm_table_request(table_url: str,
                        src_latlon: list[tuple[float, float]],
                        dst_latlon: list[tuple[float, float]],
                        label: str) -> dict:
    """One OSRM table request: distances (m) for sources x destinations."""
    coords = list(src_latlon) + list(dst_latlon)
    coord_str = ";".join(f"{lon:.6f},{lat:.6f}" for lat, lon in coords)
    src = ";".join(str(k) for k in range(len(src_latlon)))
    dst = ";".join(str(len(src_latlon) + k) for k in range(len(dst_latlon)))
    url = (f"{table_url}/{coord_str}"
           f"?annotations=distance&sources={src}&destinations={dst}")
    doc = _http_get_json(url)
    if doc.get("code") != "Ok":
        raise RuntimeError(f"OSRM {label} table error: {doc.get('code')}: {doc.get('message')}")
    return doc


def _checkpoint_path(out_dir: Path | None, label: str) -> Path | None:
    return out_dir / f".checkpoint_{label}.json" if out_dir is not None else None


def _load_checkpoint(path: Path, table_url: str,
                     latlon: list[tuple[float, float]]) -> dict:
    """Return saved blocks dict if the checkpoint matches these exact points."""
    try:
        saved = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    want = [[round(la, 6), round(lo, 6)] for la, lo in latlon]
    if saved.get("url") == table_url and saved.get("points") == want:
        return saved.get("blocks", {})
    return {}


def _save_checkpoint(path: Path, table_url: str,
                     latlon: list[tuple[float, float]], blocks: dict) -> None:
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps({
        "url": table_url,
        "points": [[round(la, 6), round(lo, 6)] for la, lo in latlon],
        "blocks": blocks,
    }), encoding="utf-8")
    tmp.replace(path)


def osrm_table_matrix(latlon: list[tuple[float, float]],
                      table_url: str,
                      block: int,
                      label: str,
                      out_dir: Path | None = None,
                      ) -> tuple[list[list[float | None]], list[tuple[float, float]]]:
    """Distance matrix (km) via an OSRM table service.

    Returns (dist_km, snapped_latlon); entries are None when the engine
    cannot route between a pair. Completed blocks are checkpointed to
    out_dir so an interrupted run resumes instead of restarting.
    """
    import time
    n = len(latlon)
    dist_km: list[list[float | None]] = [[None] * n for _ in range(n)]
    snapped: list[tuple[float, float] | None] = [None] * n
    ckpt = _checkpoint_path(out_dir, label)
    blocks: dict[str, dict] = _load_checkpoint(ckpt, table_url, latlon) if ckpt else {}
    if blocks:
        print(f"  {label}: resuming from checkpoint ({len(blocks)} blocks done)")
    total = ((n + block - 1) // block) ** 2
    for a in range(0, n, block):
        for b in range(0, n, block):
            key = f"{a}_{b}"
            a_idx = list(range(a, min(a + block, n)))
            b_idx = list(range(b, min(b + block, n)))
            if key in blocks:
                table = blocks[key]["table"]
                dest = blocks[key]["destinations"]
            else:
                doc = _osrm_table_request(
                    table_url, [latlon[i] for i in a_idx], [latlon[i] for i in b_idx], label)
                table = doc.get("distances") or doc["durations"]
                dest = doc["destinations"]
                blocks[key] = {"table": table, "destinations": dest}
                if ckpt:
                    _save_checkpoint(ckpt, table_url, latlon, blocks)
                time.sleep(OSRM_REQUEST_DELAY)
            for ri, gi in enumerate(a_idx):
                for cj, gj in enumerate(b_idx):
                    val = table[ri][cj]
                    dist_km[gi][gj] = (val / 1000.0) if val is not None else None
            for k, gj in enumerate(b_idx):
                loc = dest[k]["location"]  # [lon, lat], snapped
                snapped[gj] = (loc[1], loc[0])
            if len(blocks) % 25 == 0 or len(blocks) == total:
                print(f"  {label} matrix: {len(blocks)}/{total} blocks")
    assert all(s is not None for s in snapped)
    if ckpt and ckpt.exists():
        ckpt.unlink()
    return dist_km, [s for s in snapped if s is not None]


def osrm_table_refresh(latlon: list[tuple[float, float]],
                       table_url: str,
                       block: int,
                       label: str,
                       indices: list[int],
                       ) -> tuple[dict[int, list[float | None]], dict[int, list[float | None]]]:
    """Refresh only the rows and columns of the given point indices.

    Returns (rows, cols, snapped) with rows[i][j] = i->j and cols[i][j] = j->i
    in km, plus snapped engine positions for every point.
    Used after resampling points so a full matrix rebuild is unnecessary.
    """
    import time
    n = len(latlon)
    idx = sorted(set(indices))
    rows: dict[int, list[float | None]] = {i: [None] * n for i in idx}
    cols: dict[int, list[float | None]] = {i: [None] * n for i in idx}
    new_snapped: dict[int, tuple[float, float]] = {}
    # rows: sources=idx (batched), destinations=all
    for sb in range(0, len(idx), block):
        s_idx = idx[sb:sb + block]
        for db in range(0, n, block):
            d_idx = list(range(db, min(db + block, n)))
            doc = _osrm_table_request(
                table_url, [latlon[i] for i in s_idx], [latlon[i] for i in d_idx], label)
            table = doc.get("distances") or doc["durations"]
            for ri, gi in enumerate(s_idx):
                for cj, gj in enumerate(d_idx):
                    val = table[ri][cj]
                    rows[gi][gj] = (val / 1000.0) if val is not None else None
            for cj, gj in enumerate(d_idx):
                loc = doc["destinations"][cj]["location"]  # [lon, lat], snapped
                new_snapped[gj] = (loc[1], loc[0])
            time.sleep(OSRM_REQUEST_DELAY)
    # cols: sources=all, destinations=idx (batched)
    for sb in range(0, n, block):
        s_idx = list(range(sb, min(sb + block, n)))
        for db in range(0, len(idx), block):
            d_idx = idx[db:db + block]
            doc = _osrm_table_request(
                table_url, [latlon[i] for i in s_idx], [latlon[i] for i in d_idx], label)
            table = doc.get("distances") or doc["durations"]
            for ri, gi in enumerate(s_idx):
                for cj, gj in enumerate(d_idx):
                    val = table[ri][cj]
                    cols[gj][gi] = (val / 1000.0) if val is not None else None
            time.sleep(OSRM_REQUEST_DELAY)
    return rows, cols, new_snapped


def _valhalla_block(a_idx: list[int], b_idx: list[int],
                    latlon: list[tuple[float, float]]) -> dict:
    """One Valhalla sources_to_targets request for the given index blocks."""
    import time
    payload = {
        "sources": [{"lat": latlon[i][0], "lon": latlon[i][1]} for i in a_idx],
        "targets": [{"lat": latlon[i][0], "lon": latlon[i][1]} for i in b_idx],
        "costing": "pedestrian",
    }
    doc = _http_post_json(VALHALLA_MATRIX_URL, payload)
    time.sleep(VALHALLA_DELAY)
    return doc


def valhalla_pedestrian_matrix(latlon: list[tuple[float, float]]
                               ) -> tuple[list[list[float | None]], list[tuple[float, float]]]:
    """Robot pedestrian-network distances (km) via Valhalla.

    Returns (dist_km, snapped_latlon); entries are None when the engine
    cannot route between a pair.
    """
    n = len(latlon)
    dist_km: list[list[float | None]] = [[None] * n for _ in range(n)]
    snapped: list[tuple[float, float] | None] = [None] * n
    total = (n + VALHALLA_BLOCK - 1) // VALHALLA_BLOCK
    done = 0
    for a in range(0, n, VALHALLA_BLOCK):
        for b in range(0, n, VALHALLA_BLOCK):
            a_idx = list(range(a, min(a + VALHALLA_BLOCK, n)))
            b_idx = list(range(b, min(b + VALHALLA_BLOCK, n)))
            doc = _valhalla_block(a_idx, b_idx, latlon)
            rows = doc["sources_to_targets"]
            for ri, gi in enumerate(a_idx):
                for cj, gj in enumerate(b_idx):
                    cell = rows[ri][cj]
                    dist_km[gi][gj] = cell["distance"] if cell["distance"] is not None else None
            for k, gj in enumerate(b_idx):
                t = doc["targets"][k]
                snapped[gj] = (t["lat"], t["lon"])
            done += 1
            if done % 50 == 0 or done == total * total:
                print(f"  valhalla matrix: {done}/{total * total} blocks")
    assert all(s is not None for s in snapped)
    return dist_km, [s for s in snapped if s is not None]


def valhalla_point_rowcol(latlon: list[tuple[float, float]], idx: int
                          ) -> tuple[list[float | None], list[float | None]]:
    """Pedestrian distances for a single point: row[j] = idx->j, col[j] = j->idx.

    Used to refresh one resampled point without recomputing the whole matrix.
    """
    n = len(latlon)
    row: list[float | None] = [None] * n
    col: list[float | None] = [None] * n
    for b in range(0, n, VALHALLA_BLOCK):
        jdx = list(range(b, min(b + VALHALLA_BLOCK, n)))
        doc = _valhalla_block([idx], jdx, latlon)
        cells = doc["sources_to_targets"][0]
        for cj, gj in enumerate(jdx):
            row[gj] = cells[cj]["distance"] if cells[cj]["distance"] is not None else None
    for b in range(0, n, VALHALLA_BLOCK):
        jdx = list(range(b, min(b + VALHALLA_BLOCK, n)))
        doc = _valhalla_block(jdx, [idx], latlon)
        rows = doc["sources_to_targets"]
        for ri, gj in enumerate(jdx):
            cell = rows[ri][0]
            col[gj] = cell["distance"] if cell["distance"] is not None else None
    return row, col


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
    min_sep_km: float = 0.15,
    robot_engine: str = "valhalla",
    osm_pbf: str | Path | None = None,
) -> None:
    rng = random.Random(seed)
    if robot_engine not in ("valhalla", "osrm-foot", "local"):
        raise ValueError(f"unknown robot_engine {robot_engine!r}")

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
    out_dir.mkdir(parents=True, exist_ok=True)  # early: matrix checkpoints live here

    # Offline pedestrian engine: parse the local OSM walk network once and
    # reuse it for every round (no server, no throttling, fully deterministic).
    local_engine = None
    if robot_engine == "local":
        if osm_pbf is None:
            raise ValueError("--robot-engine local requires --osm-pbf <path-to-.osm.pbf>")
        try:
            from local_pedestrian import LocalPedestrianEngine
        except ImportError:  # invoked as scripts.build_real_world_instance
            from scripts.local_pedestrian import LocalPedestrianEngine
        print("parsing local OSM walk network (one-time)...", flush=True)
        local_engine = LocalPedestrianEngine(
            osm_pbf, (south - 0.03, north + 0.03, west - 0.03, east + 0.03))
        print(f"  walk network ready: {local_engine.n_nodes} nodes", flush=True)

    # Route every pair on the real networks. Sampled points the engines cannot
    # route are replaced (bounded rounds); a depot that cannot route is fatal.
    # The pedestrian matrix is the expensive step, so after round 0 only the
    # rows/cols of resampled points are refreshed; the truck matrix is cheap
    # enough to rebuild fully each round.
    def _rowcol_ok(m: list[list[float | None]], i: int) -> bool:
        n_ = len(m)
        return all(m[i][j] is not None and m[j][i] is not None for j in range(n_))

    robot_mat: list[list[float | None]] = []
    bad: list[int] = []

    def _robot_full_matrix(pts: list[tuple[float, float]], out: Path | None
                           ) -> tuple[list[list[float | None]], list[tuple[float, float]]]:
        if robot_engine == "osrm-foot":
            return osrm_table_matrix(pts, OSRM_FOOT_TABLE_URL, FOOT_BLOCK, "foot", out)
        if robot_engine == "local":
            return local_engine.matrix(pts)
        return valhalla_pedestrian_matrix(pts)

    def _refresh_osrm(mat: list[list[float | None]], table_url: str, block: int,
                      label: str, indices: list[int]) -> None:
        rows, cols, new_snapped = osrm_table_refresh(points, table_url, block, label, indices)
        for i in indices:
            mat[i] = rows[i]
            for j in range(len(points)):
                mat[j][i] = cols[i][j]
        for gj, pos in new_snapped.items():
            snapped[gj] = pos

    for _round in range(5):
        if _round == 0:
            print("round 1/5: truck matrix (checkpointed)...")
            truck_mat, snapped = osrm_table_matrix(
                points, OSRM_TABLE_URL, OSRM_BLOCK, "driving", out_dir)
            print(f"round 1/5: full pedestrian matrix ({robot_engine}, checkpointed)...")
            robot_mat, _snapped_walk = _robot_full_matrix(points, out_dir)
        else:
            print(f"round {_round + 1}/5: refreshing {len(bad)} resampled point(s)...")
            _refresh_osrm(truck_mat, OSRM_TABLE_URL, OSRM_BLOCK, "driving", bad)
            if robot_engine == "valhalla":
                for i in bad:
                    row, col = valhalla_point_rowcol(points, i)
                    for j in range(len(points)):
                        robot_mat[i][j] = row[j]
                        robot_mat[j][i] = col[j]
            elif robot_engine == "local":
                # local engine: full recompute is cheap (in-memory Dijkstra)
                robot_mat, _snapped_walk = local_engine.matrix(points)
                for gj, pos in enumerate(_snapped_walk):
                    snapped[gj] = pos
            else:
                _refresh_osrm(robot_mat, OSRM_FOOT_TABLE_URL, FOOT_BLOCK, "foot", bad)
        if not _rowcol_ok(truck_mat, 0) or not _rowcol_ok(robot_mat, 0):
            raise RuntimeError("depot location is not routable on the road/pedestrian network")
        bad_routing = [i for i in range(1, len(points))
                       if not (_rowcol_ok(truck_mat, i) and _rowcol_ok(robot_mat, i))]
        # Snapped de-collision: engines can snap distinct sampled points onto
        # the same network location (zero-distance pairs). Treat those like
        # unroutable points, with a wider exclusion so they cannot collapse
        # onto the same spot again (snap displacement is typically < 100 m).
        seen: list[tuple[float, float]] = [snapped[0]]
        bad_collide: list[int] = []
        for k in range(1, len(points)):
            if any(haversine_km(snapped[k][0], snapped[k][1], la, lo) < SNAP_COLLIDE_KM
                   for la, lo in seen):
                bad_collide.append(k)
            else:
                seen.append(snapped[k])
        bad = sorted(set(bad_routing) | set(bad_collide))
        if not bad:
            break
        print(f"resampling {len(bad)} point(s) "
              f"({len(bad_routing)} unroutable, {len(bad_collide)} snapped-collapsed)...")
        for i in bad:
            exclusion = max(min_sep_km, 0.25) if i in bad_collide else min_sep_km
            for _ in range(20000):
                lat = rng.uniform(south, north)
                lon = rng.uniform(west, east)
                others = [p for k, p in enumerate(points) if k != i]
                if all(haversine_km(lat, lon, la, lo) >= exclusion for la, lo in others):
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

    if robot_engine == "osrm-foot":
        robot_provenance = ("OSRM table service (foot profile) on the OSM walk network, "
                            "via routing.openstreetmap.de (FOSSGIS)")
        robot_assumption = ("Robot travel: OSRM foot-profile shortest-path distances on the OSM "
                            "walk network at 6 km/h.")
    elif robot_engine == "local":
        robot_provenance = (f"local OSM walk network parsed from {Path(osm_pbf).name} "
                            "(Geofabrik Illinois extract) with pyrosm; all-pairs shortest "
                            "paths computed in-process (Dijkstra on the undirected walk graph)")
        robot_assumption = ("Robot travel: shortest-path pedestrian distances on the OSM "
                            "walk network at 6 km/h, computed fully offline.")
    else:
        robot_provenance = ("Valhalla sources_to_targets (pedestrian costing) on the OSM walk "
                            "network, via valhalla1.openstreetmap.de (FOSSGIS)")
        robot_assumption = ("Robot travel: Valhalla shortest-path pedestrian distances on the OSM "
                            "walk network at 6 km/h.")

    params = dict(REALISTIC_PARAMS)
    params.update(
        {
            "source_dataset": "OSM real-world",
            "source_place": place_label,
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "generator": "scripts/build_real_world_instance.py",
            "generator_seed": seed,
            "robot_engine": robot_engine,
            "osm_pbf": Path(osm_pbf).name if osm_pbf else None,
            "NUM_CUSTOMERS": n,
            "NUM_TRUCKS": max(2 if n <= 12 else 3, math.ceil(n / 30)),
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
                f"robot pedestrian distances: {robot_provenance}; "
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
                robot_assumption,
                "Drone travel: haversine great-circle distance at 80 km/h (unrestricted airspace).",
                "Canonical end depot n+1 duplicates depot 0.",
            ],
            "warnings": [
                "Truck speed is constant; OSM maxspeed-based travel times are a future refinement.",
                "Drone flight is straight-line; no-fly zones and wind are not modeled.",
                (
                    "Truck road distances come from the public OSRM demo server "
                    "(router.project-osrm.org); for publication-grade instances use a "
                    "self-hosted OSRM engine. Robot pedestrian distances are computed "
                    "fully offline from a local OSM extract."
                    if robot_engine == "local" else
                    "Routing data comes from public demo servers (OSRM, Valhalla); for publication-grade "
                    "instances use a self-hosted engine or a local OSM extract."
                ),
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
        "robot_routing_source": robot_provenance,
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
    parser.add_argument("--min-sep-km", type=float, default=0.15,
                        help="minimum separation between sampled customer points (default 0.15)")
    parser.add_argument("--robot-engine", choices=["valhalla", "osrm-foot", "local"], default="valhalla",
                        help="pedestrian routing engine for robot matrices "
                             "(local = fully offline from a .osm.pbf via --osm-pbf)")
    parser.add_argument("--osm-pbf", default=None,
                        help="path to a .osm.pbf extract (required for --robot-engine local)")
    args = parser.parse_args()
    if args.place is None and args.bbox is None:
        parser.error("one of --place or --bbox is required")
    bbox = tuple(float(v) for v in args.bbox.split(",")) if args.bbox else None
    depot_latlon = tuple(float(v) for v in args.depot_latlon.split(",")) if args.depot_latlon else None
    build_instance(args.place, bbox, args.n_customers, args.seed, args.out_dir, depot_latlon,
                   min_sep_km=args.min_sep_km, robot_engine=args.robot_engine,
                   osm_pbf=args.osm_pbf)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
