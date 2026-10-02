"""Offline pedestrian distance matrix from a local OSM extract (pyrosm).

Replaces the Valhalla / OSRM-foot *servers* for robot (sidewalk) distances when
those public endpoints throttle or are otherwise unusable. Everything runs
locally: the walking network is filtered from an .osm.pbf extract with pyrosm
and all-pairs shortest paths are computed in-process with Dijkstra.

Public entry points:
    LocalPedestrianEngine(pbf_path, bbox)   # parse once, reuse across rounds
        .matrix(points) -> (dist_km, snapped)
    local_pedestrian_matrix(points, pbf_path, pad_deg=0.03)   # one-shot helper

    points    : list of (lat, lon) tuples, length n
    pbf_path  : path to an .osm.pbf extract covering the points
    bbox      : (south, north, west, east) degrees covered by the network
    returns (dist_km, snapped)
        dist_km   : n x n list of lists; None where unroutable
        snapped  : list of (lat, lon) of the nearest walk-network node
"""

import heapq
import math

import numpy as np


def _haversine_km(lat1, lon1, lat2, lon2):
    r = 6371.0088
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


class LocalPedestrianEngine:
    """Walking network parsed once from a local .osm.pbf; Dijkstra on demand."""

    def __init__(self, pbf_path, bbox):
        from pyrosm import OSM

        south, north, west, east = bbox
        # pyrosm bounding_box order: [west, south, east, north]
        osm = OSM(str(pbf_path), bounding_box=[west, south, east, north])
        nodes, edges = osm.get_network(network_type="walking", nodes=True)

        self.node_lat = nodes["lat"].to_numpy()
        self.node_lon = nodes["lon"].to_numpy()
        id2idx = {int(nid): k for k, nid in enumerate(nodes["id"].to_numpy())}

        # undirected adjacency (pedestrians ignore one-way restrictions)
        adj = [[] for _ in range(len(id2idx))]
        eu = edges["u"].to_numpy()
        ev = edges["v"].to_numpy()
        elen = edges["length"].to_numpy()  # metres
        for u, v, w in zip(eu, ev, elen):
            iu = id2idx.get(int(u))
            iv = id2idx.get(int(v))
            if iu is None or iv is None or iu == iv:
                continue
            w = float(w)
            if not (w > 0):
                continue
            adj[iu].append((iv, w))
            adj[iv].append((iu, w))
        self.adj = adj
        self.n_nodes = len(id2idx)

        # Keep only the largest connected component: delivery robots operate
        # on the main walk network, and points snapping to isolated footpath
        # islands would otherwise be spuriously "unroutable".
        seen = bytearray(len(adj))
        best: list[int] = []
        for s in range(len(adj)):
            if seen[s]:
                continue
            seen[s] = 1
            stack = [s]
            comp = [s]
            while stack:
                u = stack.pop()
                for v, _w in adj[u]:
                    if not seen[v]:
                        seen[v] = 1
                        stack.append(v)
                        comp.append(v)
            if len(comp) > len(best):
                best = comp
        keep = set(best)
        remap = {old: new for new, old in enumerate(best)}
        self.node_lat = self.node_lat[best]
        self.node_lon = self.node_lon[best]
        self.adj = [[(remap[v], w) for v, w in adj[old] if v in keep]
                    for old in best]
        self.n_nodes = len(best)
        self.n_dropped_components = len(adj) - len(best)

    def matrix(self, points):
        nlat = self.node_lat
        nlon = self.node_lon
        lats = [p[0] for p in points]
        lons = [p[1] for p in points]

        # snap each point to its nearest walk-network node (vectorised)
        plat = np.radians(np.array(lats))[:, None]
        nlat_r = np.radians(nlat)[None, :]
        dlat = nlat_r - plat
        dlon = (nlon[None, :] - np.array(lons)[:, None]) * np.cos(plat)
        nearest = np.argmin(dlat * dlat + dlon * dlon, axis=1)
        del plat, nlat_r, dlat, dlon
        snapped = [(float(nlat[i]), float(nlon[i])) for i in nearest]
        src_nodes = [int(i) for i in nearest]
        n = len(points)
        targets = set(src_nodes)

        # Dijkstra from every source, stopping once all targets are settled
        INF = float("inf")
        adj = self.adj
        dist_km = [[None] * n for _ in range(n)]
        for si, s in enumerate(src_nodes):
            dist = {s: 0.0}
            pq = [(0.0, s)]
            found = {}
            while pq and len(found) < len(targets):
                d, u = heapq.heappop(pq)
                if d > dist.get(u, INF):
                    continue
                if u in targets and u not in found:
                    found[u] = d
                    if len(found) == len(targets):
                        break
                for v, w in adj[u]:
                    nd = d + w
                    if nd < dist.get(v, INF):
                        dist[v] = nd
                        heapq.heappush(pq, (nd, v))
            row = dist_km[si]
            for ti, t in enumerate(src_nodes):
                if t in found:
                    row[ti] = found[t] / 1000.0
            row[si] = 0.0
        return dist_km, snapped


def local_pedestrian_matrix(points, pbf_path, pad_deg=0.03):
    lats = [p[0] for p in points]
    lons = [p[1] for p in points]
    bbox = (min(lats) - pad_deg, max(lats) + pad_deg,
            min(lons) - pad_deg, max(lons) + pad_deg)
    eng = LocalPedestrianEngine(pbf_path, bbox)
    return eng.matrix(points)
