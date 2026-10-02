#!/usr/bin/env python3
"""Time-dependent truck travel times for the real-world instance generator.

Implements the classic *speed-profile* construction (Ichoua, Gendreau & Semet,
"How to deal with traveling salesman and vehicle routing problems with time
windows," EJOR 144(3), 2003; widely reused e.g. Balseiro, Loiseau & Ramonet 2011;
Figliozzi 2012):

- The working day [0, T] is split into periods, each with a speed factor
  f_p in (0, 1] relative to free-flow speed.
- Travel time for a departure at time tau0 is computed by *integrating speeds*:
  find the smallest T with  integral_{tau0}^{tau0+T} f(tau) dtau = t_static,
  where t_static is the free-flow travel time of the OD pair.

Because speeds (not travel times) are the primitive, the FIFO property holds
by construction: departing later can never arrive earlier.  A checker
(`verify_fifo`) is provided and run by the generator.

Speed-factor magnitudes follow Figliozzi (2012, TRE 48(3)): at most 2.5:1
between the fastest and slowest period.  Profiles are defined as
resolution-independent piecewise segments over fractions of the working day
so any number of periods can be discretized from them.

Only the truck layer is time-dependent.  Robot (pedestrian) congestion data
does not exist publicly and drone airspace is uncongested; both stay static.
This is recorded in the instance provenance.
"""

from __future__ import annotations

import numpy as np

# Each profile: list of (day_fraction_start, day_fraction_end, speed_factor).
# speed_factor = fraction of free-flow speed in that slice.
PROFILES: dict[str, list[tuple[float, float, float]]] = {
    # Two-peak urban day, max slowdown 2:1 (within Figliozzi's 2.5:1 bound).
    "two_peak": [
        (0.00, 0.12, 1.00),
        (0.12, 0.28, 0.62),
        (0.28, 0.45, 0.92),
        (0.45, 0.58, 1.00),
        (0.58, 0.75, 0.50),
        (0.75, 1.00, 0.95),
    ],
    # Mild small-city congestion (e.g. DeKalb, IL): peaks barely bite.
    "mild": [
        (0.00, 0.15, 1.00),
        (0.15, 0.30, 0.85),
        (0.30, 0.55, 1.00),
        (0.55, 0.75, 0.80),
        (0.75, 1.00, 1.00),
    ],
    # Static baseline (all factors 1.0) for controlled comparisons.
    "flat": [(0.00, 1.00, 1.00)],
}


def discretize_profile(profile: str, n_periods: int, horizon: float = 8.0
                       ) -> tuple[np.ndarray, np.ndarray]:
    """Return (period_bounds, speed_factors) for n equal periods over [0, horizon].

    Factors are sampled at each period midpoint from the named profile.
    """
    if profile not in PROFILES:
        raise ValueError(f"unknown td profile {profile!r}; choose from {sorted(PROFILES)}")
    if n_periods < 1:
        raise ValueError("n_periods must be >= 1")
    segs = PROFILES[profile]
    bounds = np.linspace(0.0, horizon, n_periods + 1)
    mids = (bounds[:-1] + bounds[1:]) / 2.0 / horizon  # as day fractions
    factors = np.empty(n_periods)
    for k, m in enumerate(mids):
        for s0, s1, f in segs:
            if s0 <= m < s1 or (m == 1.0 and s1 == 1.0):
                factors[k] = f
                break
        else:  # pragma: no cover
            raise RuntimeError("profile does not cover the full day")
    if np.any(factors <= 0):
        raise ValueError("speed factors must be positive")
    return bounds, factors


def period_travel_matrices(t_static: np.ndarray, bounds: np.ndarray,
                           factors: np.ndarray) -> list[np.ndarray]:
    """FIFO-safe time-dependent travel-time matrices, one per departure period.

    t_static: (n, n) free-flow travel times in hours.
    Returns list of (n, n) arrays; entry [p][i, j] is the travel time in hours
    when departing node i at the *start* of period p, computed by integrating
    the stepwise speed profile (Ichoua et al. 2003).  The daily profile repeats
    cyclically for trips running past the horizon.
    """
    n = t_static.shape[0]
    n_periods = len(factors)
    period_len = np.diff(bounds)
    out: list[np.ndarray] = []
    for p in range(n_periods):
        remaining = t_static.copy()  # free-flow hours still to "consume"
        elapsed = np.zeros_like(t_static)
        cur = p
        # At most n_periods+1 iterations needed unless a trip outlasts a full
        # day of the slowest speed; loop generously and break when done.
        for _ in range(4 * n_periods + 4):
            active = remaining > 1e-12
            if not np.any(active):
                break
            # Free-flow hours consumable in the current period slice:
            capacity = period_len[cur] * factors[cur]
            # Time needed in this slice to finish:
            need = np.where(active, remaining / factors[cur], 0.0)
            finish_here = active & (need <= period_len[cur] + 1e-12)
            elapsed[finish_here] += need[finish_here]
            remaining[finish_here] = 0.0
            move_on = active & ~finish_here
            elapsed[move_on] += period_len[cur]
            remaining[move_on] -= capacity
            cur = (cur + 1) % n_periods
        out.append(elapsed)
    return out


def verify_fifo(matrices: list[np.ndarray], bounds: np.ndarray,
                rtol: float = 1e-9) -> bool:
    """Check FIFO: arrival time is nondecreasing in departure time.

    Departures are at period starts; arrival_p[i, j] = bounds[p] + matrices[p][i, j]
    must satisfy arrival_p <= arrival_{p+1} for every pair, measuring the wrapped
    pair (last -> first) on the next day's timeline (bounds[0] + horizon).
    """
    horizon = float(bounds[-1])
    n_periods = len(matrices)
    for p in range(n_periods):
        q = (p + 1) % n_periods
        arr_p = bounds[p] + matrices[p]
        arr_q = bounds[q] + matrices[q] + (horizon if q == 0 else 0.0)
        if np.any(arr_q + rtol < arr_p):
            return False
    return True


def profile_summary(profile: str, n_periods: int, horizon: float = 8.0) -> dict:
    bounds, factors = discretize_profile(profile, n_periods, horizon)
    return {
        "td_profile": profile,
        "td_periods": n_periods,
        "td_horizon": horizon,
        "td_period_bounds": [round(float(b), 4) for b in bounds],
        "td_speed_factors": [round(float(f), 4) for f in factors],
        "td_max_slowdown_ratio": round(float(1.0 / factors.min()), 3),
        "td_method": "Ichoua-Gendreau-Semet (2003) speed-profile integration; "
                     "FIFO holds by construction (speeds are the primitive).",
        "td_citations": [
            "Ichoua, Gendreau & Semet (2003), EJOR 144(3)",
            "Figliozzi (2012), TRE 48(3) (2.5:1 max ratio bound)",
            "Balseiro, Loiseau & Ramonet (2011), C&OR 38(6)",
        ],
        "td_scope": "truck layer only; robot (pedestrian) and drone layers static "
                    "(no public pedestrian-congestion data; airspace uncongested).",
    }
