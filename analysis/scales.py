"""Infer spatial, temporal, concentration, and wind scales from real PM2.5 data.

Every function here takes a dataframe shaped like analysis.io's loaders and
returns a plain dict of scalars, so results can be printed, unit-tested, or
fed into analysis.forward_model for translation into simulator config values.
"""
from __future__ import annotations

import numpy as np
import pandas as pd

EARTH_RADIUS_M = 6_371_000.0


def domain_projection_scale(df: pd.DataFrame, domain_span_m: float = 5000.0, pad_fraction: float = 0.08) -> float:
    """Domain metres per real metre when the site network is fitted into the domain.

    Mirrors project_sites_to_domain in src/io/SiteLoader.cpp: equirectangular
    projection about the site centroid, then a uniform scale fitting the
    network's bounding box inside the domain minus pad_fraction on each side.
    """
    sites = df.drop_duplicates("site_name")
    lat0 = np.radians(sites["lat_deg"].mean())
    x = np.radians(sites["lon_deg"] - sites["lon_deg"].mean()) * np.cos(lat0) * EARTH_RADIUS_M
    y = np.radians(sites["lat_deg"] - sites["lat_deg"].mean()) * EARTH_RADIUS_M
    avail = domain_span_m * (1.0 - 2.0 * pad_fraction)
    return float(min(avail / np.ptp(x), avail / np.ptp(y)))


def haversine_m(lat1, lon1, lat2, lon2) -> np.ndarray:
    """Great-circle distance in meters between paired (or broadcastable) lat/lon arrays."""
    lat1, lon1, lat2, lon2 = map(np.radians, (lat1, lon1, lat2, lon2))
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = np.sin(dlat / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS_M * np.arcsin(np.sqrt(np.clip(a, 0.0, 1.0)))


def site_spacing_stats(df: pd.DataFrame) -> dict:
    """Nearest-neighbor spacing and overall network extent, in meters.

    This is descriptive only: the app rescales real site geometry to fit
    whatever --grid-nx/--x-max domain is configured (see SiteLoader::project_sites_to_domain),
    so these numbers inform *relative* spacing/extent ratios, not domain size directly.
    """
    sites = df[["site_name", "lat_deg", "lon_deg"]].drop_duplicates("site_name").reset_index(drop=True)
    n = len(sites)
    lat = sites["lat_deg"].to_numpy()
    lon = sites["lon_deg"].to_numpy()

    nn = np.full(n, np.inf)
    max_extent = 0.0
    for i in range(n):
        d = haversine_m(lat[i], lon[i], lat, lon)
        d[i] = np.inf
        nn[i] = d.min()
        finite_max = np.max(np.where(np.isfinite(d), d, 0.0))
        max_extent = max(max_extent, finite_max)

    return {
        "n_sites": n,
        "nearest_neighbor_median_m": float(np.median(nn)),
        "nearest_neighbor_min_m": float(np.min(nn)),
        "network_extent_m": float(max_extent),
    }


def concentration_stats(df: pd.DataFrame, background_quantile: float = 0.1) -> dict:
    c = df["pm25_ugm-3"].dropna()
    background = float(c.quantile(background_quantile))
    peak = float(c.max())
    return {
        "background_ugm3": background,
        "median_ugm3": float(c.median()),
        "p90_ugm3": float(c.quantile(0.90)),
        "p95_ugm3": float(c.quantile(0.95)),
        "p99_ugm3": float(c.quantile(0.99)),
        "peak_ugm3": peak,
        "peak_to_background_ratio": peak / background if background > 0 else float("nan"),
    }


def sampling_interval_s(df: pd.DataFrame) -> float:
    """Median reporting interval, in seconds, across all sites."""
    diffs = (
        df.sort_values(["site_name", "Datetime_UTC"])
        .groupby("site_name")["Datetime_UTC"]
        .diff()
        .dropna()
        .dt.total_seconds()
    )
    return float(diffs.median())


def event_timescales(df: pd.DataFrame, background_quantile: float = 0.1) -> dict:
    """Rise/decay e-folding times of a network-mean concentration spike.

    Fits log(c - background) ~ t linearly on either side of the network-mean
    peak to get exponential rise/decay time constants, matching the
    q(t) = q0 * exp(-gamma * (t - t0)) source-decay form used by SourceModel.
    """
    agg = df.groupby("Datetime_UTC")["pm25_ugm-3"].mean().sort_index()
    t_s = (agg.index - agg.index[0]).total_seconds().to_numpy()
    c = agg.to_numpy()
    background = float(np.quantile(c, background_quantile))

    peak_idx = int(np.argmax(c))
    peak_val = float(c[peak_idx])
    peak_t = float(t_s[peak_idx])

    def _fit_tau(t_seg: np.ndarray, c_seg: np.ndarray) -> float | None:
        excess = c_seg - background
        mask = excess > 0.05 * (peak_val - background)
        if mask.sum() < 3:
            return None
        slope, _intercept = np.polyfit(t_seg[mask], np.log(excess[mask]), 1)
        if slope == 0:
            return None
        return float(abs(1.0 / slope))

    rise_tau_s = _fit_tau(t_s[: peak_idx + 1], c[: peak_idx + 1])
    decay_tau_s = _fit_tau(t_s[peak_idx:] - peak_t, c[peak_idx:])

    above = c > (background + 0.2 * (peak_val - background))
    event_duration_s = float(t_s[above].max() - t_s[above].min()) if above.any() else 0.0

    return {
        "background_ugm3": background,
        "peak_ugm3": peak_val,
        "peak_time_utc": str(agg.index[peak_idx]),
        "rise_tau_s": rise_tau_s,
        "decay_tau_s": decay_tau_s,
        "event_duration_s": event_duration_s,
    }


def autocorrelation_time_s(df: pd.DataFrame, site: str | None = None, max_lag_hours: int = 48) -> float:
    """Lag (seconds) at which the mean series' autocorrelation first drops below 1/e."""
    series = df if site is None else df[df["site_name"] == site]
    s = series.groupby("Datetime_UTC")["pm25_ugm-3"].mean().sort_index()
    dt_s = sampling_interval_s(df)
    x = s.to_numpy() - s.mean()
    n = len(x)
    max_lag = min(max_lag_hours, n - 2)
    acf = np.array([1.0] + [
        float(np.dot(x[:-lag], x[lag:]) / np.dot(x, x)) for lag in range(1, max_lag + 1)
    ])
    below = np.where(acf < 1.0 / np.e)[0]
    lag_steps = int(below[0]) if len(below) else max_lag
    return lag_steps * dt_s


def wind_stats(df: pd.DataFrame) -> dict:
    w = df.dropna(subset=["wind_speed"])
    if w.empty:
        return {}
    return {
        "speed_median_m_s": float(w["wind_speed"].median()),
        "speed_p90_m_s": float(w["wind_speed"].quantile(0.90)),
        "speed_max_m_s": float(w["wind_speed"].max()),
        "u_std_m_s": float(w["wind_u_component"].std()),
        "v_std_m_s": float(w["wind_v_component"].std()),
    }
