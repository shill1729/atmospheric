"""Loaders for the two real PM2.5 datasets used to calibrate the synthetic model.

Both files share a common core schema (Datetime_UTC, site_name, lat_deg, lon_deg,
pm25_ugm-3, wind_speed, wind_direction, wind_u_component, wind_v_component);
multimonth_pm25_dataset.csv additionally carries NO2/NO/O3/CO columns that this
project's PM2.5-only forward model does not need.
"""
from __future__ import annotations

from pathlib import Path

import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent

WILDFIRE_CSV = REPO_ROOT / "wildfire_pm25_dataset.csv"
MULTIMONTH_CSV = REPO_ROOT / "multimonth_pm25_dataset.csv"

CORE_COLUMNS = [
    "Datetime_UTC",
    "site_name",
    "lat_deg",
    "lon_deg",
    "pm25_ugm-3",
    "wind_speed",
    "wind_direction",
    "wind_u_component",
    "wind_v_component",
]


def _load(path: Path) -> pd.DataFrame:
    if not path.exists():
        raise FileNotFoundError(
            f"{path} not found. This module expects the real datasets to live "
            "at the repo root as documented in README.md."
        )
    df = pd.read_csv(path, parse_dates=["Datetime_UTC"])
    df = df.sort_values(["site_name", "Datetime_UTC"]).reset_index(drop=True)
    return df


def load_wildfire() -> pd.DataFrame:
    """Nov 2024 NY-metro wildfire smoke event: 38 sites, hourly, ~4 days."""
    return _load(WILDFIRE_CSV)


def load_multimonth() -> pd.DataFrame:
    """Sep 2023-Mar 2024 multi-pollutant record: 29 sites, hourly, ~6 months."""
    return _load(MULTIMONTH_CSV)


def load_all() -> dict[str, pd.DataFrame]:
    return {"wildfire": load_wildfire(), "multimonth": load_multimonth()}
