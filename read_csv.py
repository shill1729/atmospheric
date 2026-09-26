"""Load a synthetic export: the data table and its '#' metadata header.

Usage from Python:

    from read_csv import load_export
    data, meta = load_export()             # newest file in exports/
    data, meta = load_export("exports/atmospheric_....csv")
    meta["sources"], meta["estimates"], meta["settings"]

From the command line:

    python read_csv.py [path]              # prints a summary of the export
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

import pandas as pd

EXPORT_DIR = Path(__file__).resolve().parent / "exports"

_KEY_VALUE = re.compile(r"(\S+?)=(\S+)")
_SECONDS = re.compile(r"^[+-]?\d+(?:\.\d+)?s$")


def latest_export(export_dir: str | Path = EXPORT_DIR) -> Path:
    """Most recently modified .csv in export_dir."""
    files = sorted(Path(export_dir).glob("*.csv"), key=lambda p: p.stat().st_mtime)
    if not files:
        raise FileNotFoundError(f"No .csv exports found in {export_dir}")
    return files[-1]


def _value(text: str):
    """Convert a header value: ints, floats (a trailing 's' unit is dropped), booleans, else str."""
    if _SECONDS.match(text):
        text = text[:-1]
    if text in ("true", "false"):
        return text == "true"
    if re.fullmatch(r"[+-]?\d+", text):
        return int(text)
    try:
        return float(text)
    except ValueError:
        return text


def read_metadata(path: str | Path) -> dict:
    """Parse the '#' header into settings, sources and estimates.

    settings:  dict of the "# Key: value" lines (wind model, dt, time stretch, ...)
    sources:   DataFrame, one row per "# Source N:" line (true release positions/times)
    estimates: DataFrame, one row per "# Estimate N (method):" line, including
               err_m / nearest_source when the export recorded any sources
    """
    settings, sources, estimates = {}, [], []
    with open(path, encoding="utf-8") as f:
        for line in f:
            if not line.startswith("#"):
                break
            line = line[1:].strip()
            if m := re.match(r"Source (\d+):\s*(.*)", line):
                row = {"source_id": int(m.group(1))}
                row.update({k: _value(v) for k, v in _KEY_VALUE.findall(m.group(2))})
                sources.append(row)
            elif m := re.match(r"Estimate (\d+) \((.+?)\):\s*(.*)", line):
                row = {"estimate_id": int(m.group(1)), "method": m.group(2)}
                if m.group(3).startswith("no result"):
                    row["message"] = m.group(3)
                else:
                    row.update({k: _value(v) for k, v in _KEY_VALUE.findall(m.group(3))})
                estimates.append(row)
            elif ":" in line and not line.startswith(("Sources:", "Source term estimates")):
                key, _, value = line.partition(":")
                value = value.strip()
                # A leading number is kept as the value ("50.55 real s per ..."), else the full text.
                first = _value(value.split(" ")[0]) if value else ""
                settings[key.strip()] = first if isinstance(first, (int, float)) else value
    return {"settings": settings, "sources": pd.DataFrame(sources), "estimates": pd.DataFrame(estimates)}


def load_export(path: str | Path | None = None) -> tuple[pd.DataFrame, dict]:
    """Return (data, meta) for an export; the newest one in exports/ if path is None.

    `data` has the same columns as wildfire_pm25_dataset.csv (plus time_s), with
    Datetime_UTC parsed. `meta` is read_metadata()'s dict plus the file's "path".
    """
    path = Path(path) if path is not None else latest_export()
    data = pd.read_csv(path, comment="#", parse_dates=["Datetime_UTC"])
    meta = read_metadata(path)
    meta["path"] = path
    return data, meta


if __name__ == "__main__":
    data, meta = load_export(sys.argv[1] if len(sys.argv) > 1 else None)
    print(f"File: {meta['path']}\n")
    print(data.head(), "\n")
    print("Sources:\n", meta["sources"].to_string(index=False) if not meta["sources"].empty else "  none", "\n")
    print("Estimates:\n", meta["estimates"].to_string(index=False) if not meta["estimates"].empty else "  none")
