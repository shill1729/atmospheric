"""Numpy replica of the C++ PDE step, used to calibrate --conc-scale.

The C++ solver works in model concentration units, converted for display and
export by conc_ug_m3 = model_conc * conc_scale / mixing_height. This module
runs the same upwind-advection / scalar-diffusion update (ConstantScalar
preset) with the default Config.hpp parameters to find the raw peak of a
default source burst; solve_conc_scale then maps that peak to a real one.
"""
from __future__ import annotations

import numpy as np


def simulate_default_plume(
    nx: int = 140,
    ny: int = 140,
    x_min: float = 0.0,
    x_max: float = 5000.0,
    y_min: float = 0.0,
    y_max: float = 5000.0,
    kappa: float = 6.0,
    wind_u: float = 2.5,
    wind_v: float = 0.5,
    deposition_rate: float = 0.01,
    source_sigma: float = 50.0,
    base_emission: float = 1.0,
    decay_rate: float = 0.03,
    lifespan: float = 100.0,
    dt: float = 0.05,
    n_steps: int = 4000,
) -> dict:
    """Run the default source burst forward and return raw model concentration stats.

    Mirrors AdvectionDiffusionSolver::step's conservative upwind advection +
    scalarized diffusion (kappa = 0.5*trace(D) with D = kappa*I here) and
    SourceModel's single decaying Gaussian source, on a uniform grid with
    absorbing boundaries.
    """
    dx = (x_max - x_min) / (nx - 1)
    dy = (y_max - y_min) / (ny - 1)
    x = np.linspace(x_min, x_max, nx)
    y = np.linspace(y_min, y_max, ny)
    xx, yy = np.meshgrid(x, y, indexing="xy")  # yy varies by row (j), xx by column (i)

    source_pos = np.array([x_min + 0.5 * (x_max - x_min), y_min + 0.5 * (y_max - y_min)])
    r2 = (xx - source_pos[0]) ** 2 + (yy - source_pos[1]) ** 2
    source_shape = np.exp(-r2 / (2.0 * source_sigma ** 2)) / (2.0 * np.pi * source_sigma ** 2)

    c = np.zeros((ny, nx), dtype=np.float64)
    peak_history = np.zeros(n_steps)

    u, v = wind_u, wind_v  # spatially uniform representative wind for this scale estimate

    for step in range(n_steps):
        t = step * dt
        q_t = base_emission * np.exp(-decay_rate * t) if t <= lifespan else 0.0
        s = q_t * source_shape

        c_pad = np.pad(c, 1, mode="constant", constant_values=0.0)  # absorbing boundary

        flux_e = u * np.where(u >= 0, c_pad[1:-1, 1:-1], c_pad[1:-1, 2:])
        flux_w = u * np.where(u >= 0, c_pad[1:-1, :-2], c_pad[1:-1, 1:-1])
        flux_n = v * np.where(v >= 0, c_pad[1:-1, 1:-1], c_pad[2:, 1:-1])
        flux_s = v * np.where(v >= 0, c_pad[:-2, 1:-1], c_pad[1:-1, 1:-1])
        adv = -(flux_e - flux_w) / dx - (flux_n - flux_s) / dy

        lap_x = (c_pad[1:-1, 2:] - 2 * c_pad[1:-1, 1:-1] + c_pad[1:-1, :-2]) / dx ** 2
        lap_y = (c_pad[2:, 1:-1] - 2 * c_pad[1:-1, 1:-1] + c_pad[:-2, 1:-1]) / dy ** 2
        diff = kappa * (lap_x + lap_y)

        c = np.maximum(0.0, c + dt * (adv + diff + s - deposition_rate * c))
        peak_history[step] = c.max()

    return {
        "peak_model_conc": float(c.max()),
        "mean_positive_model_conc": float(c[c > 0].mean()) if np.any(c > 0) else 0.0,
        "quasi_steady_peak": float(peak_history[-200:].mean()),
        "peak_history": peak_history,
    }


def solve_conc_scale(raw_peak_model_conc: float, target_peak_ugm3: float, mixing_height_m: float) -> float:
    """Invert conc_ug_m3 = model_conc * conc_scale / mixing_height for conc_scale."""
    if raw_peak_model_conc <= 0.0:
        raise ValueError("raw_peak_model_conc must be positive")
    return target_peak_ugm3 * mixing_height_m / raw_peak_model_conc
