# Atmospheric (SDE + PDE + Adjoint Prototype)

Interactive C++ atmospheric transport prototype with a split view:
- **Left panel:** tagged-particle simulation via SDE (Euler-Maruyama)
- **Right panel:** concentration evolution via advection-diffusion(-reaction) PDE

The two models share the same wind field, diffusivity model, source emission schedule, deposition rate, and boundary mode. On top of the forward models, a paused simulation can be inverted from sensor readings to estimate where and when a source was released (source term estimation).

## Features

- Click-to-add continuous sources with finite lifetime and exponential decay (multi-source, capped by `--source-max`)
- SDE particle transport with drift, matrix diffusion, deposition/killing, and trails
- PDE concentration transport with source and deposition, in either scalarized or full-tensor diffusion mode
- 7 wind presets and 6 diffusivity presets (scalar and SPD tensor), cycled at runtime, plus a global wind scale factor; see [Wind and diffusivity models](#wind-and-diffusivity-models)
- Brownian/heat-equation special case (zero wind, $D=\tfrac12 I$) for checking the SDE against the PDE
- Shared runtime boundary mode: periodic, reflecting, or absorbing
- HUD with the SDE-to-PDE total mass ratio $M_{\text{sde}}/M_{\text{pde}}$ as a consistency check
- PDE-panel sensors that take noisy, spatially averaged physical samples and report window averages
- Pause-time source term estimation from sensor history using three methods (adjoint backtracking, regularized least squares, Bayesian grid). All three run on each request, and `M` cycles which one is displayed. Each reports $(x^*, t^*)$ and a posterior heatmap; the latter two also report amplitude and uncertainty.
- Time acceleration up to 100,000× requested, limited in practice by a per-frame CPU budget; the HUD shows the achieved speed and the largest stable `dt` (see [Speeding up simulations](#speeding-up-simulations))
- Loading of the real NY wildfire sensor network (`L`) and a matching hourly-observation preset (`N`); the domain is georeferenced to that network at startup, so exports are in lat/lon
- CSV recording/export of the sensor time series, with source ground truth and estimates in the file header
- A Python calibration package (`analysis/`) that derives simulator defaults from real PM2.5 datasets

## Mathematical Model

### Domain, state, and source

Let the domain be

$$
\Omega = [x_{\min}, x_{\max}] \times [y_{\min}, y_{\max}] \subset \mathbb{R}^2.
$$

A synthetic source $k$ is centered at fixed location $x_k$ (clamped to $\Omega$) and emits at rate $q_k(t)$, with

$$
q_k(t) = q_0 e^{-\gamma (t-t_k)}, \quad t_k \le t < t_k + T_s,
$$

and is removed after lifespan $T_s$.

The PDE's spatial source density is a sum of Gaussian emitters:

$$
s(t,x) = \sum_{k=1}^{K(t)} q_k(t)\,\frac{1}{2\pi\sigma^2}\exp\left(-\frac{\|x-x_k\|^2}{2\sigma^2}\right).
$$

### SDE viewpoint (tagged particles)

Particles follow the It\^o SDE

$$
dX_t = \big(w(t,X_t)+\nabla\cdot D(t,X_t)\big)dt + \sqrt{2}\,D(t,X_t)^{1/2} dB_t,
$$

with deposition (killing) rate $\lambda$.

Numerically, per particle and step $\Delta t$:

$$
X_{n+1}=X_n + \mu(t_n,X_n)\Delta t + \sqrt{2\Delta t}\,L(t_n,X_n)\,\xi_n,
$$

where $\mu = w+\nabla\cdot D$, $LL^\top = D$ (Cholesky, with an eigen-decomposition fallback), and $\xi_n\sim\mathcal{N}(0,I)$ i.i.d. per particle. $\nabla\cdot D$ is evaluated by central differences.

Particle births are generated independently from each active source. Each step, a source emits $q_k(t)\cdot\texttt{particle\_scale}\cdot\Delta t$ particles in expectation (floor plus a Bernoulli draw for the fractional part). New particles are placed at $x_k + \mathcal{N}(0, (f\sigma)^2 I)$ with $f=$ `SourceConfig::particle_spread_fraction` in `include/core/Config.hpp`. The default $f=1$ samples births from $s(t,x)$ itself, the same source term the PDE uses: by Duhamel's principle, the PDE solution is a superposition of transition densities started at birth points drawn from $s$. With $f<1$ the SDE solves the Fokker–Planck equation with a narrower source than the PDE. Each particle carries mass $1/\texttt{particle\_scale}$, which is how the HUD compares SDE and PDE total mass.

Deposition is simulated with Bernoulli survival over each step (hazard $\lambda$): remove particle with probability

$$
p_{\text{kill}} = 1-e^{-\lambda\Delta t}.
$$

### PDE viewpoint (Fokker-Planck / advection-diffusion-reaction)

Concentration $c(t,x)$ evolves by

$$
\partial_t c + \nabla\cdot\big(wc - D\nabla c\big) = s(t,x)-\lambda c.
$$

In implementation, this is advanced as an explicit advection-diffusion-reaction update using:
- conservative first-order upwind advection of $-\nabla\cdot(wc)$, using face-averaged velocities (this matches the particle drift's Fokker-Planck form even when $w$ has divergence)
- selectable diffusion operator:
  - scalarized approximation using $\kappa=\tfrac12\mathrm{tr}(D)$, face-averaged
  - full tensor flux form $\nabla\cdot(D\nabla c)$ on a centered finite-difference stencil (the default)
- source addition $s(t,x)$
- deposition sink $-\lambda c$

Negative values are clamped to zero.

### Adjoint viewpoint (inverse source estimation)

To infer likely release location/time from sensor detections, the app solves the backward adjoint transport PDE:

$$
\partial_t \phi + w\cdot \nabla \phi + \nabla\cdot(D\nabla \phi) - \lambda \phi + f = 0,
\quad \phi(T,x)=0.
$$

The forcing $f(t,x)$ is built from sensor sites $\{x_i\}$ using threshold-gated Gaussian bumps:

$$
f(t,x)=\sum_i \alpha_i\,\mathbf{1}_{\tilde c(t,x_i)>c_T}\,\beta_i
\exp\!\left(-\frac{\|x-x_i\|^2}{2\sigma^2}\right).
$$

Current defaults in code (`SourceEstimationConfig` in `include/adjoint/SourceEstimator.hpp`; not exposed on the CLI):
- equal sensor weights ($\alpha_i=1$)
- bump width $\sigma = 120$ m; $\beta_i$ normalized per bump over the computational domain
- detection threshold $c_T$: at least $10^{-6}$ (model units), raised on each run to the sensors' noise floor (see [Detection threshold](#detection-threshold))
- forcing is piecewise-constant in time from recorded sensor samples (the last report at or before $t$)
- observation window: reports from the last 900 s (`max_lookback_s`) that are still in each sensor's rolling history
- release-time search horizon: the adjoint starts 900 s (`release_search_margin_s`) before the earliest observation in the window (clamped at $t=0$). A release precedes its first detection by the source-to-sensor travel time, so searching only the observation window would pin $t^*$ to its start.
- adjoint step at most 2 s, reduced automatically when the explicit stability bound for the current fields requires it

The app computes

$$
Z(t)=\int_\Omega \phi(t,x)\,dx,\quad
t^*=\arg\max_t Z(t),\quad
x^*=\arg\max_x \int_0^T p(t,x)\,dt,\quad
p=\phi/Z.
$$

The time integral is trapezoidal over the adjoint snapshots, and the result is renormalized to a spatial density before the argmax.

This is `SourceEstimationMethod::AdjointBacktracking`, the summed-forcing heuristic above. It uses detections only and ignores reading magnitude.

### Regularized least squares and Bayesian grid (linear source-receptor inversion)

Both remaining methods (`RegularizedLeastSquares`, `BayesianGrid`) are built on the same linear source-receptor model. By adjoint reciprocity, solving the backward adjoint PDE once per sensor $i$, forced by a unit-weight bump at that sensor's own gated observation times, yields a sensitivity field $h_i(x,t)$. A unit-amplitude source released at $(x,t)$ is then predicted to produce a response

$$
y_i \approx q_0\, h_i(x,t)
$$

at sensor $i$, where $y_i=\sum_{k:\,\tilde c_i(t_k)>c_T} \tilde c_i(t_k)$ is that sensor's own sum of gated noisy readings over the observation window. Each sensor's adjoint solution is projected (nearest snapshot in time, bilinear in space) onto a coarse `search_grid_nx`$\times$`search_grid_ny`$\times$`search_time_count` candidate grid (default $14\times14\times24$) and trilinearly interpolated from there. The whole domain is therefore covered with $O(\#\text{sensors})$ adjoint solves rather than one solve per candidate, and `estimate_all` computes these sensitivity fields once and shares them between both methods.

**Regularized least squares.** At each candidate $(x,t)$, $q_0$ has the closed-form ridge (i.e. Bayesian-linear-regression MAP) solution, clamped to $\hat q_0 \ge 0$:

$$
\hat q_0 = \frac{\sum_i w_i h_i y_i}{\sum_i w_i h_i^2 + \rho}, \qquad
w_i = \sigma_i^{-2},\ \ \sigma_i = \varepsilon_{\text{rel}}\max(|y_i|,\epsilon),
$$

with $\rho=$ `amplitude_ridge` ($10^{-8}$) and $\varepsilon_{\text{rel}}=$ `relative_model_error` (0.05). The best `candidate_count` (5) coarse cells, ranked by weighted RSS, are refined over `refinement_levels` (3) rounds of a $9\times9\times9$ local grid search whose window shrinks by $0.4\times$ per round. Positional uncertainty comes from the profile-likelihood curvature of the weighted RSS at the optimum ($\text{std}=\sqrt{2/\partial^2_\theta \text{RSS}}$). If any curvature is non-positive, that std falls back to the full domain or window span and the result is flagged `weakly_identified`. $q_0$'s uncertainty is the linear-regression variance $1/(\sum_i w_i h_i^2 + \rho)$. Its heatmap `p_star` is the profile likelihood over the coarse spatial grid, $\exp(-\tfrac12 \min_t \text{RSS}(x,y,t))$ normalized to a density (flat prior, release time profiled out).

**Bayesian grid.** At each candidate cell, $q_0$ is marginalized out by trapezoidal quadrature (`response_quadrature_points`, default 6 nodes). The nodes span that cell's own ridge estimate $\pm 6$ standard deviations, truncated at $q_0\ge0$, because a tight $\varepsilon_{\text{rel}}$ makes the likelihood sharply peaked. This gives a marginal likelihood (evidence) per cell, computed in log space to avoid underflow, plus a per-cell posterior mean/variance for $q_0$.
- $t^*$ is the argmax of the time-marginal evidence.
- $x^*,y^*$ are the argmax of the evidence at $t^*$, refined with a standard 3-point parabolic sub-cell fit.
- The reported heatmap `p_star` is the time-integrated, spatially normalized evidence $p(x,y\mid \text{data})$ over the search grid, in the same layout as `AdjointBacktracking`.
- $x_{\text{std}},y_{\text{std}},t_{\text{std}}$ are posterior second moments, floored at the grid's own quantization noise ($\text{cell width}/\sqrt{12}$). Without the floor, a sharp likelihood would report more precision than the coarse grid can resolve.
- The result is flagged `weakly_identified` when the posterior peak is less than $3\times$ the uniform density.

Both methods reuse `AdjointSolver`; see `src/adjoint/SourceEstimator.cpp` for the implementation and `tests/self_check.cpp` for a known-source recovery regression test of all three methods.

### Detection threshold

Each physical sensor sample is $\max(0, c + \varepsilon)$ with $\varepsilon \sim \mathcal{N}(0, \sigma_n^2)$, so pure noise has a positive mean ($0.40\sigma_n$ per sample) and would exceed any fixed small threshold indefinitely. On each run the threshold is therefore raised to `SensorManager::noise_detection_floor(N)`. That is the report level which noise alone exceeds anywhere among the $N$ reports in the observation window with probability at most 5%. It uses a Chernoff bound on the mean of $m$ clamped samples ($m$ = report period / physical period), which is conservative. With the defaults (10 sensors, 120 reports each, $\sigma_n = 5.4\times10^{-6}$) it is about $1.1\times10^{-5}$ model units, roughly 10% of a typical plume peak. It is 0 when noise is off.

Estimation therefore refuses ("Insufficient signal") once no report in the observation window stands out from the noise. This happens, for example, after a burst has decayed (the default deposition $\lambda = 0.01\,\text{s}^{-1}$ is a 100 s e-folding time) or left an absorbing domain. It keeps estimating as long as the plume's passage is still in the window, even if the current readings are back at noise level.

## Wind and diffusivity models

Wind presets (`W` cycles them in this order; default Jet Shear). Magnitudes are calibrated to the real wind-speed distribution (median ≈ 1.9 m/s, p90 ≈ 4.4 m/s); see [Calibration](#calibration).

Every preset is multiplied by a global **wind scale** (`--wind-scale`, **Numerics → Wind Scale**; default 1, which keeps the calibration). It is a physical parameter: the advection/diffusion balance (Péclet number) shifts, and the stable `dt` shrinks roughly as 1/scale. Changes apply live, without resetting the simulation. The estimators use the same scaled wind, the HUD shows it next to the wind model, and exports record it as `# Wind scale:`.

| Preset | Description |
|---|---|
| Jet Shear | ~2 m/s westerly with time-varying sinusoidal shear |
| Vortex Pair | Counter-rotating vortex pair (peak ~10 m/s) on a ~3 m/s drift |
| Shear <-> Vortex | 160 s cycle: hold shear, ramp to vortex pair, hold, ramp back |
| Cellular Vortices | Drifting stream-function cells (~5 m/s peak) on a 2 m/s background |
| Zero Wind | $w=0$ |
| Uniform | Steady $(2.2, 0.4)$ m/s |
| Solid Body Rotation | Rigid rotation about the domain centre, $\Omega = 0.0022$ rad/s (~5 m/s at mid-edge) |

Diffusivity presets (`K` cycles them; default Constant Scalar):

| Preset | $D(t,x)$ |
|---|---|
| Constant Scalar | $\kappa I$, $\kappa = 6$ m²/s (editable as **Numerics → Const Diffusivity**) |
| Spatial Scalar | $\kappa(t,x) I$, sinusoidally varying around 8 m²/s |
| Constant Tensor | $\begin{pmatrix}6&2\\2&4\end{pmatrix}$ |
| Diagonal Tensor | Space/time-varying diagonal SPD |
| Full Anisotropic | Rotated SPD $R(\theta)\,\mathrm{diag}(\lambda_1,\lambda_2)\,R(\theta)^\top$ with varying $\theta,\lambda_i$ |
| Brownian (k=0.5) | $\tfrac12 I$ |

`H` toggles the Brownian/heat special case, which sets Zero Wind + Brownian (k=0.5) + scalarized PDE diffusion, raises particle births 12×, and zooms the left panel 2.4× around the newest source. Toggling it off restores Jet Shear + Constant Scalar.

## Initial and Boundary Conditions

### Initial conditions (IC)

- SDE: no particles until source activation (or reset)
- PDE: $c(0,x)=0$

### Boundary conditions (BC)

- **SDE:** selectable runtime mode (`B` cycles periodic → reflecting → absorbing; default periodic)
  - periodic wrapping
  - reflecting bounce
  - absorbing/outflow (particles leaving the domain are removed; births outside it are discarded)
- **PDE:** synced to the same boundary mode as SDE
  - periodic index wrapping
  - reflecting index mirroring
  - absorbing boundary with zero exterior concentration (Dirichlet-like)
- **Adjoint (source estimator):** 2D lateral inflow/outflow BCs, independent of the `B` setting
  - inflow ($w\cdot n<0$): $\phi=0$
  - outflow ($w\cdot n\ge 0$): Robin-type $D_{nn}\partial_n\phi + (w\cdot n)\phi=0$ (discretized one-sided)

## Numerical Methods Summary

### Time stepping (shared)

- Fixed internal step $\Delta t$ (`--dt`, default 0.1 s), accumulated from wall-clock time × time scale
- Each frame steps until the accumulated time is used up or `frame_step_budget_ms` (12 ms) of wall time is spent, whichever comes first (hard cap `max_substeps_per_frame` = 10,000). Time the CPU could not keep up with is dropped rather than carried over, so the achieved speed can be below the requested time scale; the HUD shows both.
- Per step: emit particles → SDE step → PDE step → age/expire sources → sensor sampling

### SDE solver

- Scheme: Euler-Maruyama
- Drift: $w+\nabla\cdot D$ (row-wise divergence of the diffusivity tensor)
- Noise: matrix diffusion via $\sqrt{2\Delta t}\,L\xi$ with $LL^\top=D$

### PDE solver

- Grid: uniform Cartesian $N_x\times N_y$ nodes (default $200\times200$ over a 5 km × 5 km domain)
- Time stepping: explicit forward Euler in time
- Spatial discretization:
  - advection: conservative first-order upwind flux form
  - diffusion: either scalarized $\kappa=\tfrac12 \mathrm{tr}(D)$ or full tensor flux $\nabla \cdot (D\nabla c)$ (default)

### Adjoint solver

- Solve direction: backward in physical time over the lookback window $[t_0,T]$, with terminal condition $\phi(T,x)=0$
- Grid: separate uniform Cartesian adjoint grid (default $96\times96$)
- Time stepping: explicit update on the adjoint grid, storing one snapshot per step; $\phi$ clamped to $\ge 0$. The step is the configured maximum (2 s), or $0.8\times$ the explicit stability bound sampled over the grid at the window's start, middle and end, whichever is smaller.
- Wind and diffusivity are evaluated once per node per step and cached for the stencils
- Advection: first-order upwind for $w\cdot\nabla\phi$
- Diffusion mode: matched to the current PDE mode (scalarized trace or full tensor flux)
- Forcing: built from recorded sensor history (threshold-gated Gaussian bumps)
- Output: time-averaged normalized density $\bar p(x)$, estimated $(x^*,t^*)$

## Dependencies

- C++20 compiler
- CMake >= 3.20
- SFML 3 (`Graphics`, `Window`, `System`)
- Eigen3
- For `analysis/` and `csv_demo.py` only: Python 3.10+ with `numpy` and `pandas`

## Build

Debug-like build:

```bash
cmake -S . -B build
cmake --build build -j
```

Release build (recommended for interactivity):

```bash
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release -j
```

## Tests

The build also produces `atmospheric_self_check` (CTest is on by default; disable it with `-DBUILD_TESTING=OFF`). It does not need SFML.

```bash
ctest --test-dir build --output-on-failure
```

It checks:
- config validation
- absorbing-boundary emission accounting
- default diffusivity positivity
- PDE source injection
- sensor history capacity/trimming
- SDE and PDE variance growth against the closed-form $2\kappa t$ rate
- known-source recovery by all three estimation methods, each producing a posterior heatmap
- noise-only sensors (no source ever placed) giving "insufficient signal" from every method

The run takes about 40 s in a Debug build.

## Run

Run from the repository root: the font (`fonts/arial.ttf`) and the NY dataset (`wildfire_pm25_dataset.csv`) are loaded by relative path.

```bash
./build-release/atmospheric
```

## Command Line Arguments

```text
--help
--window-width N
--window-height N
--x-min X
--x-max X
--y-min Y
--y-max Y
--grid-nx N
--grid-ny N
--dt X
--time-scale X
--max-particles N
--deposition X
--wind-scale X
--source-emission X
--source-decay X
--source-lifespan X
--source-sigma X
--source-max N
--sensor-physical-period X
--sensor-spatial-radius X
--sensor-history-capacity N
--conc-scale X
--mixing-height X
```

Defaults live in `include/core/Config.hpp`. The combined configuration is validated at startup, and invalid values are reported and exit with status 1.

### Speeding up simulations

One fixed step costs about 2–3 ms on the default $200\times200$ grid regardless of $\Delta t$, and about four steps fit in each frame's 12 ms budget. The simulation therefore runs at $\min(\text{requested time scale}, \sim 200\,\Delta t)$ simulated seconds per wall second, and the HUD's `speed: xR (actual xA)` line shows both. At the default `--dt 0.1` the ceiling is about 20×, so larger time scales look the same until `dt` is raised. The time scale (`[`/`]` apply immediately; **Numerics → Time Scale** after **File → Apply Queued Changes**; up to 100,000) then sets the pace.

`dt` is set with `--dt` or **PDE → dt**. The HUD's `dt: … (stable <~ X s)` line shows the explicit stability bound for the current wind and diffusivity fields. With the default presets it is several seconds, so `--dt 1` (~200×) and `--dt 2` (~400×) are stable, and the 49 h faithful-event run takes minutes. Keep `dt` at most the sensor physical sample period if every physical sample should see a distinct field. Coarser grids (`--grid-nx/--grid-ny`) make each step cheaper but lower the stability bound.

Sensors sample after every fixed step, so their readings stay correct at any speed-up.

Example:

```bash
./build-release/atmospheric \
  --window-width 1600 \
  --window-height 900 \
  --grid-nx 140 \
  --grid-ny 140 \
  --time-scale 12 \
  --max-particles 120 \
  --source-emission 1.2 \
  --conc-scale 1.0e5 \
  --mixing-height 1000
```

### Faithful NY wildfire event export

The interactive defaults (`--source-decay 0.03`, `--source-lifespan 100`) are kept fast so a clicked source evolves within seconds of wall time. The real Nov 2024 event's rise/decay time constants are much slower, ~15 h / ~42 h (see [Calibration](#calibration)). For an offline export meant to mimic that event's actual timescale, override them explicitly:

```bash
./build-release/atmospheric \
  --dt 1 \
  --time-scale 400 \
  --source-decay 6.7e-6 \
  --source-lifespan 176400 \
  --source-sigma 95 \
  --conc-scale 1.923e8 \
  --mixing-height 800
```

(`--dt`/`--time-scale` make the ~49 h event practical to run; see [Speeding up simulations](#speeding-up-simulations). The last three flags equal the current defaults and are listed so the run stays reproducible if defaults change. With the default deposition rate the plume decays within minutes, so you may also want a smaller `--deposition` for event-scale runs.)

Then press `L` to load the NY site network, `N` to apply the matching hourly-observation sensor preset, click a source, and record/export as described below.

## Runtime Controls

Keyboard:
- `Space`: pause/resume
- `R`: reset simulation (also clears sensors and estimates)
- `W`: cycle wind model
- `K`: cycle diffusivity model
- `P`: cycle PDE diffusion mode (scalarized/full tensor flux)
- `B`: cycle shared SDE/PDE boundary mode (periodic/reflecting/absorbing)
- `H`: toggle Brownian/Heat special case
- `C`: toggle PDE color scaling mode (fixed, the default, / auto)
- `U`: toggle the mass-unit label on the HUD (`g/m^2` vs `ug/m^2`)
- `[` / `]`: slower/faster simulation speed (×0.8 / ×1.25, clamped to 0.25–100,000)
- `\`: reset speed to configured base
- `E`: run source estimation (requires paused state)
- `M`: cycle which estimation method's result is displayed/HUD-reported
- `J`: cycle the right-panel overlay: selected method's posterior heatmap → backward-flow animation → off
- `L`: clear sensors and load the NY wildfire site network from `wildfire_pm25_dataset.csv`
- `N`: apply NY observation preset: physical sample period 300 s, averaging window 3600 s (mimics 5-min readings averaged to 1-hour reports); does not change domain or physics
- `F1`: open/close the controls help overlay
- `Esc`: close an open toolbar menu / cancel a field edit; otherwise open/close the preferences overlay

Mouse:
- `Left click` (left panel): add source (up to `source-max` active sources)
- `Left click` (right panel): add concentration sensor
- **Estimate** HUD card: `RUN ESTIMATE (E)` button (reads `PAUSE FIRST` while running), and `<` / `>` to cycle methods

Top toolbar menus: `File | Source | Numerics | Sensors | Display | PDE`
- `File` actions:
  - `Estimate Source (Paused)`
  - `Apply Queued Changes`
  - `Revert Queued Changes`
  - `Restore Launch Defaults` (also resets PDE diffusion mode to full tensor and recreates the simulator)
  - `Start Recording` / `Stop Recording`: toggle CSV data capture (label reflects state; `* REC` indicator appears in toolbar while active)
  - `Export CSV`: write captured sensor time series to a CSV file in the current working directory
- Editable fields:
  - `Source`: Base Emission, Decay Rate, Lifespan (s), Sigma, Max Sources
  - `Numerics`: Time Scale, Max Particles, Deposition, Const Diffusivity, Wind Scale
  - `Sensors`: Sample Period (s), Noise Std, History Capacity
  - `Display`: PDE Fixed Color Scale
  - `PDE`: Grid Nx, Grid Ny, dt, Diffusion Mode
- Edits are queued ("Queued changes pending" appears in the toolbar) and applied from `File`.
  - Changing grid size, dt, max particles, deposition, constant diffusivity, or any `Source` field **recreates the simulator**: particles, concentration, sensors, and estimates are cleared.
  - Time scale, wind scale, color scale, sensor settings, and PDE diffusion mode apply in place.
- Numeric edit UX in top menus:
  - `-` / `+` buttons for stepped adjustments. Time Scale, Wind Scale, Noise Std and PDE Fixed Color Scale step proportionally (×1.25 / ÷1.25) and display in scientific notation, since they span decades.
  - click value field to type
  - `Enter` commit, `Backspace` delete, `Esc` cancel

Preferences overlay (`Esc`), with `Up/Down` to select and `Left/Right/Enter` to adjust:
- Simulation speed, wind model, diffusivity model, PDE diffusion, boundary mode
- Wind vectors on/off (the only toggle for the left-panel wind arrows)
- Brownian/heat case, mass units, PDE color scale mode
- Particle trail length (2–180)
- Reset simulation

HUD:
- **Status** card: time, requested and achieved speed, `dt` with its stability bound, run state, models, BC, PDE max concentration (`ug/m^3`)
- **Source** card: total emission rate, emitted particles/step, active sources, particle count, newest source age, $M_{\text{sde}}/M_{\text{pde}}$
- **Estimate** card: selected method and its $x^*$, $t^*$ (and $q_0$ for the two inversion methods), and the current overlay mode
- Status strip: estimator status line, including `error=… m` (distance from $x^*$ to the most recently clicked source), recording status, and georeference / `L` / `N` status

### Source estimation workflow

1. Place sensors on the right panel (or press `L`), click a source on the left panel, and let the plume reach the sensors.
2. Pause (`Space`), then press `E` (or the **RUN ESTIMATE** button). Each method reports "Insufficient signal" if no report in the observation window exceeds the [detection threshold](#detection-threshold).
3. All three methods run. The selected method's $x^*$ is drawn as a yellow marker on both panels, and `M` (or `<`/`>`) switches between methods. The inversion methods also append `q0_std`, `wrmse`, and a `(weakly identified)` flag to the status strip.
4. The right-panel overlay (`J` cycles it) shows one of three views:
   - **posterior** (default): the selected method's `p_star` heatmap, normalized to its own maximum. Backtracking uses the 96×96 adjoint grid; the two inversion methods use the 14×14 search grid.
   - **backward flow**: an animation of backward adjoint probability flow, seeded from each sensor's most recent above-threshold report and looping over the observation window. It is a diagnostic view, separate from the three estimators, and it advances only while paused.
   - **off**

### Concentration display calibration

- Displayed PDE and sensor concentrations are reported as `ug/m^3` using:
  - `conc_display = model_concentration * conc_scale / mixing_height`
- `--conc-scale` sets `conc_scale` (ug/m^2 per model unit; default `1.923e8`).
- `--mixing-height` is the assumed vertical mixing depth (meters; default 800).
- The PDE heatmap uses a log color map. It starts in fixed mode; in auto mode it tracks 15% of the current maximum with smoothing; in fixed mode it uses **Display → PDE Fixed Color Scale**.

### Sensors

- Each sensor takes a physical sample of the PDE field every `--sensor-physical-period` (default 1 s). A sample is a 13-point disk-stencil average of radius `--sensor-spatial-radius` (default 40 m), bilinearly interpolated from the grid.
- Gaussian noise with std **Sensors → Noise Std** (model units; default `5.4e-6`, ~5% of a typical plume peak) is added to each physical sample and clamped at zero.
- Every **Sensors → Sample Period** (default 5 s) the sensor reports the window average of its noisy samples. Reports go into a rolling history of `--sensor-history-capacity` entries (default 120, i.e. 10 min at the default period). A long enough history lets the estimators see the plume's arrival at the sensors, which pins down the release time.
- On-plot labels show each sensor's latest report value (`ug/m^3`), or `N/A` before the first report.

## NY Wildfire Sensor Network

The real datasets are **not tracked in git** (`*.csv` is ignored). Place them at the repository root:
- `wildfire_pm25_dataset.csv`: hourly PM2.5 time series for 38 sensor sites across the NY metro and Hudson Valley region (Nov 2024 wildfire event). Needed for `L` and for `analysis/`.
- `multimonth_pm25_dataset.csv`: Sep 2023–Mar 2024 multi-pollutant record for 29 sites. Needed for `analysis/` only.

At startup, if `wildfire_pm25_dataset.csv` is present, the domain is georeferenced to the NY network (the same projection `L` uses). Hand-placed and `L`-loaded sensors then share one lat/lon frame in exports, and the status strip says which mode is active. Without the file, exports fall back to domain `x_m, y_m`.

Press **`L`** to load the site network into the simulation. Unique sites are read from the `site_name`, `lat_deg`, `lon_deg` columns and sorted by name. Their coordinates are projected to domain coordinates via an equirectangular approximation centred on the network centroid (lat ≈ 41.02°, lon ≈ −73.96°). The network is then uniformly scaled and centred to fit the active domain with 8% padding, preserving the relative geometry of the sites. Sensors keep their real site names as labels.

Press **`N`** after loading to apply the matching observation preset (5-min physical reads → 1-hour window averages). Combined with the CSV export, this produces a synthetic data set whose temporal structure mimics the real instrument cadence. The estimators' 900 s observation window holds at most one hourly report, so `N` is intended for export; estimation needs a shorter report period.

The `R` (reset) key always clears sensors regardless of how they were placed.

## Calibration

The `analysis/` Python package infers real-world scales from `wildfire_pm25_dataset.csv` and `multimonth_pm25_dataset.csv` and turns them into simulator defaults, so that synthetic concentrations and time scales match the real event:

- `analysis/io.py`: standardized loaders for both datasets
- `analysis/scales.py`: site-spacing, concentration, temporal (rise/decay/decorrelation), and wind statistics from real data
- `analysis/forward_model.py`: a numpy replica of `AdvectionDiffusionSolver`'s update. It translates a real concentration scale (e.g. the network-mean event peak) into a `--conc-scale` value, since that conversion has no closed form for an arbitrary wind/diffusivity field.
- `analysis/report.py`: orchestrates the above into `analysis/calibration_report.md`

Regenerate the report after either dataset changes (with a virtualenv at `.venv` that has `numpy` and `pandas`):

```bash
.venv/bin/python3 -m analysis.report
```

Several defaults were set from this report's output, and each has a comment citing the real-data quantity it matches:
- in `include/core/Config.hpp`: `mixing_height_m`, `concentration_scale_ug_per_m2`, `source.sigma`, `app.sensor_noise_std`
- in `src/science/Fields.cpp`: the wind preset magnitudes (at wind scale 1)

The "Faithful NY wildfire event export" example above uses the report's `faithful_source_decay_rate_per_s`/`faithful_source_lifespan_s` recommendations.

## CSV Export

The forward simulation as observed by the sensor network can be exported to a CSV file for offline analysis.

### Workflow

1. Place sensors on the right panel (left-click), or press `L`.
2. Optionally adjust the sensor averaging window via **Sensors → Sample Period (s)** (or press `N`). The default (5 s sim-time) averages five 1-second physical samples per report, mimicking a real instrument that samples frequently but reports a window average.
3. Open **File → Start Recording**. The `* REC` indicator appears in the top-right of the toolbar.
4. Click a source on the left panel and let the plume evolve.
5. Open **File → Stop Recording**. The status strip shows how many readings were captured.
6. Optionally pause and run `E`. The most recent estimates are written into the export header.
7. Open **File → Export CSV**. The file is written to the current working directory and the filename is shown in the status strip.

Recording is independent of the rolling sensor history used by the estimators. It is unbounded and accumulates for the full duration between start and stop. You can export the same recorded session multiple times (e.g. before and after running an estimate). Calling **Start Recording** again clears the previous buffer.

Sources already active when recording starts, and sources clicked during recording, are logged as ground-truth source events.

### Output format

The file is named to encode the key simulation parameters (spaces and path characters in model names become `_`):

```
atmospheric_{wind}_{diffusion}_{pde_mode}_dt{dt}_ts{time_scale}_sp{sample_period}s_{YYYYMMDD_HHMMSS}.csv
```

Example:
```
atmospheric_Jet_Shear_Constant_Scalar_Full_Tensor_Flux_dt0.100_ts10.0_sp5.0s_20260618_143022.csv
```

The file begins with `#` metadata comment lines, followed by a data header and one row per `(time, sensor)` pair, sorted time-first:

```
# Atmospheric Tool - Forward Simulation Export
# Wind model: Jet Shear
# Wind scale: 1.000000
# Diffusion model: Constant Scalar
# PDE diffusion mode: Full Tensor Flux
# dt (s): 0.100000
# Time scale: 10.000000
# Sensor averaging window (s): 5.000000
# Sensors recorded: 3
# Concentration: sensor window-averaged reading converted to ug/m^3
# Coordinates: lat_deg, lon_deg (WGS84, equirectangular back-projection from domain)
# Wind: u eastward, v northward (m/s); wind_direction is where the wind blows from, degrees clockwise from north
# Sources: 1
# Source 1: lat=41.018000 lon=-73.962000 born=12.340000s lifespan=100.000000s died=112.340000s
# Source term estimates (from the last 'E' run before export): 3
# Estimate 1 (Adjoint Backtracking): lat*=... lon*=... t*=...s
# Estimate 2 (Regularized Least Squares): lat*=... lon*=... t*=...s q0=... q0_std=... x_std_m=... y_std_m=... t_std_s=... weighted_rmse=... weakly_identified=false sensors_used=... observations_used=...
# Estimate 3 (Bayesian Grid): ...
Datetime_UTC,site_name,lat_deg,lon_deg,pm25_ugm-3,wind_speed,wind_direction,wind_u_component,wind_v_component,time_s
```

This is the georeferenced format, used whenever `wildfire_pm25_dataset.csv` was present at startup. Without it, positions stay in domain coordinates:
- the header reads `# Coordinates: x_m, y_m (...)`, and `# Wind:` describes domain axes
- source and estimate lines use `x=`/`y=` and `x*=`/`y*=`
- the columns are `x_m,y_m` in place of `lat_deg,lon_deg`

An estimate that failed is written as `no result - <reason>`.

The first nine columns match `wildfire_pm25_dataset.csv` in name, order, format and units, so scripts written for the real data can read an export directly. `time_s` is appended last:

- `Datetime_UTC`: synthetic timestamp, `time_s` offset from an anchor. With the geo projection active, the anchor is the real wildfire event start (2024-11-08 00:00 UTC); otherwise it is the wall-clock time recording started.
- `site_name`: sensor label; the real site name when loaded via `L`, else `Sensor_<index>`
- `lat_deg`, `lon_deg`: sensor coordinates when the geo projection is active; `x_m`, `y_m` (domain meters) otherwise
- `pm25_ugm-3`: noisy window-averaged sensor reading in µg/m³, using `conc_scale / mixing_height` conversion
- `wind_speed`: $\sqrt{u^2+v^2}$ (m/s)
- `wind_direction`: meteorological convention, as in the real datasets: the direction the wind blows *from*, in degrees clockwise from north, $\operatorname{atan2}(-u,-v)$
- `wind_u_component`, `wind_v_component`: wind at the sensor site at report time (m/s). With lat/lon output, u is eastward and v northward. The domain's +y axis points south (north is up on screen), so v is the negated domain y component.
- `time_s`: simulation time at the end of the averaging window

`csv_demo.py` shows how to read an export with pandas (`comment="#"`) and parse the `# Source` lines (lat/lon or x/y form) into a DataFrame.

## Project Structure

```text
include/
  adjoint/
    AdjointSolver.hpp               # Backward adjoint PDE solver
    SourceEstimator.hpp             # Three estimation methods + SourceEstimationConfig defaults
  app/
    Application.hpp                 # SFML app: window, event loop, HUD/menu/overlays
  core/
    Config.hpp                      # Global simulation/app parameters and defaults
    RuntimeSettings.hpp             # UI-editable runtime settings subset
    Validation.hpp                  # Centralized configuration validation
    Types.hpp                       # Vec2/Mat2 aliases (Eigen)
  export/
    DataRecorder.hpp                # CSV recording + export of sensor time series
  io/
    SiteLoader.hpp                  # NY site CSV parsing + equirectangular projection
  numerics/
    ParticleSystem.hpp              # SDE particle integrator + trails
    AdvectionDiffusionSolver.hpp    # PDE grid solver
  science/
    Fields.hpp                      # Wind and diffusivity presets
    SourceModel.hpp                 # Source lifecycle + source density
  sim/
    SensorManager.hpp               # Sensor placement, sampling, rolling history
    SimulationController.hpp        # Applies settings, recreates simulator when needed
    Simulator.hpp                   # Orchestration of SDE+PDE stepping
  ui/
    MenuModel.hpp                   # Toolbar menu state + pending edits
    TopToolbar.hpp                  # Toolbar rendering + click handling
    UiEcs.hpp                       # Legacy ECS quick-panel scaffolding (disabled by default)
    RetroTheme.hpp                  # Colors for the ECS quick panel

src/
  adjoint/AdjointSolver.cpp
  adjoint/SourceEstimator.cpp
  app/Application.cpp               # Construction, update loop, coordinate mapping
  app/ApplicationEvents.cpp         # Input handling, estimation, NY site loading
  app/ApplicationRender.cpp         # Panels, HUD cards, overlays
  core/RuntimeSettings.cpp
  core/Validation.cpp
  export/DataRecorder.cpp
  io/SiteLoader.cpp
  numerics/ParticleSystem.cpp
  numerics/AdvectionDiffusionSolver.cpp
  science/Fields.cpp
  science/SourceModel.cpp
  sim/SensorManager.cpp
  sim/SimulationController.cpp
  sim/Simulator.cpp
  ui/MenuModel.cpp
  ui/RetroTheme.cpp
  ui/TopToolbar.cpp
  ui/UiEcs.cpp
  main.cpp                          # CLI parsing + app bootstrap

tests/
  self_check.cpp                    # CTest regression checks (atmospheric_self_check)

analysis/
  __init__.py
  io.py                             # Real-dataset loaders
  scales.py                         # Spatial/temporal/concentration/wind scale inference
  forward_model.py                  # numpy PDE replica for conc-scale calibration
  report.py                         # Orchestrates the above into calibration_report.md
  calibration_report.md             # Generated calibration output

csv_demo.py                         # Example: reading an exported CSV + its source metadata

fonts/
  arial.ttf
```

## Core Classes

- `atm::Application`
  - Owns the SFML window/event loop; renders both panels, HUD, menus, and overlays
- `atm::SimulationController`
  - Holds launch and current config; applies queued settings and recreates the `Simulator` when needed
- `atm::Simulator`
  - Owns shared models and advances them in fixed steps
- `atm::Fields`
  - Wind presets $w(t,x)$ and scalar/tensor diffusivity presets $D(t,x)$, plus $\nabla\cdot D$
- `atm::SourceModel`
  - Multi-source lifecycle manager (add/expire), decaying emissions, summed Gaussian source density
- `atm::ParticleSystem`
  - Euler-Maruyama transport + deposition/killing + boundary handling + trail history
- `atm::AdvectionDiffusionSolver`
  - Explicit PDE update on a uniform grid
- `atm::SensorManager`
  - Sensor placement, noisy physical sampling, window-averaged reports, rolling observation history
- `atm::AdjointSolver`
  - Backward adjoint transport solver (scalarized/full-tensor diffusion modes)
- `atm::SourceEstimator`
  - Runs the three `SourceEstimationMethod`s over sensor history and returns $(x^*, t^*)$, amplitude, uncertainty, and posterior fields
- `atm::DataRecorder`
  - Captures sensor reports and source events during a recording session and exports them as a CSV with simulation metadata and estimates
- `load_unique_sites` / `project_sites_to_domain` (`io/SiteLoader`)
  - Parse the NY site list and map it into the simulation domain
