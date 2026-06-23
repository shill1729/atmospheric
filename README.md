# Atmospheric (SDE + PDE + Adjoint Prototype)

Interactive C++ atmospheric transport prototype with a split view:
- **Left panel:** tagged-particle simulation via SDE (Euler-Maruyama)
- **Right panel:** concentration evolution via advection-diffusion(-reaction) PDE

The two models share the same wind field, diffusivity model, source process, and deposition rate.

## Features

- Click-to-add continuous sources with finite lifetime and exponential decay (multi-source, capped)
- SDE particle transport with diffusion and deposition/killing
- PDE concentration transport with source and deposition
- Diffusivity presets: scalar and matrix/tensor examples (constant, diagonal-varying, full anisotropic SPD)
- Wind vector overlay (toggle)
- Runtime controls for speed, boundary mode, trails, and HUD/menu preferences
- PDE-panel sensors with periodic concentration sampling and on-plot readouts
- Pause-time adjoint source estimation from sensor history (posterior heatmap + \((x^*, t^*)\))

## Mathematical Model

### Domain, state, and source

Let the domain be

$$
\Omega = [x_{\min}, x_{\max}] \times [y_{\min}, y_{\max}] \subset \mathbb{R}^2.
$$

A synthetic source $k$ is centered at fixed location $x_k$ and emits at rate $q_k(t)$, with

$$
q_k(t) = q_0 e^{-\gamma (t-t_k)}, \quad t_k \le t \le t_k + T_s,
$$

and $q_k(t)=0$ after lifespan $T_s$.

The total spatial source density is a sum of Gaussian emitters:

$$
s(t,x) = \sum_{k=1}^{K(t)} q_k(t)\,\frac{1}{2\pi\sigma^2}\exp\left(-\frac{\|x-x_k\|^2}{2\sigma^2}\right),
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

where $\mu = w+\nabla\cdot D$, $LL^\top = D$, and $\xi_n\sim\mathcal{N}(0,I)$ i.i.d. per particle.  
Particle births are generated independently from each active source.

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
- upwind advection term for $-w\cdot\nabla c$
- selectable diffusion operator:
  - scalarized approximation using $\kappa=\tfrac12\mathrm{tr}(D)$
  - full tensor flux form $\nabla\cdot(D\nabla c)$ on a centered finite-difference stencil
- source addition $s(t,x)$
- deposition sink $-\lambda c$

Negative values are clamped to zero.

### Adjoint viewpoint (inverse source estimation)

To infer likely release location/time from sensor detections, the app solves the backward adjoint transport PDE:

$$
\partial_t \phi + w\cdot \nabla \phi + \nabla\cdot(D\nabla \phi) - \lambda \phi + f = 0,
\quad \phi(T,x)=0.
$$

The forcing \(f(t,x)\) is built from sensor sites \(\{x_i\}\) using threshold-gated Gaussian bumps:

$$
f(t,x)=\sum_i \alpha_i\,\mathbf{1}_{\tilde c(t,x_i)>c_T}\,\beta_i
\exp\!\left(-\frac{\|x-x_i\|^2}{2\sigma^2}\right).
$$

Current defaults in code:
- equal sensor weights ($\alpha_i=1$)
- fixed detection threshold $c_T$
- $\beta_i$ normalized per bump over the computational domain
- forcing is piecewise-constant in time from recorded sensor samples

The app computes
$$
Z(t)=\int_\Omega \phi(t,x)\,dx,\quad
t^*=\arg\max_t Z(t),\quad
x^*=\arg\max_x \int_0^T p(t,x)dt,\quad
p=\phi/Z.
$$


## Initial and Boundary Conditions

### Initial conditions (IC)

- SDE: no particles until source activation (or reset)
- PDE: $c(0,x)=0$

### Boundary conditions (BC)

- **SDE:** selectable runtime mode
  - periodic wrapping
  - reflecting bounce
  - absorbing/outflow
- **PDE:** now synced to the same boundary mode toggle as SDE (`B` key)
  - periodic index wrapping
  - reflecting index mirroring
  - absorbing boundary with zero exterior concentration (Dirichlet-like)
- **Adjoint (source estimator):** 2D lateral inflow/outflow style BCs
  - inflow ($w\cdot n<0$): $\phi=0$
  - outflow ($w\cdot n\ge 0$): Robin-type $D\partial_n\phi + (w\cdot n)\phi=0$ (discretized)

## Numerical Methods Summary

### SDE solver

- Scheme: Euler-Maruyama
- Drift: $w+\nabla\cdot D$ (row-wise divergence of the diffusivity tensor)
- Noise: matrix diffusion via $\sqrt{2\Delta t}\,D^{1/2}\xi$ (SPD factorization)
- Time stepping: fixed $\Delta t$ internal step, with wall-clock scaling and per-frame substep cap

### PDE solver

- Grid: uniform Cartesian $N_x\times N_y$
- Time stepping: explicit forward Euler in time
- Spatial discretization:
  - advection: first-order upwind
  - diffusion: either scalarized $\kappa=\tfrac12 \mathrm{tr}(D)$ or full tensor flux $\nabla \cdot (D\nabla c)$

### Adjoint solver

- Solve direction: backward in physical time ($t\in[t_0,T]$) with terminal condition $\phi(T,x)=0$
- Grid: separate uniform Cartesian adjoint grid (default $96\times96$)
- Time stepping: explicit update on adjoint grid
- Advection: first-order upwind for $w\cdot\nabla\phi$
- Diffusion mode: matched to current PDE mode
  - scalarized trace mode
  - full tensor flux mode
- Forcing: built from recorded sensor history (threshold-gated Gaussian bumps)
- Output: posterior-like field $p(t^*,x)$, estimated $(x^*,t^*)$, overlay heatmap

## Dependencies

- C++20 compiler
- CMake >= 3.20
- SFML 3 (`Graphics`, `Window`, `System`)
- Eigen3

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

## Run

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

## Runtime Controls

- `Left click` (left panel): add source (up to `source-max` active sources)
- `Left click` (right panel): add concentration sensor
- `W`: cycle wind model
- `K`: cycle diffusivity model
- `P`: cycle PDE diffusion mode (scalarized/full tensor flux)
- `C`: toggle PDE color scaling mode (auto/fixed)
- `E`: run source estimation (requires paused state)
- `J`: toggle adjoint overlay on the right panel
- Top toolbar menus: `File | Source | Numerics | Sensors | Display | PDE`
- Top toolbar `File` actions:
  - `Estimate Source (Paused)`
  - `Apply Queued Changes`
  - `Revert Queued Changes`
  - `Restore Launch Defaults`
  - `Start Recording` / `Stop Recording` — toggle CSV data capture (label reflects state; `* REC` indicator appears in toolbar while active)
  - `Export CSV` — write captured sensor time series to a CSV file in the current working directory
- Parameter edits in `Source/Numerics/Sensors/Display/PDE` are queued; apply from `File`.
- Numeric edit UX in top menus:
  - `-` / `+` buttons for stepped adjustments
  - click value field to type
  - `Enter` commit, `Backspace` delete, `Esc` cancel
- `H`: toggle Brownian/Heat special case
- `U`: toggle HUD mass units (`g/m^2` vs `ug/m^2`)
- `Space`: pause/resume
- `R`: reset simulation
- `B`: toggle shared SDE/PDE boundary mode (periodic/reflecting/absorbing)
- `[` / `]`: slower/faster simulation speed
- `\`: reset speed to configured base
- `Esc`: open/close preferences menu

Concentration display calibration:
- Displayed PDE and sensor concentrations are reported as `ug/m^3` using:
  - `conc_display = model_concentration * conc_scale / mixing_height`
- `--conc-scale` sets `conc_scale` (ug/m^2 per model unit).
- `--mixing-height` is the assumed vertical mixing depth (meters).
- `--sensor-history-capacity` sets max observations retained per sensor history.
- Sensors now sample physically at `--sensor-physical-period` (default `1s`) and report period-averaged observations
  every `sensor sample period` (default `5s`).
- Sensor on-plot labels show each sensor's latest report value (`ug/m^3`) or `N/A` before first report.
- Reported sensor value is the window average of noisy physical samples in that report interval.
- `--sensor-spatial-radius` sets disk-like local spatial averaging radius around each sensor (meters).

## CSV Export

The forward simulation as observed by the sensor network can be exported to a CSV file for offline analysis.

### Workflow

1. Place sensors on the right panel (left-click).
2. Optionally adjust the sensor averaging window via **Sensors → Sample Period (s)**. The default (5 s sim-time) averages five 1-second physical samples per report, mimicking a real instrument that samples frequently but reports a window average. For longer synthetic runs where you want to mimic hourly averages of 5-minute readings, increase the sample period accordingly.
3. Open **File → Start Recording**. The `* REC` indicator appears in the top-right of the toolbar.
4. Click a source on the left panel and let the plume evolve.
5. Open **File → Stop Recording**. The status strip shows how many readings were captured.
6. Open **File → Export CSV**. The file is written to the current working directory and the filename is shown in the status strip.

Recording is independent of the rolling sensor history used by the adjoint estimator — it is unbounded and accumulates for the full duration between start and stop. You can export the same recorded session multiple times (e.g. before and after changing the wind model to compare). Calling **Start Recording** again clears the previous buffer.

### Output format

The file is named to encode the key simulation parameters:

```
atmospheric_{wind}_{diffusion}_{pde_mode}_dt{dt}_ts{time_scale}_sp{sample_period}s_{YYYYMMDD_HHMMSS}.csv
```

Example:
```
atmospheric_JetShear_ConstantScalar_FullTensor_dt0.010_ts10.0_sp5.0s_20260618_143022.csv
```

The file begins with `#` metadata comment lines followed by a data header and one row per `(time, sensor)` pair, sorted time-first:

```
# Atmospheric Tool - Forward Simulation Export
# Wind model: JetShear
# Diffusion model: ConstantScalar
# PDE diffusion mode: Full Tensor Flux
# dt (s): 0.010
# Time scale: 10.0
# Sensor averaging window (s): 5.0
# Sensors recorded: 3
# Concentration: sensor window-averaged reading converted to ug/m^3
time_s,x_m,y_m,concentration_ug_m3,wind_u_m_s,wind_v_m_s
```

- `time_s` — simulation time at end of the averaging window
- `x_m`, `y_m` — sensor domain coordinates (meters)
- `concentration_ug_m3` — noisy window-averaged sensor reading in µg/m³, using `conc_scale / mixing_height` conversion
- `wind_u_m_s`, `wind_v_m_s` — wind vector at the sensor site sampled at report time

Legacy menu controls (modal preferences overlay):
- `Up/Down`: select option
- `Left/Right/Enter`: adjust/apply

## Project Structure

```text
include/
  adjoint/
    AdjointSolver.hpp               # Backward adjoint PDE solver
    SourceEstimator.hpp             # Sensor-forcing builder + (x*, t*) estimator
  app/
    Application.hpp                  # SFML app, rendering, HUD/menu/input
  core/
    Config.hpp                       # Global simulation/app parameters
    RuntimeSettings.hpp              # UI-editable runtime settings subset
    Validation.hpp                   # Centralized configuration validation
    Types.hpp                        # Vec2/Mat2 aliases (Eigen)
  export/
    DataRecorder.hpp                 # CSV recording + export of sensor time series
  numerics/
    ParticleSystem.hpp               # SDE particle integrator + trails
    AdvectionDiffusionSolver.hpp     # PDE grid solver
  science/
    Fields.hpp                       # Wind and diffusivity models
    SourceModel.hpp                  # Source lifecycle + source density
  sim/
    SensorManager.hpp                # Sensor placement + periodic sampled history
    SimulationController.hpp         # Applies settings, recreates simulator when needed
    Simulator.hpp                    # Orchestration of SDE+PDE stepping
  ui/
    MenuModel.hpp                    # Toolbar menu state + pending edits
    TopToolbar.hpp                   # Toolbar rendering + click handling
    UiEcs.hpp                        # Legacy ECS quick-panel scaffolding (currently disabled by default)

src/
  adjoint/AdjointSolver.cpp
  adjoint/SourceEstimator.cpp
  app/Application.cpp
  core/RuntimeSettings.cpp
  core/Validation.cpp
  numerics/ParticleSystem.cpp
  numerics/AdvectionDiffusionSolver.cpp
  science/Fields.cpp
  science/SourceModel.cpp
  sim/SimulationController.cpp
  sim/Simulator.cpp
  ui/MenuModel.cpp
  ui/TopToolbar.cpp
  ui/UiEcs.cpp
  main.cpp                           # CLI parsing + app bootstrap

tests/
  self_check.cpp                     # Lightweight ctest regression checks

fonts/
  arial.ttf
```

## Core Classes

- `atm::Application`
  - Owns SFML window/event loop, renders both panels, HUD, menu
- `atm::Simulator`
  - Owns shared models and advances them in fixed steps
- `atm::Fields`
  - Defines synthetic $w(t,x)$ and scalar diffusivity model $\kappa(t,x)$ (with tensor roadmap)
- `atm::SourceModel`
  - Multi-source lifecycle manager (add/expire), decaying emissions, summed Gaussian source density
- `atm::ParticleSystem`
  - Euler-Maruyama transport + deposition/killing + trail history
- `atm::AdvectionDiffusionSolver`
  - Explicit PDE update on uniform grid
- `atm::SensorManager`
  - Sensor placement, periodic sampling, rolling observation history
- `atm::AdjointSolver`
  - Backward adjoint transport solver (scalarized/full-tensor diffusion modes)
- `atm::SourceEstimator`
  - Builds adjoint forcing from sensors and extracts \((x^*, t^*)\)
- `atm::DataRecorder`
  - Captures sensor window reports during a recording session and exports them as a CSV with simulation metadata

