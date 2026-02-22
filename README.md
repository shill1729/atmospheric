# Atmospheric (SDE + PDE Prototype)

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

## Mathematical Model

### Domain, state, and source

Let the domain be

$$
\Omega = [x_{\min}, x_{\max}] \times [y_{\min}, y_{\max}] \subset \mathbb{R}^2.
$$

A synthetic source $k$ is centered at $x_k(t)$ and emits at rate $q_k(t)$, with

$$
q_k(t) = q_0 e^{-\gamma (t-t_k)}, \quad t_k \le t \le t_k + T_s,
$$

and $q_k(t)=0$ after lifespan $T_s$.

The total spatial source density is a sum of Gaussian emitters:

$$
s(t,x) = \sum_{k=1}^{K(t)} q_k(t)\,\frac{1}{2\pi\sigma^2}\exp\left(-\frac{\|x-x_k(t)\|^2}{2\sigma^2}\right),
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
  --source-emission 1.2
```

## Runtime Controls

- `Left click` (left panel): add source (up to `source-max` active sources)
- `W`: cycle wind model
- `K`: cycle diffusivity model
- `P`: cycle PDE diffusion mode (scalarized/full tensor flux)
- `C`: toggle PDE color scaling mode (auto/fixed)
- Top toolbar `PDE`: editable pending settings for `grid nx/ny`, `dt`, and PDE diffusion mode, with explicit `Apply/Revert`
- `H`: toggle Brownian/Heat special case
- `U`: toggle HUD mass units (`g/m^2` vs `ug/m^2`)
- `Space`: pause/resume
- `R`: reset simulation
- `B`: toggle shared SDE/PDE boundary mode (periodic/reflecting/absorbing)
- `[` / `]`: slower/faster simulation speed
- `\`: reset speed to configured base
- `Esc`: open/close preferences menu

Menu controls:
- `Up/Down`: select option
- `Left/Right/Enter`: adjust/apply

## Project Structure

```text
include/
  app/
    Application.hpp                  # SFML app, rendering, HUD/menu/input
  core/
    Config.hpp                       # Global simulation/app parameters
    RuntimeSettings.hpp              # UI-editable runtime settings subset
    Validation.hpp                   # Centralized configuration validation
    Types.hpp                        # Vec2/Mat2 aliases (Eigen)
  numerics/
    ParticleSystem.hpp               # SDE particle integrator + trails
    AdvectionDiffusionSolver.hpp     # PDE grid solver
  science/
    Fields.hpp                       # Wind and diffusivity models
    SourceModel.hpp                  # Source lifecycle + source density
  sim/
    SimulationController.hpp         # Applies settings, recreates simulator when needed
    Simulator.hpp                    # Orchestration of SDE+PDE stepping
  ui/
    MenuModel.hpp                    # Toolbar menu state + pending edits
    TopToolbar.hpp                   # Toolbar rendering + click handling

src/
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

## Current Scope and Limitations

- SDE supports scalar and SPD tensor diffusivity presets (including Brownian/Heat case with $\kappa=\tfrac{1}{2}$ and zero wind).
- PDE supports both scalarized tensor approximation and full tensor flux diffusion mode.
- No data assimilation yet (Kalman/filtering/adjoint not yet implemented).
- No persistent preferences/config save file yet.

## Features TBA

1. Config file IO for reproducible runs.
2. Optional measurement stations and inverse-source workflow.
3. Expanded automated numerical regression tests.
