# Atmospheric (SDE + PDE Prototype)

Interactive C++ atmospheric transport prototype with a split view:
- **Left panel:** tagged-particle simulation via SDE (Euler-Maruyama)
- **Right panel:** concentration evolution via advection-diffusion(-reaction) PDE

The two models share the same wind field, diffusivity model, source process, and deposition rate.

## Features

- Click-to-place continuous point source with finite lifetime and exponential decay
- SDE particle transport with diffusion and deposition/killing
- PDE concentration transport with source and deposition
- Wind vector overlay (toggle)
- Runtime controls for speed, boundary mode, trails, and HUD/menu preferences

## Mathematical Model

### Domain, state, and source

Let the domain be

$$
\Omega = [x_{\min}, x_{\max}] \times [y_{\min}, y_{\max}] \subset \mathbb{R}^2.
$$

A synthetic source is centered at $x_s(t)$ and emits at rate $q(t)$, with

$$
q(t) = q_0 e^{-\gamma t}, \quad 0 \le t \le T_s,
$$

and $q(t)=0$ after lifespan $T_s$.

The spatial source density is Gaussian:

$$
s(t,x) = q(t)\,\frac{1}{2\pi\sigma^2}\exp\left(-\frac{\|x-x_s(t)\|^2}{2\sigma^2}\right).
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
- central finite differences for diffusion via $\kappa\Delta c$ with $\kappa = \tfrac{1}{2}\operatorname{tr}(D)$
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
- **PDE:** edge sampling currently uses clamped neighbor values (zero-normal-gradient style behavior for finite differences)

## Numerical Methods Summary

### SDE solver

- Scheme: Euler-Maruyama
- Noise: full matrix factorization via Cholesky $D=LL^\top$
- Time stepping: fixed $\Delta t$ internal step, with wall-clock scaling and per-frame substep cap

### PDE solver

- Grid: uniform Cartesian $N_x\times N_y$
- Time stepping: explicit forward Euler in time
- Spatial discretization:
  - advection: first-order upwind
  - diffusion: second-order central Laplacian (effective isotropic coefficient from $D$ trace)

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

- `Left click` (left panel): place/activate source
- `Space`: pause/resume
- `R`: reset simulation
- `B`: toggle particle BC mode (periodic/reflecting)
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
    Types.hpp                        # Vec2/Mat2 aliases (Eigen)
  numerics/
    ParticleSystem.hpp               # SDE particle integrator + trails
    AdvectionDiffusionSolver.hpp     # PDE grid solver
  science/
    Fields.hpp                       # Wind and diffusivity models
    SourceModel.hpp                  # Source lifecycle + source density
  sim/
    Simulator.hpp                    # Orchestration of SDE+PDE stepping

src/
  app/Application.cpp
  numerics/ParticleSystem.cpp
  numerics/AdvectionDiffusionSolver.cpp
  science/Fields.cpp
  science/SourceModel.cpp
  sim/Simulator.cpp
  main.cpp                           # CLI parsing + app bootstrap

fonts/
  arial.ttf
```

## Core Classes

- `atm::Application`
  - Owns SFML window/event loop, renders both panels, HUD, menu
- `atm::Simulator`
  - Owns shared models and advances them in fixed steps
- `atm::Fields`
  - Defines synthetic $w(t,x)$ and $D(t,x)$, with numerical $\nabla\cdot D$
- `atm::SourceModel`
  - Source activation, age/lifespan, decaying emission, Gaussian source density
- `atm::ParticleSystem`
  - Euler-Maruyama transport + deposition/killing + trail history
- `atm::AdvectionDiffusionSolver`
  - Explicit PDE update on uniform grid

## Current Scope and Limitations

- PDE discretization uses an effective scalar diffusion from $\operatorname{tr}(D)$ for robustness/simplicity.
- No data assimilation yet (Kalman/filtering/adjoint not yet implemented).
- No persistent preferences/config save file yet.

## Next Steps (recommended)

1. Full tensor-diffusion PDE discretization in flux form $\nabla\cdot(D\nabla c)$.
2. Config file IO for reproducible runs.
3. Colormap and dynamic range controls for PDE panel.
4. Optional measurement stations and inverse-source workflow.
