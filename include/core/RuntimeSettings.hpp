#pragma once

#include "core/Config.hpp"
#include "numerics/AdvectionDiffusionSolver.hpp"

namespace atm {

struct RuntimeSettings {
    int pde_grid_nx = 0;
    int pde_grid_ny = 0;
    float dt = 0.0f;
    AdvectionDiffusionSolver::DiffusionMode pde_diffusion_mode = AdvectionDiffusionSolver::DiffusionMode::FullTensorFlux;
};

RuntimeSettings make_runtime_settings(const Config& config, AdvectionDiffusionSolver::DiffusionMode mode);
bool same_runtime_settings(const RuntimeSettings& a, const RuntimeSettings& b);

} // namespace atm
