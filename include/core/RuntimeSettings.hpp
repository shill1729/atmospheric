#pragma once

#include "core/Config.hpp"
#include "numerics/AdvectionDiffusionSolver.hpp"

namespace atm {

struct RuntimeSettings {
    float time_scale = 0.0f;
    int max_particles = 0;
    float deposition_rate = 0.0f;
    float constant_scalar_diffusivity = 0.0f;
    float source_base_emission = 0.0f;
    float source_decay_rate = 0.0f;
    float source_lifespan = 0.0f;
    float source_sigma = 0.0f;
    int source_max_sources = 0;
    float pde_fixed_color_scale = 0.0f;
    float sensor_sample_period_s = 0.0f;
    float sensor_noise_std = 0.0f;
    int sensor_history_capacity = 0;
    int pde_grid_nx = 0;
    int pde_grid_ny = 0;
    float dt = 0.0f;
    AdvectionDiffusionSolver::DiffusionMode pde_diffusion_mode = AdvectionDiffusionSolver::DiffusionMode::FullTensorFlux;
};

RuntimeSettings make_runtime_settings(const Config& config, AdvectionDiffusionSolver::DiffusionMode mode);
bool same_runtime_settings(const RuntimeSettings& a, const RuntimeSettings& b);

} // namespace atm
