#include "core/RuntimeSettings.hpp"

namespace atm {

RuntimeSettings make_runtime_settings(const Config& config, AdvectionDiffusionSolver::DiffusionMode mode) {
    RuntimeSettings out;
    out.time_scale = config.numerics.time_scale;
    out.max_particles = static_cast<int>(config.numerics.max_particles);
    out.deposition_rate = config.physics.deposition_rate;
    out.constant_scalar_diffusivity = config.physics.constant_scalar_diffusivity;
    out.wind_scale = config.physics.wind_scale;
    out.source_base_emission = config.source.base_emission;
    out.source_decay_rate = config.source.decay_rate;
    out.source_lifespan = config.source.lifespan;
    out.source_sigma = config.source.sigma;
    out.source_max_sources = config.source.max_sources;
    out.pde_fixed_color_scale = config.app.pde_fixed_color_scale;
    out.sensor_sample_period_s = config.app.sensor_sample_period_s;
    out.sensor_noise_std = config.app.sensor_noise_std;
    out.sensor_history_capacity = static_cast<int>(config.app.sensor_history_capacity);
    out.pde_grid_nx = config.domain.nx;
    out.pde_grid_ny = config.domain.ny;
    out.dt = config.numerics.dt;
    out.pde_diffusion_mode = mode;
    return out;
}

bool same_runtime_settings(const RuntimeSettings& a, const RuntimeSettings& b) {
    return a.time_scale == b.time_scale && a.max_particles == b.max_particles
        && a.deposition_rate == b.deposition_rate && a.constant_scalar_diffusivity == b.constant_scalar_diffusivity
        && a.wind_scale == b.wind_scale
        && a.source_base_emission == b.source_base_emission
        && a.source_decay_rate == b.source_decay_rate && a.source_lifespan == b.source_lifespan
        && a.source_sigma == b.source_sigma && a.source_max_sources == b.source_max_sources
        && a.pde_fixed_color_scale == b.pde_fixed_color_scale
        && a.sensor_sample_period_s == b.sensor_sample_period_s && a.sensor_noise_std == b.sensor_noise_std
        && a.sensor_history_capacity == b.sensor_history_capacity
        && a.pde_grid_nx == b.pde_grid_nx && a.pde_grid_ny == b.pde_grid_ny && a.dt == b.dt
        && a.pde_diffusion_mode == b.pde_diffusion_mode;
}

} // namespace atm
