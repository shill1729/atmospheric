#include "sim/SimulationController.hpp"

#include "core/Validation.hpp"

#include <utility>

namespace atm {

SimulationController::SimulationController(const Config& config)
    : config_(config)
    , simulator_(std::make_unique<Simulator>(config_)) {
}

Simulator& SimulationController::simulator() {
    return *simulator_;
}

const Simulator& SimulationController::simulator() const {
    return *simulator_;
}

RuntimeSettings SimulationController::current_settings() const {
    return make_runtime_settings(config_, simulator().pde_diffusion_mode());
}

ApplySettingsReport SimulationController::apply_settings(const RuntimeSettings& next) {
    ApplySettingsReport report;
    const RuntimeSettings current = current_settings();
    if (same_runtime_settings(current, next)) {
        return report;
    }

    const bool requires_recreate = next.pde_grid_nx != current.pde_grid_nx || next.pde_grid_ny != current.pde_grid_ny
        || next.dt != current.dt || next.max_particles != current.max_particles
        || next.deposition_rate != current.deposition_rate
        || next.constant_scalar_diffusivity != current.constant_scalar_diffusivity
        || next.source_base_emission != current.source_base_emission
        || next.source_decay_rate != current.source_decay_rate || next.source_lifespan != current.source_lifespan
        || next.source_sigma != current.source_sigma || next.source_max_sources != current.source_max_sources;

    if (requires_recreate) {
        Config next_config = config_;
        next_config.numerics.max_particles = static_cast<std::size_t>(next.max_particles);
        next_config.physics.deposition_rate = next.deposition_rate;
        next_config.physics.constant_scalar_diffusivity = next.constant_scalar_diffusivity;
        next_config.source.base_emission = next.source_base_emission;
        next_config.source.decay_rate = next.source_decay_rate;
        next_config.source.lifespan = next.source_lifespan;
        next_config.source.sigma = next.source_sigma;
        next_config.source.max_sources = next.source_max_sources;
        next_config.domain.nx = next.pde_grid_nx;
        next_config.domain.ny = next.pde_grid_ny;
        next_config.numerics.dt = next.dt;
        next_config.numerics.time_scale = next.time_scale;

        const auto errors = validate_config(next_config);
        if (!errors.empty()) {
            return report;
        }

        config_ = next_config;
        simulator_ = std::make_unique<Simulator>(config_);
        report.recreated_simulator = true;
        report.reset_state = true;
    }

    if (!report.recreated_simulator && next.time_scale != current.time_scale) {
        config_.numerics.time_scale = next.time_scale;
        simulator().set_time_scale(next.time_scale);
    }
    if (next.pde_fixed_color_scale != current.pde_fixed_color_scale) {
        config_.app.pde_fixed_color_scale = next.pde_fixed_color_scale;
    }
    if (next.sensor_sample_period_s != current.sensor_sample_period_s) {
        config_.app.sensor_sample_period_s = next.sensor_sample_period_s;
    }
    if (next.sensor_noise_std != current.sensor_noise_std) {
        config_.app.sensor_noise_std = next.sensor_noise_std;
    }
    if (next.sensor_history_capacity != current.sensor_history_capacity) {
        config_.app.sensor_history_capacity = static_cast<std::size_t>(next.sensor_history_capacity);
    }

    simulator().set_pde_diffusion_mode(next.pde_diffusion_mode);
    report.changed = true;
    return report;
}

} // namespace atm
