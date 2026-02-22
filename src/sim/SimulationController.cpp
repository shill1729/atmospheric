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
        || next.dt != current.dt;

    if (requires_recreate) {
        Config next_config = config_;
        next_config.domain.nx = next.pde_grid_nx;
        next_config.domain.ny = next.pde_grid_ny;
        next_config.numerics.dt = next.dt;

        const auto errors = validate_config(next_config);
        if (!errors.empty()) {
            return report;
        }

        config_ = next_config;
        simulator_ = std::make_unique<Simulator>(config_);
        report.recreated_simulator = true;
        report.reset_state = true;
    }

    simulator().set_pde_diffusion_mode(next.pde_diffusion_mode);
    report.changed = true;
    return report;
}

} // namespace atm
