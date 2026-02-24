#pragma once

#include "core/Config.hpp"
#include "core/RuntimeSettings.hpp"
#include "sim/Simulator.hpp"

#include <memory>

namespace atm {

struct ApplySettingsReport {
    bool changed = false;
    bool recreated_simulator = false;
    bool reset_state = false;
};

class SimulationController {
public:
    explicit SimulationController(const Config& config);

    Simulator& simulator();
    const Simulator& simulator() const;

    RuntimeSettings current_settings() const;
    ApplySettingsReport apply_settings(const RuntimeSettings& next);
    ApplySettingsReport restore_launch_defaults();

private:
    const Config launch_config_;
    Config config_;
    std::unique_ptr<Simulator> simulator_;
};

} // namespace atm
