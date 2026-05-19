#pragma once

#include "adjoint/AdjointSolver.hpp"
#include "core/Types.hpp"
#include "sim/SensorManager.hpp"
#include "sim/Simulator.hpp"

#include <string>
#include <vector>

namespace atm {

struct SourceEstimationConfig {
    float detection_threshold  = 0.000001f;
    float gaussian_sigma = 120.0f;
    int adjoint_grid_nx = 96;
    int adjoint_grid_ny = 96;
    float max_lookback_s = 150.0f;
    int max_samples_per_sensor = 30;
    float adjoint_dt_s = 0.5f;
};

struct SourceEstimateResult {
    bool success = false;
    bool insufficient_signal = false;
    std::string message;
    Vec2 x_star = Vec2::Zero();
    float t_star_s = 0.0f;

    int nx = 0;
    int ny = 0;
    std::vector<float> p_star;
};

class SourceEstimator {
public:
    explicit SourceEstimator(SourceEstimationConfig cfg = {});

    const SourceEstimationConfig& config() const;
    SourceEstimateResult estimate(const std::vector<SensorManager::Sensor>& sensors, const Simulator& sim) const;

private:
    SourceEstimationConfig cfg_;
    AdjointSolver solver_;
};

} // namespace atm
