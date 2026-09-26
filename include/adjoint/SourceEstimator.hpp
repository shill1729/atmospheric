#pragma once

#include "adjoint/AdjointSolver.hpp"
#include "core/Types.hpp"
#include "sim/SensorManager.hpp"
#include "sim/Simulator.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace atm {

enum class SourceEstimationMethod {
    AdjointBacktracking = 0,
    RegularizedLeastSquares = 1,
    BayesianGrid = 2
};

std::string_view source_estimation_method_name(SourceEstimationMethod method);

struct SourceEstimationConfig {
    // Minimum report value counted as a detection (model units). Each run
    // raises it to the sensors' noise floor (see SourceEstimator::set_noise_floor).
    float detection_threshold = 0.000001f;
    float gaussian_sigma = 120.0f;
    int adjoint_grid_nx = 96;
    int adjoint_grid_ny = 96;
    // Observation window: reports older than this (or already dropped from
    // a sensor's rolling history) are ignored. If no report inside it
    // exceeds detection_threshold, estimation reports insufficient signal.
    float max_lookback_s = 1800.0f;
    // Release-time search horizon extends this far before the earliest
    // observation in the window, since a release precedes its first
    // detection by the source-to-sensor travel time (~3.5 km at ~2 m/s).
    float release_search_margin_s = 1800.0f;
    int max_samples_per_sensor = 500;
    // Maximum adjoint step; AdjointSolver shrinks it when stability requires.
    float adjoint_dt_s = 2.0f;
    int candidate_count = 5;
    int refinement_levels = 3;
    int search_grid_nx = 14;
    int search_grid_ny = 14;
    int search_time_count = 24;
    int response_quadrature_points = 6;
    int max_total_observations = 2500;
    float relative_model_error = 0.05f;
    float amplitude_ridge = 1.0e-8f;
};

struct SourceEstimationObservation {
    int sensor_index = -1;
    Vec2 sensor_position = Vec2::Zero();
    float time_s = 0.0f;
    float measured_concentration = 0.0f;
    float predicted_concentration = 0.0f;
    float residual = 0.0f;
    float assumed_std = 0.0f;
};

struct SourceEstimateResult {
    SourceEstimationMethod method = SourceEstimationMethod::AdjointBacktracking;
    bool success = false;
    bool insufficient_signal = false;
    std::string message;
    Vec2 x_star = Vec2::Zero();
    float t_star_s = 0.0f;
    float q0 = 0.0f;
    float q0_std = 0.0f;
    float total_released_mass = 0.0f;
    float weighted_rmse = 0.0f;
    float x_std_m = 0.0f;
    float y_std_m = 0.0f;
    float t_std_s = 0.0f;
    bool weakly_identified = false;
    int sensors_used = 0;
    int observations_used = 0;

    int nx = 0;
    int ny = 0;
    std::vector<float> p_star;
    std::vector<float> predicted_observations;
    std::vector<float> residuals;
    std::vector<SourceEstimationObservation> observation_fit;
};

class SourceEstimator {
public:
    explicit SourceEstimator(SourceEstimationConfig cfg = {});

    const SourceEstimationConfig& config() const;
    // Raises the effective detection threshold to at least `floor` (e.g.
    // SensorManager::noise_detection_floor()), so sensor noise alone is not
    // treated as a detection; the configured threshold remains the minimum.
    void set_noise_floor(float floor);
    SourceEstimateResult estimate(const std::vector<SensorManager::Sensor>& sensors, const Simulator& sim) const;
    SourceEstimateResult estimate(
        const std::vector<SensorManager::Sensor>& sensors, const Simulator& sim, SourceEstimationMethod method) const;
    std::vector<SourceEstimateResult> estimate_all(
        const std::vector<SensorManager::Sensor>& sensors, const Simulator& sim) const;

    // Observation window [t_now - max_lookback_s, t_now], as used by every method.
    float observation_window_start(const Simulator& sim) const;

private:
    SourceEstimateResult estimate_adjoint_backtracking(
        const std::vector<SensorManager::Sensor>& sensors, const Simulator& sim) const;
    SourceEstimateResult estimate_regularized_least_squares(
        const std::vector<SensorManager::Sensor>& sensors, const Simulator& sim) const;
    SourceEstimateResult estimate_bayesian_grid(
        const std::vector<SensorManager::Sensor>& sensors, const Simulator& sim) const;

    SourceEstimationConfig cfg_;
    float base_detection_threshold_ = 0.0f;
    AdjointSolver solver_;
};

} // namespace atm
