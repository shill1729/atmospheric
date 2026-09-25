#pragma once

#include "core/Config.hpp"
#include "core/Types.hpp"
#include "numerics/AdvectionDiffusionSolver.hpp"

#include <cstddef>
#include <random>
#include <string>
#include <vector>

namespace atm {

class SensorManager {
public:
    struct Observation {
        float time_s = 0.0f;
        float concentration = 0.0f;
        float noisy_concentration = 0.0f;
    };

    struct Sensor {
        Vec2 position = Vec2::Zero();
        std::string label;
        float next_physical_sample_time_s = 0.0f;
        float next_report_time_s = 0.0f;
        float window_sum = 0.0f;
        float window_noisy_sum = 0.0f;
        int window_count = 0;
        std::vector<Observation> history;
    };

    SensorManager(
        float sample_period_s, float noise_std, std::size_t history_capacity = 30, float physical_sample_period_s = 1.0f,
        float spatial_average_radius_m = 0.0f);

    void set_sample_period(float seconds);
    float sample_period() const;
    void set_noise_std(float stddev);
    float noise_std() const;
    void set_physical_sample_period(float seconds);
    float physical_sample_period() const;
    void set_spatial_average_radius(float radius_m);
    float spatial_average_radius() const;
    // Report level that noise alone exceeds, anywhere among `reports_tested`
    // reports, with probability ~family_false_alarm (a Bonferroni bound, so
    // a longer history or more sensors raise the bar accordingly). Each
    // physical sample is max(0, c + N(0, noise_std^2)), so pure noise has
    // mean 0.40*noise_std and std 0.58*noise_std per sample, averaged over
    // the samples in one report window. 0 when noise is off.
    float noise_detection_floor(std::size_t reports_tested, float family_false_alarm = 0.05f) const;
    void set_history_capacity(std::size_t capacity);
    std::size_t history_capacity() const;

    struct PendingReport {
        std::size_t sensor_index;
        float time_s;
        float concentration;
        float noisy_concentration;
    };

    void add_sensor(const Vec2& position, float current_time_s, const std::string& label = "");
    void clear();
    void step(float current_time_s, const AdvectionDiffusionSolver& pde, const DomainConfig& domain);
    float sample_concentration(
        const AdvectionDiffusionSolver& pde, const DomainConfig& domain, const Vec2& position) const;

    const std::vector<Sensor>& sensors() const;
    const std::vector<PendingReport>& pending_reports() const;
    void clear_pending_reports();

private:
    std::vector<Sensor> sensors_;
    std::vector<PendingReport> pending_reports_;
    float sample_period_s_ = 5.0f;
    float noise_std_ = 0.0f;
    float physical_sample_period_s_ = 1.0f;
    float spatial_average_radius_m_ = 0.0f;
    std::size_t history_capacity_ = 30;
    std::mt19937 rng_;
    std::normal_distribution<float> standard_normal_;
};

} // namespace atm
