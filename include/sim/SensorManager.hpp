#pragma once

#include "core/Config.hpp"
#include "core/Types.hpp"
#include "numerics/AdvectionDiffusionSolver.hpp"

#include <cstddef>
#include <random>
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
        float next_sample_time_s = 0.0f;
        std::vector<Observation> history;
    };

    SensorManager(float sample_period_s, float noise_std, std::size_t history_capacity = 30);

    void set_sample_period(float seconds);
    float sample_period() const;
    void set_noise_std(float stddev);
    float noise_std() const;
    void set_history_capacity(std::size_t capacity);
    std::size_t history_capacity() const;

    void add_sensor(const Vec2& position, float current_time_s);
    void clear();
    void step(float current_time_s, const AdvectionDiffusionSolver& pde, const DomainConfig& domain);

    const std::vector<Sensor>& sensors() const;

private:
    float sample_concentration_bilinear(const AdvectionDiffusionSolver& pde, const DomainConfig& domain, const Vec2& p) const;

    std::vector<Sensor> sensors_;
    float sample_period_s_ = 5.0f;
    float noise_std_ = 0.0f;
    std::size_t history_capacity_ = 30;
    std::mt19937 rng_;
    std::normal_distribution<float> standard_normal_;
};

} // namespace atm
