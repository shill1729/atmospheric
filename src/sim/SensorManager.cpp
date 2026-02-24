#include "sim/SensorManager.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace atm {
SensorManager::SensorManager(
    float sample_period_s, float noise_std, std::size_t history_capacity, float physical_sample_period_s,
    float spatial_average_radius_m)
    : sample_period_s_(std::max(0.1f, sample_period_s))
    , noise_std_(std::max(0.0f, noise_std))
    , physical_sample_period_s_(std::max(0.01f, physical_sample_period_s))
    , spatial_average_radius_m_(std::max(0.0f, spatial_average_radius_m))
    , history_capacity_(std::max<std::size_t>(1, history_capacity))
    , rng_(std::random_device{}())
    , standard_normal_(0.0f, 1.0f) {
}

void SensorManager::set_sample_period(float seconds) {
    sample_period_s_ = std::max(0.1f, seconds);
}

float SensorManager::sample_period() const {
    return sample_period_s_;
}

void SensorManager::set_noise_std(float stddev) {
    noise_std_ = std::max(0.0f, stddev);
}

float SensorManager::noise_std() const {
    return noise_std_;
}

void SensorManager::set_physical_sample_period(float seconds) {
    physical_sample_period_s_ = std::max(0.01f, seconds);
}

float SensorManager::physical_sample_period() const {
    return physical_sample_period_s_;
}

void SensorManager::set_spatial_average_radius(float radius_m) {
    spatial_average_radius_m_ = std::max(0.0f, radius_m);
}

float SensorManager::spatial_average_radius() const {
    return spatial_average_radius_m_;
}

void SensorManager::set_history_capacity(std::size_t capacity) {
    history_capacity_ = std::max<std::size_t>(1, capacity);
    for (auto& sensor : sensors_) {
        if (sensor.history.size() > history_capacity_) {
            sensor.history.erase(sensor.history.begin(), sensor.history.end() - static_cast<std::ptrdiff_t>(history_capacity_));
        }
    }
}

std::size_t SensorManager::history_capacity() const {
    return history_capacity_;
}

void SensorManager::add_sensor(const Vec2& position, float current_time_s) {
    Sensor sensor;
    sensor.position = position;
    sensor.next_physical_sample_time_s = current_time_s + physical_sample_period_s_;
    sensor.next_report_time_s = current_time_s + sample_period_s_;
    sensor.window_sum = 0.0f;
    sensor.window_noisy_sum = 0.0f;
    sensor.window_count = 0;
    sensors_.push_back(sensor);
}

void SensorManager::clear() {
    sensors_.clear();
}

void SensorManager::step(float current_time_s, const AdvectionDiffusionSolver& pde, const DomainConfig& domain) {
    for (Sensor& sensor : sensors_) {
        while (true) {
            const float next_event_time = std::min(sensor.next_physical_sample_time_s, sensor.next_report_time_s);
            if (current_time_s < next_event_time) {
                break;
            }

            if (sensor.next_physical_sample_time_s <= sensor.next_report_time_s) {
                const float c = sample_concentration_bilinear(pde, domain, sensor.position);
                float noisy = c;
                if (noise_std_ > 0.0f) {
                    noisy += noise_std_ * standard_normal_(rng_);
                    noisy = std::max(0.0f, noisy);
                }
                sensor.window_sum += c;
                sensor.window_noisy_sum += noisy;
                ++sensor.window_count;
                sensor.next_physical_sample_time_s += physical_sample_period_s_;
                continue;
            }

            float concentration = 0.0f;
            float noisy = 0.0f;
            if (sensor.window_count > 0) {
                concentration = sensor.window_sum / static_cast<float>(sensor.window_count);
                noisy = sensor.window_noisy_sum / static_cast<float>(sensor.window_count);
            } else {
                concentration = sample_concentration_bilinear(pde, domain, sensor.position);
                noisy = concentration;
                if (noise_std_ > 0.0f) {
                    noisy += noise_std_ * standard_normal_(rng_);
                    noisy = std::max(0.0f, noisy);
                }
            }

            sensor.history.push_back(Observation{sensor.next_report_time_s, concentration, noisy});
            if (sensor.history.size() > history_capacity_) {
                sensor.history.erase(sensor.history.begin());
            }

            sensor.window_sum = 0.0f;
            sensor.window_noisy_sum = 0.0f;
            sensor.window_count = 0;
            sensor.next_report_time_s += sample_period_s_;
        }
    }
}

const std::vector<SensorManager::Sensor>& SensorManager::sensors() const {
    return sensors_;
}

float SensorManager::sample_concentration_bilinear(
    const AdvectionDiffusionSolver& pde, const DomainConfig& domain, const Vec2& p) const {
    const int nx = pde.nx();
    const int ny = pde.ny();
    const auto& c = pde.concentration();
    if (nx < 2 || ny < 2 || c.empty()) {
        return 0.0f;
    }

    const float width = domain.x_max - domain.x_min;
    const float height = domain.y_max - domain.y_min;
    if (!(width > 0.0f) || !(height > 0.0f)) {
        return 0.0f;
    }

    const auto sample_point = [&](const Vec2& q) {
        const float gx = std::clamp((q.x() - domain.x_min) / width, 0.0f, 1.0f) * static_cast<float>(nx - 1);
        const float gy = std::clamp((q.y() - domain.y_min) / height, 0.0f, 1.0f) * static_cast<float>(ny - 1);

        const int i0 = static_cast<int>(std::floor(gx));
        const int j0 = static_cast<int>(std::floor(gy));
        const int i1 = std::min(i0 + 1, nx - 1);
        const int j1 = std::min(j0 + 1, ny - 1);

        const float tx = gx - static_cast<float>(i0);
        const float ty = gy - static_cast<float>(j0);

        const auto idx = [nx](int i, int j) { return j * nx + i; };
        const float c00 = c[idx(i0, j0)];
        const float c10 = c[idx(i1, j0)];
        const float c01 = c[idx(i0, j1)];
        const float c11 = c[idx(i1, j1)];

        const float c0 = (1.0f - tx) * c00 + tx * c10;
        const float c1 = (1.0f - tx) * c01 + tx * c11;
        return (1.0f - ty) * c0 + ty * c1;
    };

    if (spatial_average_radius_m_ <= 1.0e-6f) {
        return sample_point(p);
    }

    // Disk-like local averaging stencil around the sensor location.
    const float r = spatial_average_radius_m_;
    const float r_half = 0.5f * r;
    const std::array<Vec2, 13> offsets{
        Vec2(0.0f, 0.0f),
        Vec2(r, 0.0f),
        Vec2(-r, 0.0f),
        Vec2(0.0f, r),
        Vec2(0.0f, -r),
        Vec2(0.70710678f * r, 0.70710678f * r),
        Vec2(-0.70710678f * r, 0.70710678f * r),
        Vec2(0.70710678f * r, -0.70710678f * r),
        Vec2(-0.70710678f * r, -0.70710678f * r),
        Vec2(r_half, 0.0f),
        Vec2(-r_half, 0.0f),
        Vec2(0.0f, r_half),
        Vec2(0.0f, -r_half),
    };
    float sum = 0.0f;
    for (const Vec2& off : offsets) {
        sum += sample_point(p + off);
    }
    return sum / static_cast<float>(offsets.size());
}

} // namespace atm
