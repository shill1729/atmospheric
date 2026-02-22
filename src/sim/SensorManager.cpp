#include "sim/SensorManager.hpp"

#include <algorithm>
#include <cmath>

namespace atm {
namespace {
constexpr std::size_t MAX_HISTORY = 30;
}

SensorManager::SensorManager(float sample_period_s, float noise_std)
    : sample_period_s_(std::max(0.1f, sample_period_s))
    , noise_std_(std::max(0.0f, noise_std))
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

void SensorManager::add_sensor(const Vec2& position, float current_time_s) {
    Sensor sensor;
    sensor.position = position;
    sensor.next_sample_time_s = current_time_s + sample_period_s_;
    sensors_.push_back(sensor);
}

void SensorManager::clear() {
    sensors_.clear();
}

void SensorManager::step(float current_time_s, const AdvectionDiffusionSolver& pde, const DomainConfig& domain) {
    for (Sensor& sensor : sensors_) {
        while (current_time_s >= sensor.next_sample_time_s) {
            const float concentration = sample_concentration_bilinear(pde, domain, sensor.position);
            float noisy = concentration;
            if (noise_std_ > 0.0f) {
                noisy += noise_std_ * standard_normal_(rng_);
                noisy = std::max(0.0f, noisy);
            }

            sensor.history.push_back(Observation{sensor.next_sample_time_s, concentration, noisy});
            if (sensor.history.size() > MAX_HISTORY) {
                sensor.history.erase(sensor.history.begin());
            }
            sensor.next_sample_time_s += sample_period_s_;
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

    const float gx = std::clamp((p.x() - domain.x_min) / width, 0.0f, 1.0f) * static_cast<float>(nx - 1);
    const float gy = std::clamp((p.y() - domain.y_min) / height, 0.0f, 1.0f) * static_cast<float>(ny - 1);

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
}

} // namespace atm
