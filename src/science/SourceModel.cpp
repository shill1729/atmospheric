#include "science/SourceModel.hpp"

#include <algorithm>
#include <cmath>

namespace atm {
namespace {
constexpr float PI = 3.1415926535f;
}

SourceModel::SourceModel(const DomainConfig& domain, const SourceConfig& source)
    : domain_(domain), source_(source) {
    position_ = Vec2(0.5f * (domain_.x_min + domain_.x_max), 0.5f * (domain_.y_min + domain_.y_max));
}

void SourceModel::activate(const Vec2& position) {
    position_.x() = std::clamp(position.x(), domain_.x_min, domain_.x_max);
    position_.y() = std::clamp(position.y(), domain_.y_min, domain_.y_max);
    age_ = 0.0f;
    active_ = true;
}

void SourceModel::deactivate() {
    active_ = false;
    age_ = 0.0f;
}

void SourceModel::step(float dt) {
    if (!active_) {
        return;
    }
    age_ += dt;
    if (age_ >= source_.lifespan) {
        active_ = false;
    }
}

bool SourceModel::is_active() const {
    return active_;
}

float SourceModel::age_s() const {
    return age_;
}

float SourceModel::lifespan_s() const {
    return source_.lifespan;
}

float SourceModel::emission_rate() const {
    if (!active_) {
        return 0.0f;
    }
    return source_.base_emission * std::exp(-source_.decay_rate * age_);
}

float SourceModel::source_density(const Vec2& x) const {
    if (!active_) {
        return 0.0f;
    }

    const float sigma2 = source_.sigma * source_.sigma;
    const float r2 = (x - position_).squaredNorm();
    const float norm = 1.0f / (2.0f * PI * sigma2);
    return emission_rate() * norm * std::exp(-r2 / (2.0f * sigma2));
}

const Vec2& SourceModel::position() const {
    return position_;
}

} // namespace atm
