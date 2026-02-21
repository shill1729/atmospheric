#include "science/SourceModel.hpp"

#include <algorithm>
#include <cmath>

namespace atm {
namespace {
constexpr float PI = 3.1415926535f;
}

SourceModel::SourceModel(const DomainConfig& domain, const SourceConfig& source)
    : domain_(domain)
    , source_(source) {
}

void SourceModel::activate(const Vec2& position) {
    if (active_sources_.size() >= max_sources()) {
        return;
    }

    ActiveSource src;
    src.position.x() = std::clamp(position.x(), domain_.x_min, domain_.x_max);
    src.position.y() = std::clamp(position.y(), domain_.y_min, domain_.y_max);
    src.age_s = 0.0f;
    active_sources_.push_back(src);
}

void SourceModel::deactivate() {
    active_sources_.clear();
}

void SourceModel::step(float dt) {
    for (auto& src : active_sources_) {
        src.age_s += dt;
    }

    active_sources_.erase(
        std::remove_if(active_sources_.begin(), active_sources_.end(), [&](const ActiveSource& src) {
            return src.age_s >= source_.lifespan;
        }),
        active_sources_.end());
}

bool SourceModel::is_active() const {
    return !active_sources_.empty();
}

float SourceModel::newest_age_s() const {
    if (active_sources_.empty()) {
        return 0.0f;
    }
    return active_sources_.back().age_s;
}

std::size_t SourceModel::active_count() const {
    return active_sources_.size();
}

std::size_t SourceModel::max_sources() const {
    return static_cast<std::size_t>(std::max(1, source_.max_sources));
}

float SourceModel::lifespan_s() const {
    return source_.lifespan;
}

float SourceModel::emission_rate() const {
    float total = 0.0f;
    for (const auto& src : active_sources_) {
        total += emission_rate(src);
    }
    return total;
}

float SourceModel::emission_rate(const ActiveSource& source) const {
    return source_.base_emission * std::exp(-source_.decay_rate * source.age_s);
}

float SourceModel::source_density(const Vec2& x) const {
    if (active_sources_.empty()) {
        return 0.0f;
    }

    const float sigma2 = source_.sigma * source_.sigma;
    const float norm = 1.0f / (2.0f * PI * sigma2);

    float s = 0.0f;
    for (const auto& src : active_sources_) {
        const float r2 = (x - src.position).squaredNorm();
        s += emission_rate(src) * norm * std::exp(-r2 / (2.0f * sigma2));
    }
    return s;
}

const std::vector<SourceModel::ActiveSource>& SourceModel::active_sources() const {
    return active_sources_;
}

} // namespace atm
