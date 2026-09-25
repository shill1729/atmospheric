#include "numerics/ParticleSystem.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace atm {
namespace {
Mat2 matrix_sqrt_spd(const Mat2& d) {
    const Mat2 sym = 0.5f * (d + d.transpose());

    Eigen::LLT<Mat2> llt(sym);
    if (llt.info() == Eigen::Success) {
        return llt.matrixL();
    }

    Eigen::SelfAdjointEigenSolver<Mat2> es(sym);
    if (es.info() != Eigen::Success) {
        return Mat2::Zero();
    }
    const auto eval_sqrt = es.eigenvalues().cwiseMax(0.0f).cwiseSqrt();
    return es.eigenvectors() * eval_sqrt.asDiagonal();
}
}

ParticleSystem::ParticleSystem(const DomainConfig& domain, const SourceConfig& source, std::size_t max_particles)
    : domain_(domain)
    , source_(source)
    , max_particles_(max_particles)
    , rng_(std::random_device{}())
    , standard_normal_(0.0f, 1.0f) {
    particles_.reserve(max_particles_);
    previous_particles_.reserve(max_particles_);
    trails_.reserve(max_particles_);
}

void ParticleSystem::emit(float emission_rate, float dt, const Vec2& source_position, float birth_multiplier) {
    last_emitted_count_ = 0;
    last_rate_per_second_ = 0.0f;

    if (emission_rate <= 0.0f || dt <= 0.0f || particles_.size() >= max_particles_) {
        return;
    }

    const float mult = std::max(0.0f, birth_multiplier);
    const float expected_count = emission_rate * source_.particle_scale * mult * dt;
    if (expected_count <= 0.0f) {
        return;
    }

    int spawn_count = static_cast<int>(std::floor(expected_count));
    const float frac = expected_count - static_cast<float>(spawn_count);
    std::uniform_real_distribution<float> uniform01(0.0f, 1.0f);
    if (uniform01(rng_) < frac) {
        ++spawn_count;
    }

    const std::size_t remaining = max_particles_ - particles_.size();
    spawn_count = std::min<int>(spawn_count, static_cast<int>(remaining));
    const float spread = std::max(0.0f, source_.particle_spread_fraction) * source_.sigma;
    int emitted_count = 0;
    for (int i = 0; i < spawn_count; ++i) {
        Vec2 p = source_position;
        p.x() += spread * standard_normal_(rng_);
        p.y() += spread * standard_normal_(rng_);
        if (boundary_mode_ == BoundaryMode::Absorbing && !is_inside_domain(p)) {
            continue;
        }
        const Vec2 bounded = apply_boundary(p);
        particles_.push_back(bounded);
        previous_particles_.push_back(bounded);
        trails_.emplace_back(1, bounded);
        ++emitted_count;
    }
    last_emitted_count_ = emitted_count;
    last_rate_per_second_ = static_cast<float>(emitted_count) / dt;
}

void ParticleSystem::step(float time_s, float dt, const Fields& fields, float deposition_rate) {
    if (particles_.empty()) {
        return;
    }

    const float kill_prob = std::clamp(1.0f - std::exp(-deposition_rate * dt), 0.0f, 1.0f);
    std::uniform_real_distribution<float> uniform01(0.0f, 1.0f);

    std::size_t i = 0;
    while (i < particles_.size()) {
        Vec2 x = particles_[i];
        previous_particles_[i] = x;

        const Vec2 drift = fields.wind(time_s, x) + fields.div_diffusivity(time_s, x);
        const Mat2 dmat = fields.diffusivity(time_s, x);
        const Mat2 dsqrt = matrix_sqrt_spd(dmat);

        Vec2 next = x;
        next += drift * dt;
        const Vec2 xi(standard_normal_(rng_), standard_normal_(rng_));
        next += std::sqrt(2.0f * dt) * (dsqrt * xi);
        if (boundary_mode_ == BoundaryMode::Absorbing && !is_inside_domain(next)) {
            particles_[i] = particles_.back();
            previous_particles_[i] = previous_particles_.back();
            trails_[i] = trails_.back();
            particles_.pop_back();
            previous_particles_.pop_back();
            trails_.pop_back();
            continue;
        }
        next = apply_boundary(next);

        if (uniform01(rng_) < kill_prob) {
            particles_[i] = particles_.back();
            previous_particles_[i] = previous_particles_.back();
            trails_[i] = trails_.back();
            particles_.pop_back();
            previous_particles_.pop_back();
            trails_.pop_back();
            continue;
        }

        particles_[i] = next;
        trails_[i].push_back(next);
        if (trails_[i].size() > trail_length_) {
            trails_[i].erase(trails_[i].begin());
        }
        ++i;
    }
}

void ParticleSystem::clear() {
    particles_.clear();
    previous_particles_.clear();
    trails_.clear();
    last_emitted_count_ = 0;
    last_rate_per_second_ = 0.0f;
}

const std::vector<Vec2>& ParticleSystem::particles() const {
    return particles_;
}

const std::vector<Vec2>& ParticleSystem::previous_particles() const {
    return previous_particles_;
}

const std::vector<std::vector<Vec2>>& ParticleSystem::trails() const {
    return trails_;
}

void ParticleSystem::toggle_boundary_mode() {
    switch (boundary_mode_) {
    case BoundaryMode::Periodic:
        boundary_mode_ = BoundaryMode::Reflecting;
        break;
    case BoundaryMode::Reflecting:
        boundary_mode_ = BoundaryMode::Absorbing;
        break;
    case BoundaryMode::Absorbing:
        boundary_mode_ = BoundaryMode::Periodic;
        break;
    }
}

BoundaryMode ParticleSystem::boundary_mode() const {
    return boundary_mode_;
}

int ParticleSystem::last_emitted_count() const {
    return last_emitted_count_;
}

float ParticleSystem::emission_rate_per_second() const {
    return last_rate_per_second_;
}

void ParticleSystem::set_trail_length(std::size_t length) {
    trail_length_ = std::clamp<std::size_t>(length, 2, 180);
    for (auto& trail : trails_) {
        if (trail.size() > trail_length_) {
            trail.erase(trail.begin(), trail.end() - static_cast<std::ptrdiff_t>(trail_length_));
        }
    }
}

bool ParticleSystem::is_inside_domain(const Vec2& p) const {
    return p.x() >= domain_.x_min && p.x() <= domain_.x_max && p.y() >= domain_.y_min && p.y() <= domain_.y_max;
}

std::size_t ParticleSystem::trail_length() const {
    return trail_length_;
}

Vec2 ParticleSystem::apply_boundary(const Vec2& p) const {
    Vec2 out = p;

    const float lx = domain_.x_max - domain_.x_min;
    const float ly = domain_.y_max - domain_.y_min;

    if (lx <= 0.0f || ly <= 0.0f) {
        return Vec2(domain_.x_min, domain_.y_min);
    }

    if (boundary_mode_ == BoundaryMode::Periodic) {
        while (out.x() < domain_.x_min) {
            out.x() += lx;
        }
        while (out.x() > domain_.x_max) {
            out.x() -= lx;
        }

        while (out.y() < domain_.y_min) {
            out.y() += ly;
        }
        while (out.y() > domain_.y_max) {
            out.y() -= ly;
        }
        return out;
    }

    while (out.x() < domain_.x_min || out.x() > domain_.x_max) {
        if (out.x() < domain_.x_min) {
            out.x() = domain_.x_min + (domain_.x_min - out.x());
        }
        if (out.x() > domain_.x_max) {
            out.x() = domain_.x_max - (out.x() - domain_.x_max);
        }
    }

    while (out.y() < domain_.y_min || out.y() > domain_.y_max) {
        if (out.y() < domain_.y_min) {
            out.y() = domain_.y_min + (domain_.y_min - out.y());
        }
        if (out.y() > domain_.y_max) {
            out.y() = domain_.y_max - (out.y() - domain_.y_max);
        }
    }

    return out;
}

} // namespace atm
