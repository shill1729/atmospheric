#pragma once

#include "core/Config.hpp"
#include "core/Types.hpp"
#include "science/Fields.hpp"

#include <cstddef>
#include <random>
#include <vector>

namespace atm {

enum class BoundaryMode {
    Reflecting,
    Periodic,
    Absorbing
};

class ParticleSystem {
public:
    ParticleSystem(const DomainConfig& domain, const SourceConfig& source, std::size_t max_particles);

    void emit(float emission_rate, float dt, const Vec2& source_position, float birth_multiplier = 1.0f);
    void step(float time_s, float dt, const Fields& fields, float deposition_rate);
    void toggle_boundary_mode();
    BoundaryMode boundary_mode() const;
    int last_emitted_count() const;
    float emission_rate_per_second() const;
    void set_trail_length(std::size_t length);
    std::size_t trail_length() const;

    void clear();
    const std::vector<Vec2>& particles() const;
    const std::vector<Vec2>& previous_particles() const;
    const std::vector<std::vector<Vec2>>& trails() const;

private:
    bool is_inside_domain(const Vec2& p) const;
    Vec2 apply_boundary(const Vec2& p) const;

    const DomainConfig& domain_;
    const SourceConfig& source_;
    std::size_t max_particles_;
    BoundaryMode boundary_mode_ = BoundaryMode::Absorbing;

    std::vector<Vec2> particles_;
    std::vector<Vec2> previous_particles_;
    std::vector<std::vector<Vec2>> trails_;
    std::mt19937 rng_;
    std::normal_distribution<float> standard_normal_;
    int last_emitted_count_ = 0;
    float last_rate_per_second_ = 0.0f;
    std::size_t trail_length_ = 30;
};

} // namespace atm
