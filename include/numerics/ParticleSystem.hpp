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
    Periodic
};

class ParticleSystem {
public:
    ParticleSystem(const DomainConfig& domain, const SourceConfig& source, std::size_t max_particles);

    void emit(float emission_rate, float dt, const Vec2& source_position);
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
    Vec2 apply_boundary(const Vec2& p) const;

    const DomainConfig& domain_;
    const SourceConfig& source_;
    std::size_t max_particles_;
    BoundaryMode boundary_mode_ = BoundaryMode::Periodic;

    std::vector<Vec2> particles_;
    std::vector<Vec2> previous_particles_;
    std::vector<std::vector<Vec2>> trails_;
    std::mt19937 rng_;
    std::normal_distribution<float> standard_normal_;
    float emission_carry_ = 0.0f;
    int last_emitted_count_ = 0;
    float last_rate_per_second_ = 0.0f;
    std::size_t trail_length_ = 14;
};

} // namespace atm
