#include "sim/Simulator.hpp"

#include <algorithm>

namespace atm {

Simulator::Simulator(const Config& config)
    : config_(config)
    , fields_(config_.domain)
    , source_(config_.domain, config_.source)
    , particles_(config_.domain, config_.source, config_.numerics.max_particles)
    , pde_(config_.domain, config_.physics.deposition_rate)
    , time_scale_runtime_(config_.numerics.time_scale) {
}

void Simulator::step(float frame_dt) {
    if (paused_) {
        return;
    }

    accumulator_ += frame_dt * time_scale_runtime_;
    int substeps = 0;
    while (accumulator_ >= config_.numerics.dt && substeps < config_.numerics.max_substeps_per_frame) {
        step_fixed(config_.numerics.dt);
        accumulator_ -= config_.numerics.dt;
        ++substeps;
    }

    if (substeps == config_.numerics.max_substeps_per_frame) {
        accumulator_ = 0.0f;
    }
}

void Simulator::set_paused(bool paused) {
    paused_ = paused;
}

bool Simulator::paused() const {
    return paused_;
}

void Simulator::toggle_paused() {
    paused_ = !paused_;
}

void Simulator::reset() {
    time_s_ = 0.0f;
    accumulator_ = 0.0f;
    particles_.clear();
    pde_.reset();
    source_.deactivate();
}

void Simulator::set_source(const Vec2& x) {
    source_.activate(x);
}

void Simulator::toggle_boundary_mode() {
    particles_.toggle_boundary_mode();
}

BoundaryMode Simulator::boundary_mode() const {
    return particles_.boundary_mode();
}

Vec2 Simulator::wind_at(const Vec2& x) const {
    return fields_.wind(time_s_, x);
}

void Simulator::scale_time(float factor) {
    time_scale_runtime_ = std::clamp(time_scale_runtime_ * factor, 0.25f, 120.0f);
}

void Simulator::reset_time_scale() {
    time_scale_runtime_ = config_.numerics.time_scale;
}

float Simulator::time_scale() const {
    return time_scale_runtime_;
}

void Simulator::adjust_trail_length(int delta) {
    const std::size_t current = particles_.trail_length();
    const int next = static_cast<int>(current) + delta;
    particles_.set_trail_length(static_cast<std::size_t>(std::max(2, next)));
}

std::size_t Simulator::trail_length() const {
    return particles_.trail_length();
}

void Simulator::cycle_wind_model(int direction) {
    fields_.cycle_wind_preset(direction);
}

std::string_view Simulator::wind_model_name() const {
    return fields_.wind_preset_name();
}

const ParticleSystem& Simulator::particles() const {
    return particles_;
}

const AdvectionDiffusionSolver& Simulator::pde() const {
    return pde_;
}

const SourceModel& Simulator::source() const {
    return source_;
}

const Config& Simulator::config() const {
    return config_;
}

float Simulator::time_s() const {
    return time_s_;
}

void Simulator::step_fixed(float dt) {
    source_.step(dt);
    for (const auto& src : source_.active_sources()) {
        particles_.emit(source_.emission_rate(src), dt, src.position);
    }
    particles_.step(time_s_, dt, fields_, config_.physics.deposition_rate);
    pde_.step(time_s_, dt, fields_, source_);
    time_s_ += dt;
}

} // namespace atm
