#include "sim/Simulator.hpp"

#include <algorithm>
#include <cmath>

namespace atm {

Simulator::Simulator(const Config& config)
    : config_(config)
    , fields_(config_.domain, config_.physics)
    , source_(config_.domain, config_.source)
    , particles_(config_.domain, config_.source, config_.numerics.max_particles)
    , pde_(config_.domain, config_.physics.deposition_rate)
    , time_scale_runtime_(config_.numerics.time_scale) {
    const float base_scale = std::max(1.0e-6f, config_.source.particle_scale);
    particle_mass_ = 1.0f / (base_scale * particle_birth_multiplier_);
}

void Simulator::step(float frame_dt) {
    if (paused_) {
        last_emitted_total_ = 0;
        last_emission_rate_per_second_ = 0.0f;
        return;
    }

    accumulator_ += frame_dt * time_scale_runtime_;
    int substeps = 0;
    int emitted_frame = 0;
    float sim_dt_frame = 0.0f;
    while (accumulator_ >= config_.numerics.dt && substeps < config_.numerics.max_substeps_per_frame) {
        emitted_frame += step_fixed(config_.numerics.dt);
        sim_dt_frame += config_.numerics.dt;
        accumulator_ -= config_.numerics.dt;
        ++substeps;
    }

    if (substeps > 0) {
        last_emitted_total_ = static_cast<int>(std::lround(static_cast<float>(emitted_frame) / substeps));
        last_emission_rate_per_second_ = sim_dt_frame > 0.0f ? static_cast<float>(emitted_frame) / sim_dt_frame : 0.0f;
    } else {
        last_emitted_total_ = 0;
        last_emission_rate_per_second_ = 0.0f;
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
    last_emitted_total_ = 0;
    last_emission_rate_per_second_ = 0.0f;
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

void Simulator::set_time_scale(float value) {
    time_scale_runtime_ = std::clamp(value, 0.25f, 120.0f);
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

void Simulator::cycle_diffusion_model(int direction) {
    fields_.cycle_diffusivity_preset(direction);
}

std::string_view Simulator::diffusion_model_name() const {
    return fields_.diffusivity_preset_name();
}

void Simulator::cycle_pde_diffusion_mode(int direction) {
    pde_.cycle_diffusion_mode(direction);
}

AdvectionDiffusionSolver::DiffusionMode Simulator::pde_diffusion_mode() const {
    return pde_.diffusion_mode();
}

void Simulator::set_pde_diffusion_mode(AdvectionDiffusionSolver::DiffusionMode mode) {
    pde_.set_diffusion_mode(mode);
}

std::string_view Simulator::pde_diffusion_mode_name() const {
    return pde_.diffusion_mode_name();
}

void Simulator::toggle_brownian_heat_case() {
    if (brownian_heat_case()) {
        fields_.set_wind_preset(Fields::WindPreset::JetShear);
        fields_.set_diffusivity_preset(Fields::DiffusivityPreset::ConstantScalar);
        pde_.set_diffusion_mode(AdvectionDiffusionSolver::DiffusionMode::ScalarizedTrace);
        particle_birth_multiplier_ = 1.0f;
    } else {
        fields_.set_wind_preset(Fields::WindPreset::Zero);
        fields_.set_diffusivity_preset(Fields::DiffusivityPreset::BrownianHalf);
        pde_.set_diffusion_mode(AdvectionDiffusionSolver::DiffusionMode::ScalarizedTrace);
        particle_birth_multiplier_ = 12.0f;
    }
    const float base_scale = std::max(1.0e-6f, config_.source.particle_scale);
    particle_mass_ = 1.0f / (base_scale * particle_birth_multiplier_);
}

bool Simulator::brownian_heat_case() const {
    return fields_.wind_preset() == Fields::WindPreset::Zero
        && fields_.diffusivity_preset() == Fields::DiffusivityPreset::BrownianHalf;
}

int Simulator::last_emitted_total() const {
    return last_emitted_total_;
}

float Simulator::last_emission_rate_per_second() const {
    return last_emission_rate_per_second_;
}

float Simulator::particle_mass() const {
    return particle_mass_;
}

float Simulator::sde_total_mass() const {
    return static_cast<float>(particles_.particles().size()) * particle_mass_;
}

float Simulator::pde_total_mass() const {
    return pde_.total_mass();
}

float Simulator::mass_ratio_sde_to_pde() const {
    const float pde_m = pde_total_mass();
    if (pde_m <= 1.0e-12f) {
        return 0.0f;
    }
    return sde_total_mass() / pde_m;
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

int Simulator::step_fixed(float dt) {
    int emitted_sum = 0;
    for (const auto& src : source_.active_sources()) {
        particles_.emit(source_.emission_rate(src), dt, src.position, particle_birth_multiplier_);
        emitted_sum += particles_.last_emitted_count();
    }
    source_.step(dt);
    particles_.step(time_s_, dt, fields_, config_.physics.deposition_rate);
    pde_.step(time_s_, dt, fields_, source_, particles_.boundary_mode());
    time_s_ += dt;
    return emitted_sum;
}

} // namespace atm
