#include "sim/Simulator.hpp"

#include <algorithm>
#include <chrono>
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

void Simulator::step(float frame_dt, const std::function<void()>& after_step) {
    if (paused_) {
        last_emitted_total_ = 0;
        last_emission_rate_per_second_ = 0.0f;
        achieved_time_scale_ = 0.0f;
        return;
    }

    using Clock = std::chrono::steady_clock;
    const auto frame_start = Clock::now();
    const auto budget = std::chrono::duration<float, std::milli>(std::max(0.0f, config_.numerics.frame_step_budget_ms));

    accumulator_ += frame_dt * time_scale_runtime_;
    int substeps = 0;
    int emitted_frame = 0;
    float sim_dt_frame = 0.0f;
    bool out_of_budget = false;
    while (accumulator_ >= config_.numerics.dt && substeps < config_.numerics.max_substeps_per_frame) {
        emitted_frame += step_fixed(config_.numerics.dt);
        if (after_step) {
            after_step();
        }
        sim_dt_frame += config_.numerics.dt;
        accumulator_ -= config_.numerics.dt;
        ++substeps;
        if (Clock::now() - frame_start >= budget) {
            out_of_budget = accumulator_ >= config_.numerics.dt;
            break;
        }
    }

    if (frame_dt > 0.0f) {
        const float rate = sim_dt_frame / frame_dt;
        achieved_time_scale_ = achieved_time_scale_ <= 0.0f ? rate : 0.9f * achieved_time_scale_ + 0.1f * rate;
    }

    if (substeps > 0) {
        last_emitted_total_ = static_cast<int>(std::lround(static_cast<float>(emitted_frame) / substeps));
        last_emission_rate_per_second_ = sim_dt_frame > 0.0f ? static_cast<float>(emitted_frame) / sim_dt_frame : 0.0f;
    } else {
        last_emitted_total_ = 0;
        last_emission_rate_per_second_ = 0.0f;
    }

    // Drop time the CPU could not keep up with instead of carrying a
    // growing backlog into later frames.
    if (out_of_budget || substeps == config_.numerics.max_substeps_per_frame) {
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

Vec2 Simulator::wind_at_time(float time_s, const Vec2& x) const {
    return fields_.wind(time_s, x);
}

Mat2 Simulator::diffusivity_at_time(float time_s, const Vec2& x) const {
    return fields_.diffusivity(time_s, x);
}

void Simulator::scale_time(float factor) {
    time_scale_runtime_ = std::clamp(time_scale_runtime_ * factor, kMinTimeScale, kMaxTimeScale);
}

void Simulator::set_time_scale(float value) {
    time_scale_runtime_ = std::clamp(value, kMinTimeScale, kMaxTimeScale);
}

void Simulator::reset_time_scale() {
    time_scale_runtime_ = config_.numerics.time_scale;
}

float Simulator::time_scale() const {
    return time_scale_runtime_;
}

float Simulator::achieved_time_scale() const {
    return achieved_time_scale_;
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

Fields::WindPreset Simulator::wind_preset() const {
    return fields_.wind_preset();
}

std::string_view Simulator::wind_model_name() const {
    return fields_.wind_preset_name();
}

void Simulator::cycle_diffusion_model(int direction) {
    fields_.cycle_diffusivity_preset(direction);
}

Fields::DiffusivityPreset Simulator::diffusivity_preset() const {
    return fields_.diffusivity_preset();
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
    particles_.step(time_s_, dt, fields_, config_.physics.deposition_rate);
    pde_.step(time_s_, dt, fields_, source_, particles_.boundary_mode());
    source_.step(dt);
    time_s_ += dt;
    return emitted_sum;
}

} // namespace atm
