#pragma once

#include "core/Config.hpp"
#include "core/Types.hpp"
#include "numerics/AdvectionDiffusionSolver.hpp"
#include "numerics/ParticleSystem.hpp"
#include "science/Fields.hpp"
#include "science/SourceModel.hpp"

#include <string_view>

namespace atm {

class Simulator {
public:
    explicit Simulator(const Config& config);

    void step(float frame_dt);
    void set_paused(bool paused);
    bool paused() const;
    void toggle_paused();

    void reset();
    void set_source(const Vec2& x);
    void toggle_boundary_mode();
    BoundaryMode boundary_mode() const;
    Vec2 wind_at(const Vec2& x) const;
    Vec2 wind_at_time(float time_s, const Vec2& x) const;
    Mat2 diffusivity_at_time(float time_s, const Vec2& x) const;
    void scale_time(float factor);
    void set_time_scale(float value);
    void reset_time_scale();
    float time_scale() const;
    void adjust_trail_length(int delta);
    std::size_t trail_length() const;
    void cycle_wind_model(int direction);
    Fields::WindPreset wind_preset() const;
    std::string_view wind_model_name() const;
    void cycle_diffusion_model(int direction);
    Fields::DiffusivityPreset diffusivity_preset() const;
    std::string_view diffusion_model_name() const;
    void cycle_pde_diffusion_mode(int direction);
    AdvectionDiffusionSolver::DiffusionMode pde_diffusion_mode() const;
    void set_pde_diffusion_mode(AdvectionDiffusionSolver::DiffusionMode mode);
    std::string_view pde_diffusion_mode_name() const;
    void toggle_brownian_heat_case();
    bool brownian_heat_case() const;
    int last_emitted_total() const;
    float last_emission_rate_per_second() const;
    float particle_mass() const;
    float sde_total_mass() const;
    float pde_total_mass() const;
    float mass_ratio_sde_to_pde() const;

    const ParticleSystem& particles() const;
    const AdvectionDiffusionSolver& pde() const;
    const SourceModel& source() const;
    const Config& config() const;
    float time_s() const;

private:
    int step_fixed(float dt);

    const Config config_;
    Fields fields_;
    SourceModel source_;
    ParticleSystem particles_;
    AdvectionDiffusionSolver pde_;

    float time_s_ = 0.0f;
    float accumulator_ = 0.0f;
    bool paused_ = false;
    float time_scale_runtime_ = 1.0f;
    int last_emitted_total_ = 0;
    float last_emission_rate_per_second_ = 0.0f;
    float particle_mass_ = 1.0f;
    float particle_birth_multiplier_ = 1.0f;
};

} // namespace atm
