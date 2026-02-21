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
    void scale_time(float factor);
    void reset_time_scale();
    float time_scale() const;
    void adjust_trail_length(int delta);
    std::size_t trail_length() const;
    void cycle_wind_model(int direction);
    std::string_view wind_model_name() const;

    const ParticleSystem& particles() const;
    const AdvectionDiffusionSolver& pde() const;
    const SourceModel& source() const;
    const Config& config() const;
    float time_s() const;

private:
    void step_fixed(float dt);

    const Config config_;
    Fields fields_;
    SourceModel source_;
    ParticleSystem particles_;
    AdvectionDiffusionSolver pde_;

    float time_s_ = 0.0f;
    float accumulator_ = 0.0f;
    bool paused_ = false;
    float time_scale_runtime_ = 1.0f;
};

} // namespace atm
