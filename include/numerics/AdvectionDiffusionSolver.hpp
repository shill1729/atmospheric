#pragma once

#include "core/Config.hpp"
#include "numerics/ParticleSystem.hpp"
#include "science/Fields.hpp"
#include "science/SourceModel.hpp"

#include <string_view>
#include <vector>

namespace atm {

class AdvectionDiffusionSolver {
public:
    enum class DiffusionMode {
        ScalarizedTrace = 0,
        FullTensorFlux = 1
    };

    AdvectionDiffusionSolver(const DomainConfig& domain, float deposition_rate);

    void reset();
    void step(float time_s, float dt, const Fields& fields, const SourceModel& source, BoundaryMode boundary_mode);
    void cycle_diffusion_mode(int direction);
    DiffusionMode diffusion_mode() const;
    void set_diffusion_mode(DiffusionMode mode);
    std::string_view diffusion_mode_name() const;

    int nx() const;
    int ny() const;
    float dx() const;
    float dy() const;
    const std::vector<float>& concentration() const;
    float max_concentration() const;
    float total_mass() const;
    // Approximate explicit-Euler stability limit on dt for the fields seen
    // in the most recent step (advective CFL + diffusive + deposition rates,
    // maximized over the grid). Infinity before the first step.
    float max_stable_dt() const;

private:
    int idx(int i, int j) const;
    int map_index(int i, int n, BoundaryMode boundary_mode) const;
    float sample(const std::vector<float>& c, int i, int j, BoundaryMode boundary_mode) const;
    Mat2 sample_diffusivity(const std::vector<Mat2>& d, int i, int j, BoundaryMode boundary_mode) const;
    Vec2 sample_wind(const std::vector<Vec2>& w, int i, int j, BoundaryMode boundary_mode) const;
    float sample_source(const std::vector<float>& s, int i, int j, BoundaryMode boundary_mode) const;

    DomainConfig domain_;
    float deposition_rate_ = 0.0f;
    float dx_ = 1.0f;
    float dy_ = 1.0f;

    std::vector<float> c_;
    std::vector<float> c_next_;
    std::vector<Vec2> wind_cache_;
    std::vector<Mat2> diff_cache_;
    std::vector<float> source_cache_;
    float max_rate_ = 0.0f;
    DiffusionMode diffusion_mode_ = DiffusionMode::FullTensorFlux;
};

} // namespace atm
