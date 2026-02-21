#pragma once

#include "core/Config.hpp"
#include "numerics/ParticleSystem.hpp"
#include "science/Fields.hpp"
#include "science/SourceModel.hpp"

#include <vector>

namespace atm {

class AdvectionDiffusionSolver {
public:
    AdvectionDiffusionSolver(const DomainConfig& domain, float deposition_rate);

    void reset();
    void step(float time_s, float dt, const Fields& fields, const SourceModel& source, BoundaryMode boundary_mode);

    int nx() const;
    int ny() const;
    float dx() const;
    float dy() const;
    const std::vector<float>& concentration() const;
    float max_concentration() const;
    float total_mass() const;

private:
    int idx(int i, int j) const;
    float sample(const std::vector<float>& c, int i, int j, BoundaryMode boundary_mode) const;

    DomainConfig domain_;
    float deposition_rate_ = 0.0f;
    float dx_ = 1.0f;
    float dy_ = 1.0f;

    std::vector<float> c_;
    std::vector<float> c_next_;
};

} // namespace atm
