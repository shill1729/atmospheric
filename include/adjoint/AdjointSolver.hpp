#pragma once

#include "core/Config.hpp"
#include "core/Types.hpp"

#include <functional>
#include <vector>

namespace atm {

class AdjointSolver {
public:
    enum class DiffusionMode {
        ScalarizedTrace = 0,
        FullTensorFlux = 1
    };

    struct Config {
        DomainConfig domain;
        int nx = 96;
        int ny = 96;
        float dt_s = 0.5f;
        float deposition_rate = 0.0f;
        DiffusionMode diffusion_mode = DiffusionMode::ScalarizedTrace;
    };

    struct Snapshot {
        float time_s = 0.0f;
        std::vector<float> phi;
    };

    struct Solution {
        int nx = 0;
        int ny = 0;
        float dx = 1.0f;
        float dy = 1.0f;
        std::vector<Snapshot> snapshots;
    };

    using WindFn = std::function<Vec2(float time_s, const Vec2&)>;
    using DiffusivityFn = std::function<Mat2(float time_s, const Vec2&)>;
    using ForcingFn = std::function<void(float time_s, std::vector<float>& out)>;

    Solution solve_backward(
        float t_start_s, float t_end_s, const Config& cfg, const WindFn& wind_fn, const DiffusivityFn& diffusivity_fn,
        const ForcingFn& forcing_fn) const;
};

} // namespace atm
