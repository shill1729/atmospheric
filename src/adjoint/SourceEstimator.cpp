#include "adjoint/SourceEstimator.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace atm {
namespace {
struct SensorSeries {
    Vec2 position = Vec2::Zero();
    std::vector<SensorManager::Observation> obs;
    std::vector<float> bump;
};

int idx(int i, int j, int nx) {
    return j * nx + i;
}

int last_obs_at_or_before(const std::vector<SensorManager::Observation>& obs, float t) {
    int out = -1;
    for (int i = 0; i < static_cast<int>(obs.size()); ++i) {
        if (obs[static_cast<std::size_t>(i)].time_s <= t) {
            out = i;
        } else {
            break;
        }
    }
    return out;
}
} // namespace

SourceEstimator::SourceEstimator(SourceEstimationConfig cfg)
    : cfg_(cfg) {
}

const SourceEstimationConfig& SourceEstimator::config() const {
    return cfg_;
}

SourceEstimateResult SourceEstimator::estimate(const std::vector<SensorManager::Sensor>& sensors, const Simulator& sim) const {
    SourceEstimateResult out;

    const float t_end = sim.time_s();
    const float t_lower = std::max(0.0f, t_end - std::max(1.0f, cfg_.max_lookback_s));

    std::vector<SensorSeries> series;
    series.reserve(sensors.size());

    float t_start = t_end;
    bool has_gated_signal = false;

    for (const auto& sensor : sensors) {
        if (sensor.history.empty()) {
            continue;
        }

        SensorSeries s;
        s.position = sensor.position;
        const int keep = std::max(1, cfg_.max_samples_per_sensor);
        const int begin = std::max(0, static_cast<int>(sensor.history.size()) - keep);
        for (int i = begin; i < static_cast<int>(sensor.history.size()); ++i) {
            const auto& obs = sensor.history[static_cast<std::size_t>(i)];
            if (obs.time_s < t_lower || obs.time_s > t_end) {
                continue;
            }
            s.obs.push_back(obs);
            t_start = std::min(t_start, obs.time_s);
            if (obs.noisy_concentration > cfg_.detection_threshold) {
                has_gated_signal = true;
            }
        }
        if (!s.obs.empty()) {
            series.push_back(std::move(s));
        }
    }

    if (series.empty()) {
        out.insufficient_signal = true;
        out.message = "No sensor history available in lookback window.";
        return out;
    }
    if (!has_gated_signal) {
        out.insufficient_signal = true;
        out.message = "Insufficient signal: no sensor exceeds detection threshold.";
        return out;
    }

    const auto& domain = sim.config().domain;
    const int nx = std::max(2, cfg_.adjoint_grid_nx);
    const int ny = std::max(2, cfg_.adjoint_grid_ny);
    const float dx = (domain.x_max - domain.x_min) / static_cast<float>(nx - 1);
    const float dy = (domain.y_max - domain.y_min) / static_cast<float>(ny - 1);
    const float cell_area = dx * dy;

    const float sigma = std::max(1.0f, cfg_.gaussian_sigma);
    const float inv_2s2 = 1.0f / (2.0f * sigma * sigma);
    const std::size_t n = static_cast<std::size_t>(nx * ny);

    for (auto& s : series) {
        s.bump.assign(n, 0.0f);
        float integ = 0.0f;
        for (int j = 0; j < ny; ++j) {
            const float y = domain.y_min + static_cast<float>(j) * dy;
            for (int i = 0; i < nx; ++i) {
                const float x = domain.x_min + static_cast<float>(i) * dx;
                const float ddx = x - s.position.x();
                const float ddy = y - s.position.y();
                const float g = std::exp(-(ddx * ddx + ddy * ddy) * inv_2s2);
                s.bump[static_cast<std::size_t>(idx(i, j, nx))] = g;
                integ += g * cell_area;
            }
        }
        const float beta = integ > 1.0e-12f ? 1.0f / integ : 0.0f;
        for (float& v : s.bump) {
            v *= beta;
        }
    }

    AdjointSolver::Config solve_cfg;
    solve_cfg.domain = domain;
    solve_cfg.nx = nx;
    solve_cfg.ny = ny;
    solve_cfg.dt_s = std::max(0.05f, cfg_.adjoint_dt_s);
    solve_cfg.deposition_rate = sim.config().physics.deposition_rate;
    solve_cfg.diffusion_mode = sim.pde_diffusion_mode() == AdvectionDiffusionSolver::DiffusionMode::FullTensorFlux
        ? AdjointSolver::DiffusionMode::FullTensorFlux
        : AdjointSolver::DiffusionMode::ScalarizedTrace;

    auto wind_fn = [&](float time_s, const Vec2& p) { return sim.wind_at_time(time_s, p); };
    auto diffusivity_fn = [&](float time_s, const Vec2& p) { return sim.diffusivity_at_time(time_s, p); };
    auto forcing_fn = [&](float time_s, std::vector<float>& f) {
        f.assign(n, 0.0f);
        for (const auto& s : series) {
            const int obs_idx = last_obs_at_or_before(s.obs, time_s);
            if (obs_idx < 0) {
                continue;
            }
            if (s.obs[static_cast<std::size_t>(obs_idx)].noisy_concentration <= cfg_.detection_threshold) {
                continue;
            }
            for (std::size_t k = 0; k < n; ++k) {
                f[k] += s.bump[k];
            }
        }
    };

    const auto sol = solver_.solve_backward(t_start, t_end, solve_cfg, wind_fn, diffusivity_fn, forcing_fn);

    std::vector<float> z_by_snapshot(sol.snapshots.size(), 0.0f);
    float best_z = -std::numeric_limits<float>::infinity();
    int best_snapshot = -1;
    for (int k = 0; k < static_cast<int>(sol.snapshots.size()); ++k) {
        const auto& snap = sol.snapshots[static_cast<std::size_t>(k)].phi;
        float z = 0.0f;
        for (float v : snap) {
            z += v * cell_area;
        }
        z_by_snapshot[static_cast<std::size_t>(k)] = z;
        if (z > best_z) {
            best_z = z;
            best_snapshot = k;
        }
    }

    if (best_snapshot < 0 || best_z <= 1.0e-12f) {
        out.insufficient_signal = true;
        out.message = "Adjoint solve completed, but no usable posterior mass was formed.";
        return out;
    }

    out.nx = nx;
    out.ny = ny;
    out.p_star.assign(n, 0.0f);

    // Build time-integrated posterior over space from per-snapshot normalized PDFs.
    float total_time_weight = 0.0f;
    for (int k = 0; k < static_cast<int>(sol.snapshots.size()); ++k) {
        const float z = z_by_snapshot[static_cast<std::size_t>(k)];
        if (z <= 1.0e-12f) {
            continue;
        }

        float w = 0.0f;
        if (sol.snapshots.size() == 1) {
            w = 1.0f;
        } else if (k == 0) {
            w = 0.5f
                * std::abs(
                    sol.snapshots[static_cast<std::size_t>(1)].time_s
                    - sol.snapshots[static_cast<std::size_t>(0)].time_s);
        } else if (k == static_cast<int>(sol.snapshots.size()) - 1) {
            w = 0.5f
                * std::abs(
                    sol.snapshots[static_cast<std::size_t>(k)].time_s
                    - sol.snapshots[static_cast<std::size_t>(k - 1)].time_s);
        } else {
            w = 0.5f
                * std::abs(
                    sol.snapshots[static_cast<std::size_t>(k + 1)].time_s
                    - sol.snapshots[static_cast<std::size_t>(k - 1)].time_s);
        }
        if (w <= 0.0f) {
            continue;
        }

        const auto& phi = sol.snapshots[static_cast<std::size_t>(k)].phi;
        for (std::size_t i = 0; i < n; ++i) {
            out.p_star[i] += (phi[i] / z) * w;
        }
        total_time_weight += w;
    }

    if (total_time_weight <= 1.0e-12f) {
        out.insufficient_signal = true;
        out.message = "Adjoint solve completed, but no usable temporal posterior was formed.";
        return out;
    }
    for (float& v : out.p_star) {
        v /= total_time_weight;
    }
    float z_space = 0.0f;
    for (float v : out.p_star) {
        z_space += v * cell_area;
    }
    if (z_space <= 1.0e-12f) {
        out.insufficient_signal = true;
        out.message = "Adjoint solve completed, but posterior normalization failed.";
        return out;
    }
    for (float& v : out.p_star) {
        v /= z_space;
    }

    int best_i = 0;
    int best_j = 0;
    float best_p = -std::numeric_limits<float>::infinity();
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const float p = out.p_star[static_cast<std::size_t>(idx(i, j, nx))];
            if (p > best_p) {
                best_p = p;
                best_i = i;
                best_j = j;
            }
        }
    }

    out.x_star = Vec2(domain.x_min + static_cast<float>(best_i) * dx, domain.y_min + static_cast<float>(best_j) * dy);
    out.t_star_s = sol.snapshots[static_cast<std::size_t>(best_snapshot)].time_s;
    out.success = true;
    std::ostringstream msg;
    msg << std::fixed << std::setprecision(1) << "x*=(" << out.x_star.x() << ", " << out.x_star.y() << ")"
        << ", t*=" << out.t_star_s << " s"
        << ", age=" << (t_end - out.t_star_s) << " s";
    out.message = msg.str();
    return out;
}

} // namespace atm
