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
    // Sum of gated (above-threshold) noisy readings; this is the linear
    // response functional R_i[c] whose sensitivity to a unit source at
    // (x,t) is exactly what build_candidate_field's per-sensor adjoint
    // solve computes (same threshold-gated forcing on both sides).
    float gated_sum = 0.0f;
    int gated_count = 0;
};

struct SeriesBuildResult {
    std::vector<SensorSeries> series;
    float t_start = 0.0f;
    bool has_gated_signal = false;
};

struct CandidateField {
    int nx = 0;
    int ny = 0;
    int nt = 0;
    float x_min = 0.0f, x_max = 0.0f, y_min = 0.0f, y_max = 0.0f;
    std::vector<float> times;
    // data[sensor][time_index][j*nx+i]
    std::vector<std::vector<std::vector<float>>> data;
};

struct CellFit {
    float q0 = 0.0f;
    float q0_variance = 0.0f;
    float weighted_rss = 0.0f;
    int dof = 0;
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

float bilinear_sample(
    const std::vector<float>& field, int nx, int ny, float x_min, float x_max, float y_min, float y_max, float x,
    float y) {
    if (nx < 2 || ny < 2) {
        return field.empty() ? 0.0f : field[0];
    }
    const float gx = std::clamp((x - x_min) / std::max(1.0e-6f, x_max - x_min), 0.0f, 1.0f)
        * static_cast<float>(nx - 1);
    const float gy = std::clamp((y - y_min) / std::max(1.0e-6f, y_max - y_min), 0.0f, 1.0f)
        * static_cast<float>(ny - 1);
    const int i0 = static_cast<int>(std::floor(gx));
    const int j0 = static_cast<int>(std::floor(gy));
    const int i1 = std::min(i0 + 1, nx - 1);
    const int j1 = std::min(j0 + 1, ny - 1);
    const float tx = gx - static_cast<float>(i0);
    const float ty = gy - static_cast<float>(j0);

    auto at = [&](int i, int j) { return field[static_cast<std::size_t>(idx(i, j, nx))]; };
    const float c0 = (1.0f - tx) * at(i0, j0) + tx * at(i1, j0);
    const float c1 = (1.0f - tx) * at(i0, j1) + tx * at(i1, j1);
    return (1.0f - ty) * c0 + ty * c1;
}

// Sensor i's predicted response to a unit-amplitude source at (x,y,t),
// trilinearly interpolated from the precomputed candidate field.
float sample_candidate(const CandidateField& cf, std::size_t sensor_idx, float x, float y, float t) {
    if (cf.nt < 2) {
        return bilinear_sample(cf.data[sensor_idx][0], cf.nx, cf.ny, cf.x_min, cf.x_max, cf.y_min, cf.y_max, x, y);
    }
    int k0 = 0;
    while (k0 < cf.nt - 2 && cf.times[static_cast<std::size_t>(k0 + 1)] < t) {
        ++k0;
    }
    const int k1 = std::min(k0 + 1, cf.nt - 1);
    float tt = 0.0f;
    const float span = cf.times[static_cast<std::size_t>(k1)] - cf.times[static_cast<std::size_t>(k0)];
    if (span > 1.0e-9f) {
        tt = std::clamp((t - cf.times[static_cast<std::size_t>(k0)]) / span, 0.0f, 1.0f);
    }
    const float v0
        = bilinear_sample(cf.data[sensor_idx][static_cast<std::size_t>(k0)], cf.nx, cf.ny, cf.x_min, cf.x_max,
            cf.y_min, cf.y_max, x, y);
    const float v1
        = bilinear_sample(cf.data[sensor_idx][static_cast<std::size_t>(k1)], cf.nx, cf.ny, cf.x_min, cf.x_max,
            cf.y_min, cf.y_max, x, y);
    return (1.0f - tt) * v0 + tt * v1;
}

// Weighted ridge-regularized 1-D fit of source amplitude q0 at a fixed
// candidate (x,y,t), given each sensor's predicted unit-source response
// (from the candidate field) and its observed gated response. The ridge
// term doubles as a Gaussian-prior precision on q0 (see estimate_regularized_least_squares).
CellFit fit_cell(
    const CandidateField& cf, const std::vector<SensorSeries>& series, float x, float y, float t,
    float relative_model_error, float ridge) {
    double num = 0.0;
    double den = static_cast<double>(std::max(0.0f, ridge));
    std::vector<float> h(series.size(), 0.0f);

    for (std::size_t i = 0; i < series.size(); ++i) {
        if (series[i].gated_count <= 0) {
            continue;
        }
        const float hi = sample_candidate(cf, i, x, y, t);
        h[i] = hi;
        const float yi = series[i].gated_sum;
        const float sigma_i = std::max(1.0e-9f, relative_model_error * std::max(std::abs(yi), 1.0e-9f));
        const double wi = 1.0 / (static_cast<double>(sigma_i) * sigma_i);
        num += wi * hi * yi;
        den += wi * static_cast<double>(hi) * hi;
    }

    CellFit out;
    out.q0 = den > 1.0e-12 ? std::max(0.0f, static_cast<float>(num / den)) : 0.0f;
    out.q0_variance = den > 1.0e-12 ? static_cast<float>(1.0 / den) : std::numeric_limits<float>::infinity();

    double rss = 0.0;
    int dof = 0;
    for (std::size_t i = 0; i < series.size(); ++i) {
        if (series[i].gated_count <= 0) {
            continue;
        }
        const float yi = series[i].gated_sum;
        const float sigma_i = std::max(1.0e-9f, relative_model_error * std::max(std::abs(yi), 1.0e-9f));
        const float resid = yi - out.q0 * h[i];
        rss += (static_cast<double>(resid) * resid) / (static_cast<double>(sigma_i) * sigma_i);
        ++dof;
    }
    out.weighted_rss = static_cast<float>(rss);
    out.dof = dof;
    return out;
}

} // namespace

SourceEstimator::SourceEstimator(SourceEstimationConfig cfg)
    : cfg_(cfg)
    , base_detection_threshold_(cfg.detection_threshold) {
}

const SourceEstimationConfig& SourceEstimator::config() const {
    return cfg_;
}

void SourceEstimator::set_noise_floor(float floor) {
    cfg_.detection_threshold = std::max(base_detection_threshold_, floor);
}

std::string_view source_estimation_method_name(SourceEstimationMethod method) {
    switch (method) {
    case SourceEstimationMethod::AdjointBacktracking:
        return "Adjoint Backtracking";
    case SourceEstimationMethod::RegularizedLeastSquares:
        return "Regularized Least Squares";
    case SourceEstimationMethod::BayesianGrid:
        return "Bayesian Grid";
    }
    return "Adjoint Backtracking";
}

namespace {

SeriesBuildResult build_series(
    const std::vector<SensorManager::Sensor>& sensors, const Simulator& sim, const SourceEstimationConfig& cfg) {
    SeriesBuildResult out;

    const float t_end = sim.time_s();
    const float t_lower = std::max(0.0f, t_end - std::max(1.0f, cfg.max_lookback_s));
    const auto& domain = sim.config().domain;

    const int nx = std::max(2, cfg.adjoint_grid_nx);
    const int ny = std::max(2, cfg.adjoint_grid_ny);
    const float dx = (domain.x_max - domain.x_min) / static_cast<float>(nx - 1);
    const float dy = (domain.y_max - domain.y_min) / static_cast<float>(ny - 1);
    const float cell_area = dx * dy;
    const float sigma = std::max(1.0f, cfg.gaussian_sigma);
    const float inv_2s2 = 1.0f / (2.0f * sigma * sigma);
    const std::size_t n = static_cast<std::size_t>(nx * ny);

    float earliest_obs = t_end;

    for (const auto& sensor : sensors) {
        if (sensor.history.empty()) {
            continue;
        }

        SensorSeries s;
        s.position = sensor.position;
        const int keep = std::max(1, cfg.max_samples_per_sensor);
        const int begin = std::max(0, static_cast<int>(sensor.history.size()) - keep);
        for (int i = begin; i < static_cast<int>(sensor.history.size()); ++i) {
            const auto& obs = sensor.history[static_cast<std::size_t>(i)];
            if (obs.time_s < t_lower || obs.time_s > t_end) {
                continue;
            }
            s.obs.push_back(obs);
            earliest_obs = std::min(earliest_obs, obs.time_s);
            if (obs.noisy_concentration > cfg.detection_threshold) {
                out.has_gated_signal = true;
                s.gated_sum += obs.noisy_concentration;
                ++s.gated_count;
            }
        }
        if (s.obs.empty()) {
            continue;
        }

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

        out.series.push_back(std::move(s));
    }

    // The adjoint (and hence the candidate release times) starts before the
    // first observation: a release precedes its first detection.
    out.t_start = std::max(0.0f, earliest_obs - std::max(0.0f, cfg.release_search_margin_s));
    return out;
}

// Shared preconditions for every method; returns false (with `out` filled
// in) when there is nothing to invert.
bool check_signal(const SeriesBuildResult& build, SourceEstimateResult& out) {
    if (build.series.empty()) {
        out.insufficient_signal = true;
        out.message = "No sensor history available in lookback window.";
        return false;
    }
    if (!build.has_gated_signal) {
        out.insufficient_signal = true;
        out.message = "Insufficient signal: no report in the lookback window exceeds the detection threshold.";
        return false;
    }
    return true;
}

// Runs one backward adjoint solve per sensor (unit-weighted, threshold-gated
// forcing at that sensor's own observation times) and projects the result
// onto a coarser (search_grid_nx x search_grid_ny x search_time_count)
// candidate grid via bilinear/linear interpolation. By adjoint reciprocity,
// sensor i's projected field at (x,y,t) is its predicted response to a
// unit-amplitude source released at (x,y,t).
CandidateField build_candidate_field(
    const std::vector<SensorSeries>& series, const Simulator& sim, const SourceEstimationConfig& cfg,
    const AdjointSolver& solver, float t_start, float t_end, int search_nx, int search_ny, int search_nt) {
    CandidateField cf;
    cf.nx = std::max(2, search_nx);
    cf.ny = std::max(2, search_ny);
    cf.nt = std::max(2, search_nt);
    const auto& domain = sim.config().domain;
    cf.x_min = domain.x_min;
    cf.x_max = domain.x_max;
    cf.y_min = domain.y_min;
    cf.y_max = domain.y_max;

    cf.times.resize(static_cast<std::size_t>(cf.nt));
    for (int k = 0; k < cf.nt; ++k) {
        const float frac = cf.nt > 1 ? static_cast<float>(k) / static_cast<float>(cf.nt - 1) : 0.0f;
        cf.times[static_cast<std::size_t>(k)] = t_start + frac * (t_end - t_start);
    }

    AdjointSolver::Config solve_cfg;
    solve_cfg.domain = domain;
    solve_cfg.nx = std::max(2, cfg.adjoint_grid_nx);
    solve_cfg.ny = std::max(2, cfg.adjoint_grid_ny);
    solve_cfg.dt_s = std::max(0.05f, cfg.adjoint_dt_s);
    solve_cfg.deposition_rate = sim.config().physics.deposition_rate;
    solve_cfg.diffusion_mode = sim.pde_diffusion_mode() == AdvectionDiffusionSolver::DiffusionMode::FullTensorFlux
        ? AdjointSolver::DiffusionMode::FullTensorFlux
        : AdjointSolver::DiffusionMode::ScalarizedTrace;

    auto wind_fn = [&](float time_s, const Vec2& p) { return sim.wind_at_time(time_s, p); };
    auto diffusivity_fn = [&](float time_s, const Vec2& p) { return sim.diffusivity_at_time(time_s, p); };

    cf.data.assign(series.size(), {});
    for (std::size_t si = 0; si < series.size(); ++si) {
        const auto& s = series[si];
        auto forcing_fn = [&](float time_s, std::vector<float>& f) {
            f.assign(static_cast<std::size_t>(solve_cfg.nx * solve_cfg.ny), 0.0f);
            const int obs_idx = last_obs_at_or_before(s.obs, time_s);
            if (obs_idx < 0) {
                return;
            }
            if (s.obs[static_cast<std::size_t>(obs_idx)].noisy_concentration <= cfg.detection_threshold) {
                return;
            }
            for (std::size_t k = 0; k < f.size(); ++k) {
                f[k] += s.bump[k];
            }
        };

        // `sol` holds one full-resolution grid per adjoint step; scoping it to
        // this iteration bounds peak memory regardless of sensor count.
        const auto sol = solver.solve_backward(t_start, t_end, solve_cfg, wind_fn, diffusivity_fn, forcing_fn);

        std::vector<std::vector<float>> per_time(static_cast<std::size_t>(cf.nt));
        for (int k = 0; k < cf.nt; ++k) {
            std::size_t best = 0;
            float best_dt = std::numeric_limits<float>::infinity();
            for (std::size_t sj = 0; sj < sol.snapshots.size(); ++sj) {
                const float d = std::abs(sol.snapshots[sj].time_s - cf.times[static_cast<std::size_t>(k)]);
                if (d < best_dt) {
                    best_dt = d;
                    best = sj;
                }
            }
            const auto& phi = sol.snapshots[best].phi;

            std::vector<float> projected(static_cast<std::size_t>(cf.nx * cf.ny));
            for (int j = 0; j < cf.ny; ++j) {
                const float yfrac = cf.ny > 1 ? static_cast<float>(j) / static_cast<float>(cf.ny - 1) : 0.0f;
                const float y = cf.y_min + yfrac * (cf.y_max - cf.y_min);
                for (int i = 0; i < cf.nx; ++i) {
                    const float xfrac = cf.nx > 1 ? static_cast<float>(i) / static_cast<float>(cf.nx - 1) : 0.0f;
                    const float x = cf.x_min + xfrac * (cf.x_max - cf.x_min);
                    projected[static_cast<std::size_t>(idx(i, j, cf.nx))]
                        = bilinear_sample(phi, sol.nx, sol.ny, cf.x_min, cf.x_max, cf.y_min, cf.y_max, x, y);
                }
            }
            per_time[static_cast<std::size_t>(k)] = std::move(projected);
        }
        cf.data[si] = std::move(per_time);
    }

    return cf;
}

SourceEstimateResult regularized_least_squares_from(
    const SeriesBuildResult& build, const CandidateField& cf, const Simulator& sim, const SourceEstimationConfig& cfg);
SourceEstimateResult bayesian_grid_from(
    const SeriesBuildResult& build, const CandidateField& cf, const Simulator& sim, const SourceEstimationConfig& cfg);

} // namespace

SourceEstimateResult SourceEstimator::estimate(
    const std::vector<SensorManager::Sensor>& sensors, const Simulator& sim) const {
    return estimate(sensors, sim, SourceEstimationMethod::AdjointBacktracking);
}

SourceEstimateResult SourceEstimator::estimate(
    const std::vector<SensorManager::Sensor>& sensors, const Simulator& sim, SourceEstimationMethod method) const {
    switch (method) {
    case SourceEstimationMethod::AdjointBacktracking:
        return estimate_adjoint_backtracking(sensors, sim);
    case SourceEstimationMethod::RegularizedLeastSquares:
        return estimate_regularized_least_squares(sensors, sim);
    case SourceEstimationMethod::BayesianGrid:
        return estimate_bayesian_grid(sensors, sim);
    }
    return estimate_adjoint_backtracking(sensors, sim);
}

std::vector<SourceEstimateResult> SourceEstimator::estimate_all(
    const std::vector<SensorManager::Sensor>& sensors, const Simulator& sim) const {
    std::vector<SourceEstimateResult> results{
        estimate_adjoint_backtracking(sensors, sim),
    };

    // RegularizedLeastSquares and BayesianGrid invert the same per-sensor
    // sensitivity fields; build them once (the dominant cost) for both.
    const auto build = build_series(sensors, sim, cfg_);
    SourceEstimateResult rls;
    rls.method = SourceEstimationMethod::RegularizedLeastSquares;
    SourceEstimateResult bayes;
    bayes.method = SourceEstimationMethod::BayesianGrid;
    const bool rls_ok = check_signal(build, rls);
    const bool bayes_ok = check_signal(build, bayes);
    if (rls_ok && bayes_ok) {
        const auto cf = build_candidate_field(build.series, sim, cfg_, solver_, build.t_start, sim.time_s(),
            cfg_.search_grid_nx, cfg_.search_grid_ny, cfg_.search_time_count);
        rls = regularized_least_squares_from(build, cf, sim, cfg_);
        bayes = bayesian_grid_from(build, cf, sim, cfg_);
    }
    results.push_back(std::move(rls));
    results.push_back(std::move(bayes));
    return results;
}

float SourceEstimator::observation_window_start(const Simulator& sim) const {
    return std::max(0.0f, sim.time_s() - std::max(1.0f, cfg_.max_lookback_s));
}

SourceEstimateResult SourceEstimator::estimate_adjoint_backtracking(
    const std::vector<SensorManager::Sensor>& sensors, const Simulator& sim) const {
    SourceEstimateResult out;
    out.method = SourceEstimationMethod::AdjointBacktracking;

    const auto build = build_series(sensors, sim, cfg_);
    if (!check_signal(build, out)) {
        return out;
    }

    const auto& domain = sim.config().domain;
    const int nx = std::max(2, cfg_.adjoint_grid_nx);
    const int ny = std::max(2, cfg_.adjoint_grid_ny);
    const float dx = (domain.x_max - domain.x_min) / static_cast<float>(nx - 1);
    const float dy = (domain.y_max - domain.y_min) / static_cast<float>(ny - 1);
    const float cell_area = dx * dy;
    const std::size_t n = static_cast<std::size_t>(nx * ny);
    const float t_end = sim.time_s();

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
        for (const auto& s : build.series) {
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

    const auto sol = solver_.solve_backward(build.t_start, t_end, solve_cfg, wind_fn, diffusivity_fn, forcing_fn);

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
    out.sensors_used = static_cast<int>(build.series.size());
    for (const auto& s : build.series) {
        out.observations_used += s.gated_count;
    }
    out.success = true;
    std::ostringstream msg;
    msg << std::fixed << std::setprecision(1) << "x*=(" << out.x_star.x() << ", " << out.x_star.y() << ")"
        << ", t*=" << out.t_star_s << " s"
        << ", age=" << (t_end - out.t_star_s) << " s";
    out.message = msg.str();
    return out;
}

SourceEstimateResult SourceEstimator::estimate_regularized_least_squares(
    const std::vector<SensorManager::Sensor>& sensors, const Simulator& sim) const {
    SourceEstimateResult out;
    out.method = SourceEstimationMethod::RegularizedLeastSquares;
    const auto build = build_series(sensors, sim, cfg_);
    if (!check_signal(build, out)) {
        return out;
    }
    const auto cf = build_candidate_field(build.series, sim, cfg_, solver_, build.t_start, sim.time_s(),
        cfg_.search_grid_nx, cfg_.search_grid_ny, cfg_.search_time_count);
    return regularized_least_squares_from(build, cf, sim, cfg_);
}

SourceEstimateResult SourceEstimator::estimate_bayesian_grid(
    const std::vector<SensorManager::Sensor>& sensors, const Simulator& sim) const {
    SourceEstimateResult out;
    out.method = SourceEstimationMethod::BayesianGrid;
    const auto build = build_series(sensors, sim, cfg_);
    if (!check_signal(build, out)) {
        return out;
    }
    const auto cf = build_candidate_field(build.series, sim, cfg_, solver_, build.t_start, sim.time_s(),
        cfg_.search_grid_nx, cfg_.search_grid_ny, cfg_.search_time_count);
    return bayesian_grid_from(build, cf, sim, cfg_);
}

namespace {

SourceEstimateResult regularized_least_squares_from(
    const SeriesBuildResult& build, const CandidateField& cf, const Simulator& sim, const SourceEstimationConfig& cfg) {
    SourceEstimateResult out;
    out.method = SourceEstimationMethod::RegularizedLeastSquares;

    const float t_end = sim.time_s();
    const int nt = cf.nt;
    const int nx0 = cf.nx;
    const int ny0 = cf.ny;
    const auto& domain = sim.config().domain;

    struct Cell {
        float x = 0.0f, y = 0.0f, t = 0.0f;
        float rss = std::numeric_limits<float>::infinity();
        float q0 = 0.0f;
        float q0_variance = 0.0f;
    };

    auto eval = [&](float x, float y, float t) {
        const auto fit = fit_cell(cf, build.series, x, y, t, cfg.relative_model_error, cfg.amplitude_ridge);
        return Cell{x, y, t, fit.weighted_rss, fit.q0, fit.q0_variance};
    };

    std::vector<Cell> cells;
    cells.reserve(static_cast<std::size_t>(nx0 * ny0 * nt));
    for (int k = 0; k < nt; ++k) {
        const float t = cf.times[static_cast<std::size_t>(k)];
        for (int j = 0; j < ny0; ++j) {
            const float yfrac = ny0 > 1 ? static_cast<float>(j) / static_cast<float>(ny0 - 1) : 0.0f;
            const float y = domain.y_min + yfrac * (domain.y_max - domain.y_min);
            for (int i = 0; i < nx0; ++i) {
                const float xfrac = nx0 > 1 ? static_cast<float>(i) / static_cast<float>(nx0 - 1) : 0.0f;
                const float x = domain.x_min + xfrac * (domain.x_max - domain.x_min);
                cells.push_back(eval(x, y, t));
            }
        }
    }

    // Posterior heatmap: profile likelihood over (x,y) with the release time
    // profiled out, exp(-0.5 * min_t weighted_RSS) under a flat prior,
    // normalized to a density on the coarse search grid (same convention
    // as the other methods' p_star).
    {
        const std::size_t ncell = static_cast<std::size_t>(nx0 * ny0);
        std::vector<float> min_rss(ncell, std::numeric_limits<float>::infinity());
        for (std::size_t c = 0; c < cells.size(); ++c) {
            min_rss[c % ncell] = std::min(min_rss[c % ncell], cells[c].rss);
        }
        const float global_min = *std::min_element(min_rss.begin(), min_rss.end());
        const float dxg = (domain.x_max - domain.x_min) / static_cast<float>(nx0 - 1);
        const float dyg = (domain.y_max - domain.y_min) / static_cast<float>(ny0 - 1);
        out.nx = nx0;
        out.ny = ny0;
        out.p_star.assign(ncell, 0.0f);
        float z = 0.0f;
        for (std::size_t c = 0; c < ncell; ++c) {
            if (std::isfinite(min_rss[c])) {
                out.p_star[c] = std::exp(-0.5f * (min_rss[c] - global_min));
                z += out.p_star[c] * dxg * dyg;
            }
        }
        if (z > 0.0f) {
            for (float& v : out.p_star) {
                v /= z;
            }
        }
    }

    std::sort(cells.begin(), cells.end(), [](const Cell& a, const Cell& b) { return a.rss < b.rss; });
    const int keep = std::clamp(cfg.candidate_count, 1, static_cast<int>(cells.size()));
    std::vector<Cell> top(cells.begin(), cells.begin() + keep);

    float window_x = (domain.x_max - domain.x_min) / static_cast<float>(std::max(1, nx0 - 1));
    float window_y = (domain.y_max - domain.y_min) / static_cast<float>(std::max(1, ny0 - 1));
    float window_t = nt > 1 ? (cf.times.back() - cf.times.front()) / static_cast<float>(nt - 1) : 1.0f;

    for (int level = 0; level < std::max(0, cfg.refinement_levels); ++level) {
        for (auto& cell : top) {
            Cell best = cell;
            constexpr int kSub = 4;
            for (int tk = -kSub; tk <= kSub; ++tk) {
                const float t = std::clamp(
                    cell.t + (window_t * static_cast<float>(tk)) / static_cast<float>(kSub), cf.times.front(),
                    cf.times.back());
                for (int jk = -kSub; jk <= kSub; ++jk) {
                    const float y = std::clamp(
                        cell.y + (window_y * static_cast<float>(jk)) / static_cast<float>(kSub), domain.y_min,
                        domain.y_max);
                    for (int ik = -kSub; ik <= kSub; ++ik) {
                        const float x = std::clamp(
                            cell.x + (window_x * static_cast<float>(ik)) / static_cast<float>(kSub), domain.x_min,
                            domain.x_max);
                        const Cell candidate = eval(x, y, t);
                        if (candidate.rss < best.rss) {
                            best = candidate;
                        }
                    }
                }
            }
            cell = best;
        }
        window_x *= 0.4f;
        window_y *= 0.4f;
        window_t *= 0.4f;
    }

    std::sort(top.begin(), top.end(), [](const Cell& a, const Cell& b) { return a.rss < b.rss; });
    const Cell& best = top.front();

    // Profile-likelihood curvature (finite differences of weighted RSS) for
    // positional uncertainty; q0's variance is exact from the linear fit.
    auto curvature = [&](float delta, float x0, float y0, float t0, int axis) -> float {
        Cell plus = axis == 0 ? eval(x0 + delta, y0, t0) : (axis == 1 ? eval(x0, y0 + delta, t0) : eval(x0, y0, t0 + delta));
        Cell minus = axis == 0 ? eval(x0 - delta, y0, t0) : (axis == 1 ? eval(x0, y0 - delta, t0) : eval(x0, y0, t0 - delta));
        return (plus.rss - 2.0f * best.rss + minus.rss) / (delta * delta);
    };

    const float hx = std::max(window_x, 1.0f);
    const float hy = std::max(window_y, 1.0f);
    const float ht = std::max(window_t, 1.0f);
    const float hxx = curvature(hx, best.x, best.y, best.t, 0);
    const float hyy = curvature(hy, best.x, best.y, best.t, 1);
    const float htt = curvature(ht, best.x, best.y, best.t, 2);

    out.x_std_m = hxx > 1.0e-9f ? std::sqrt(2.0f / hxx) : (domain.x_max - domain.x_min);
    out.y_std_m = hyy > 1.0e-9f ? std::sqrt(2.0f / hyy) : (domain.y_max - domain.y_min);
    out.t_std_s = htt > 1.0e-9f ? std::sqrt(2.0f / htt) : (t_end - build.t_start);
    out.weakly_identified = (hxx <= 1.0e-9f || hyy <= 1.0e-9f || htt <= 1.0e-9f);

    out.x_star = Vec2(best.x, best.y);
    out.t_star_s = best.t;
    out.q0 = best.q0;
    out.q0_std = std::sqrt(std::max(0.0f, best.q0_variance));
    out.total_released_mass = best.q0;
    out.success = true;
    out.sensors_used = static_cast<int>(build.series.size());

    int dof = 0;
    for (const auto& s : build.series) {
        if (s.gated_count > 0) {
            out.observations_used += s.gated_count;
            ++dof;
        }
        out.observation_fit.push_back(SourceEstimationObservation{});
        auto& fit = out.observation_fit.back();
        fit.sensor_index = static_cast<int>(&s - &build.series[0]);
        fit.sensor_position = s.position;
        fit.time_s = best.t;
        fit.measured_concentration = s.gated_sum;
        fit.predicted_concentration
            = s.gated_count > 0 ? best.q0 * sample_candidate(cf, static_cast<std::size_t>(fit.sensor_index), best.x, best.y, best.t) : 0.0f;
        fit.residual = fit.measured_concentration - fit.predicted_concentration;
        fit.assumed_std = std::max(1.0e-9f, cfg.relative_model_error * std::max(std::abs(fit.measured_concentration), 1.0e-9f));
    }
    out.weighted_rmse = dof > 0 ? std::sqrt(best.rss / static_cast<float>(dof)) : 0.0f;

    std::ostringstream msg;
    msg << std::fixed << std::setprecision(1) << "x*=(" << out.x_star.x() << ", " << out.x_star.y() << ")"
        << ", t*=" << out.t_star_s << " s, q0=" << std::setprecision(3) << out.q0
        << " +/- " << out.q0_std << ", wrmse=" << out.weighted_rmse;
    out.message = msg.str();
    return out;
}

SourceEstimateResult bayesian_grid_from(
    const SeriesBuildResult& build, const CandidateField& cf, const Simulator& sim, const SourceEstimationConfig& cfg) {
    SourceEstimateResult out;
    out.method = SourceEstimationMethod::BayesianGrid;

    const float t_end = sim.time_s();
    const int nt = cf.nt;
    const int nx = cf.nx;
    const int ny = cf.ny;
    const auto& domain = sim.config().domain;
    const float dxg = (domain.x_max - domain.x_min) / static_cast<float>(std::max(1, nx - 1));
    const float dyg = (domain.y_max - domain.y_min) / static_cast<float>(std::max(1, ny - 1));
    const float cell_area = dxg * dyg;
    const std::size_t ncell = static_cast<std::size_t>(nx * ny);

    // Each cell centers its q0 quadrature on its own ridge estimate, since a
    // tight relative_model_error makes the likelihood too sharp for one global
    // range. q0_scan_max only bounds cells with no sensitivity to any sensor
    // (infinite analytic variance).
    float q0_scan_max = 0.0f;
    for (int k = 0; k < nt; ++k) {
        const float t = cf.times[static_cast<std::size_t>(k)];
        for (int j = 0; j < ny; ++j) {
            const float yfrac = ny > 1 ? static_cast<float>(j) / static_cast<float>(ny - 1) : 0.0f;
            const float y = domain.y_min + yfrac * (domain.y_max - domain.y_min);
            for (int i = 0; i < nx; ++i) {
                const float xfrac = nx > 1 ? static_cast<float>(i) / static_cast<float>(nx - 1) : 0.0f;
                const float x = domain.x_min + xfrac * (domain.x_max - domain.x_min);
                const auto fit = fit_cell(cf, build.series, x, y, t, cfg.relative_model_error, cfg.amplitude_ridge);
                q0_scan_max = std::max(q0_scan_max, fit.q0);
            }
        }
    }
    const float q0_fallback_max = std::max(1.0e-9f, 3.0f * q0_scan_max);
    const int nq = std::max(3, cfg.response_quadrature_points);

    // Evidence per (time, cell): trapezoidal quadrature over q0 >= 0, which
    // also gives each cell's posterior mean and second moment of q0. The raw
    // likelihood underflows even a double, so each cell's sums use a local
    // log-sum-exp shift (it cancels in the moment ratios), and log-evidence
    // is exponentiated only relative to the best cell over the whole grid.
    std::vector<std::vector<float>> log_evidence(static_cast<std::size_t>(nt), std::vector<float>(ncell, -std::numeric_limits<float>::infinity()));
    std::vector<std::vector<float>> q0_mean(static_cast<std::size_t>(nt), std::vector<float>(ncell, 0.0f));
    std::vector<std::vector<float>> q0_m2(static_cast<std::size_t>(nt), std::vector<float>(ncell, 0.0f));
    double global_log_max = -std::numeric_limits<double>::infinity();

    std::vector<double> neg_half_chi2_scratch(static_cast<std::size_t>(nq));
    for (int k = 0; k < nt; ++k) {
        const float t = cf.times[static_cast<std::size_t>(k)];
        for (int j = 0; j < ny; ++j) {
            const float yfrac = ny > 1 ? static_cast<float>(j) / static_cast<float>(ny - 1) : 0.0f;
            const float y = domain.y_min + yfrac * (domain.y_max - domain.y_min);
            for (int i = 0; i < nx; ++i) {
                const float xfrac = nx > 1 ? static_cast<float>(i) / static_cast<float>(nx - 1) : 0.0f;
                const float x = domain.x_min + xfrac * (domain.x_max - domain.x_min);
                const std::size_t cell = static_cast<std::size_t>(idx(i, j, nx));

                const auto fit = fit_cell(cf, build.series, x, y, t, cfg.relative_model_error, cfg.amplitude_ridge);
                const float raw_std = std::isfinite(fit.q0_variance) ? std::sqrt(std::max(0.0f, fit.q0_variance)) : (q0_fallback_max / 6.0f);
                const float std_q0 = std::max(raw_std, 1.0e-6f * std::max(1.0f, q0_fallback_max));
                constexpr float kSpanStd = 6.0f;
                const float q0_lo = std::max(0.0f, fit.q0 - kSpanStd * std_q0);
                const float q0_hi = fit.q0 + kSpanStd * std_q0;
                const float dq = (q0_hi - q0_lo) / static_cast<float>(nq - 1);

                double local_max = -std::numeric_limits<double>::infinity();
                for (int qi = 0; qi < nq; ++qi) {
                    const float q0 = q0_lo + static_cast<float>(qi) * dq;
                    double neg_half_chi2 = 0.0;
                    for (const auto& s : build.series) {
                        if (s.gated_count <= 0) {
                            continue;
                        }
                        const float hi = sample_candidate(cf, static_cast<std::size_t>(&s - &build.series[0]), x, y, t);
                        const float yi = s.gated_sum;
                        const float sigma_i = std::max(1.0e-9f, cfg.relative_model_error * std::max(std::abs(yi), 1.0e-9f));
                        const float resid = yi - q0 * hi;
                        neg_half_chi2 -= 0.5 * (static_cast<double>(resid) * resid) / (static_cast<double>(sigma_i) * sigma_i);
                    }
                    neg_half_chi2_scratch[static_cast<std::size_t>(qi)] = neg_half_chi2;
                    local_max = std::max(local_max, neg_half_chi2);
                }

                // Local log-sum-exp: sum_local = sum_qi w_qi*exp(chi2_qi - local_max)*dq,
                // so log(z_cell) = local_max + log(sum_local). q0_mean/q0_m2 use the
                // same shift, which cancels in the mean_num/sum_local and m2_num/sum_local ratios.
                double sum_local = 0.0, mean_num = 0.0, m2_num = 0.0;
                for (int qi = 0; qi < nq; ++qi) {
                    const float q0 = q0_lo + static_cast<float>(qi) * dq;
                    const double rel_likelihood = std::exp(neg_half_chi2_scratch[static_cast<std::size_t>(qi)] - local_max);
                    const double w = (qi == 0 || qi == nq - 1) ? 0.5 : 1.0;
                    sum_local += w * rel_likelihood;
                    mean_num += w * rel_likelihood * q0;
                    m2_num += w * rel_likelihood * static_cast<double>(q0) * q0;
                }

                const double log_z = (sum_local > 0.0) ? (local_max + std::log(sum_local * dq)) : -std::numeric_limits<double>::infinity();
                log_evidence[static_cast<std::size_t>(k)][cell] = static_cast<float>(log_z);
                global_log_max = std::max(global_log_max, log_z);
                q0_mean[static_cast<std::size_t>(k)][cell] = sum_local > 0.0 ? static_cast<float>(mean_num / sum_local) : 0.0f;
                q0_m2[static_cast<std::size_t>(k)][cell] = sum_local > 0.0 ? static_cast<float>(m2_num / sum_local) : 0.0f;
            }
        }
    }

    // Evidence relative to the best (time, cell); the constant factor
    // exp(global_log_max) cancels in every normalization below.
    std::vector<std::vector<float>> evidence(static_cast<std::size_t>(nt), std::vector<float>(ncell, 0.0f));
    for (int k = 0; k < nt; ++k) {
        for (std::size_t c = 0; c < ncell; ++c) {
            const float lz = log_evidence[static_cast<std::size_t>(k)][c];
            evidence[static_cast<std::size_t>(k)][c]
                = std::isfinite(lz) ? static_cast<float>(std::exp(static_cast<double>(lz) - global_log_max)) : 0.0f;
        }
    }

    // Time-marginal evidence picks t*; time-integrated (trapezoidal),
    // spatially-normalized evidence becomes the posterior p_star, matching
    // the AdjointBacktracking result's field layout/normalization convention.
    std::vector<float> z_by_time(static_cast<std::size_t>(nt), 0.0f);
    for (int k = 0; k < nt; ++k) {
        float z = 0.0f;
        for (float v : evidence[static_cast<std::size_t>(k)]) {
            z += v * cell_area;
        }
        z_by_time[static_cast<std::size_t>(k)] = z;
    }

    int best_k = 0;
    float best_z = -std::numeric_limits<float>::infinity();
    for (int k = 0; k < nt; ++k) {
        if (z_by_time[static_cast<std::size_t>(k)] > best_z) {
            best_z = z_by_time[static_cast<std::size_t>(k)];
            best_k = k;
        }
    }
    if (best_z <= 1.0e-30f) {
        out.insufficient_signal = true;
        out.message = "Bayesian grid solve completed, but no usable posterior mass was formed.";
        return out;
    }

    out.nx = nx;
    out.ny = ny;
    out.p_star.assign(ncell, 0.0f);
    float total_time_weight = 0.0f;
    for (int k = 0; k < nt; ++k) {
        float w = 0.0f;
        if (nt == 1) {
            w = 1.0f;
        } else if (k == 0) {
            w = 0.5f * std::abs(cf.times[1] - cf.times[0]);
        } else if (k == nt - 1) {
            w = 0.5f
                * std::abs(
                    cf.times[static_cast<std::size_t>(k)] - cf.times[static_cast<std::size_t>(k - 1)]);
        } else {
            w = 0.5f
                * std::abs(
                    cf.times[static_cast<std::size_t>(k + 1)] - cf.times[static_cast<std::size_t>(k - 1)]);
        }
        for (std::size_t c = 0; c < ncell; ++c) {
            out.p_star[c] += evidence[static_cast<std::size_t>(k)][c] * w;
        }
        total_time_weight += w;
    }
    if (total_time_weight > 1.0e-12f) {
        for (float& v : out.p_star) {
            v /= total_time_weight;
        }
    }
    float z_space = 0.0f;
    for (float v : out.p_star) {
        z_space += v * cell_area;
    }
    if (z_space > 1.0e-30f) {
        for (float& v : out.p_star) {
            v /= z_space;
        }
    }

    int best_i = 0, best_j = 0;
    float best_p = -std::numeric_limits<float>::infinity();
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const float p = evidence[static_cast<std::size_t>(best_k)][static_cast<std::size_t>(idx(i, j, nx))];
            if (p > best_p) {
                best_p = p;
                best_i = i;
                best_j = j;
            }
        }
    }
    const std::size_t best_cell = static_cast<std::size_t>(idx(best_i, best_j, nx));

    // 3-point parabolic sub-cell refinement of the argmax along each axis,
    // since the coarse search grid puts nodes up to a cell width from the peak.
    auto parabolic_offset = [](float fm, float f0, float fp) -> float {
        const float denom = fm - 2.0f * f0 + fp;
        if (std::abs(denom) < 1.0e-20f) {
            return 0.0f;
        }
        return std::clamp(0.5f * (fm - fp) / denom, -1.0f, 1.0f);
    };
    float i_refined = static_cast<float>(best_i);
    float j_refined = static_cast<float>(best_j);
    if (best_i > 0 && best_i < nx - 1) {
        const float fm = evidence[static_cast<std::size_t>(best_k)][static_cast<std::size_t>(idx(best_i - 1, best_j, nx))];
        const float fp = evidence[static_cast<std::size_t>(best_k)][static_cast<std::size_t>(idx(best_i + 1, best_j, nx))];
        i_refined += parabolic_offset(fm, best_p, fp);
    }
    if (best_j > 0 && best_j < ny - 1) {
        const float fm = evidence[static_cast<std::size_t>(best_k)][static_cast<std::size_t>(idx(best_i, best_j - 1, nx))];
        const float fp = evidence[static_cast<std::size_t>(best_k)][static_cast<std::size_t>(idx(best_i, best_j + 1, nx))];
        j_refined += parabolic_offset(fm, best_p, fp);
    }
    const float best_x = domain.x_min + (nx > 1 ? i_refined / static_cast<float>(nx - 1) : 0.0f) * (domain.x_max - domain.x_min);
    const float best_y = domain.y_min + (ny > 1 ? j_refined / static_cast<float>(ny - 1) : 0.0f) * (domain.y_max - domain.y_min);

    out.x_star = Vec2(best_x, best_y);
    out.t_star_s = cf.times[static_cast<std::size_t>(best_k)];
    out.q0 = q0_mean[static_cast<std::size_t>(best_k)][best_cell];
    out.q0_std = std::sqrt(std::max(0.0f, q0_m2[static_cast<std::size_t>(best_k)][best_cell] - out.q0 * out.q0));
    out.total_released_mass = out.q0;

    // Posterior spatial second moments about x*, y*.
    double var_x = 0.0, var_y = 0.0, mass = 0.0;
    for (int j = 0; j < ny; ++j) {
        const float yfrac = ny > 1 ? static_cast<float>(j) / static_cast<float>(ny - 1) : 0.0f;
        const float y = domain.y_min + yfrac * (domain.y_max - domain.y_min);
        for (int i = 0; i < nx; ++i) {
            const float xfrac = nx > 1 ? static_cast<float>(i) / static_cast<float>(nx - 1) : 0.0f;
            const float x = domain.x_min + xfrac * (domain.x_max - domain.x_min);
            const float p = out.p_star[static_cast<std::size_t>(idx(i, j, nx))] * cell_area;
            var_x += static_cast<double>(p) * (x - best_x) * (x - best_x);
            var_y += static_cast<double>(p) * (y - best_y) * (y - best_y);
            mass += p;
        }
    }
    // Floor at the within-cell std (width/sqrt(12)): a sharp likelihood can
    // collapse the second moment below what the search grid resolves.
    const float quantization_std_x = dxg / std::sqrt(12.0f);
    const float quantization_std_y = dyg / std::sqrt(12.0f);
    out.x_std_m = std::max(
        quantization_std_x, mass > 1.0e-12 ? static_cast<float>(std::sqrt(var_x / mass)) : (domain.x_max - domain.x_min));
    out.y_std_m = std::max(
        quantization_std_y, mass > 1.0e-12 ? static_cast<float>(std::sqrt(var_y / mass)) : (domain.y_max - domain.y_min));

    double var_t = 0.0, mass_t = 0.0;
    for (int k = 0; k < nt; ++k) {
        const double w = static_cast<double>(z_by_time[static_cast<std::size_t>(k)]);
        var_t += w * (cf.times[static_cast<std::size_t>(k)] - out.t_star_s) * (cf.times[static_cast<std::size_t>(k)] - out.t_star_s);
        mass_t += w;
    }
    const float dtg = nt > 1 ? (cf.times.back() - cf.times.front()) / static_cast<float>(nt - 1) : 0.0f;
    const float quantization_std_t = dtg / std::sqrt(12.0f);
    out.t_std_s = std::max(
        quantization_std_t, mass_t > 1.0e-300 ? static_cast<float>(std::sqrt(var_t / mass_t)) : (t_end - build.t_start));

    const float uniform_density = 1.0f / (static_cast<float>(nx * ny) * cell_area);
    out.weakly_identified
        = (out.p_star[static_cast<std::size_t>(idx(best_i, best_j, nx))] < 3.0f * uniform_density);

    out.sensors_used = static_cast<int>(build.series.size());
    int dof = 0;
    double rss = 0.0;
    for (const auto& s : build.series) {
        if (s.gated_count <= 0) {
            continue;
        }
        const std::size_t si = static_cast<std::size_t>(&s - &build.series[0]);
        SourceEstimationObservation fit;
        fit.sensor_index = static_cast<int>(si);
        fit.sensor_position = s.position;
        fit.time_s = out.t_star_s;
        fit.measured_concentration = s.gated_sum;
        fit.predicted_concentration = out.q0 * sample_candidate(cf, si, best_x, best_y, out.t_star_s);
        fit.residual = fit.measured_concentration - fit.predicted_concentration;
        fit.assumed_std = std::max(1.0e-9f, cfg.relative_model_error * std::max(std::abs(fit.measured_concentration), 1.0e-9f));
        rss += (static_cast<double>(fit.residual) * fit.residual) / (static_cast<double>(fit.assumed_std) * fit.assumed_std);
        out.observation_fit.push_back(fit);
        out.observations_used += s.gated_count;
        ++dof;
    }
    out.weighted_rmse = dof > 0 ? static_cast<float>(std::sqrt(rss / dof)) : 0.0f;
    out.success = true;

    std::ostringstream msg;
    msg << std::fixed << std::setprecision(1) << "x*=(" << out.x_star.x() << ", " << out.x_star.y() << ")"
        << ", t*=" << out.t_star_s << " s, q0=" << std::setprecision(3) << out.q0 << " +/- " << out.q0_std;
    out.message = msg.str();
    return out;
}

} // namespace

} // namespace atm
