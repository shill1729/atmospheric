#include "adjoint/AdjointSolver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace atm {
namespace {
int idx(int i, int j, int nx) {
    return j * nx + i;
}

float sample_clamped(const std::vector<float>& f, int i, int j, int nx, int ny) {
    const int ii = std::clamp(i, 0, nx - 1);
    const int jj = std::clamp(j, 0, ny - 1);
    return f[static_cast<std::size_t>(idx(ii, jj, nx))];
}

void apply_adjoint_bc(
    std::vector<float>& phi, float time_s, const AdjointSolver::Config& cfg, const AdjointSolver::WindFn& wind_fn,
    const AdjointSolver::DiffusivityFn& diffusivity_fn, float dx, float dy) {
    const int nx = cfg.nx;
    const int ny = cfg.ny;
    const float x0 = cfg.domain.x_min;
    const float x1 = cfg.domain.x_max;
    const float y0 = cfg.domain.y_min;
    const float y1 = cfg.domain.y_max;

    for (int j = 0; j < ny; ++j) {
        const float y = y0 + static_cast<float>(j) * dy;
        {
            const Vec2 p(x0, y);
            const float wn = -wind_fn(time_s, p).x();
            if (wn < 0.0f) {
                phi[static_cast<std::size_t>(idx(0, j, nx))] = 0.0f;
            } else {
                const float kappa = std::max(1.0e-6f, diffusivity_fn(time_s, p)(0, 0));
                const float d = kappa / dx;
                phi[static_cast<std::size_t>(idx(0, j, nx))] = (d / (d + wn)) * phi[static_cast<std::size_t>(idx(1, j, nx))];
            }
        }
        {
            const Vec2 p(x1, y);
            const float wn = wind_fn(time_s, p).x();
            if (wn < 0.0f) {
                phi[static_cast<std::size_t>(idx(nx - 1, j, nx))] = 0.0f;
            } else {
                const float kappa = std::max(1.0e-6f, diffusivity_fn(time_s, p)(0, 0));
                const float d = kappa / dx;
                phi[static_cast<std::size_t>(idx(nx - 1, j, nx))] = (d / (d + wn)) * phi[static_cast<std::size_t>(idx(nx - 2, j, nx))];
            }
        }
    }

    for (int i = 0; i < nx; ++i) {
        const float x = x0 + static_cast<float>(i) * dx;
        {
            const Vec2 p(x, y0);
            const float wn = -wind_fn(time_s, p).y();
            if (wn < 0.0f) {
                phi[static_cast<std::size_t>(idx(i, 0, nx))] = 0.0f;
            } else {
                const float kappa = std::max(1.0e-6f, diffusivity_fn(time_s, p)(1, 1));
                const float d = kappa / dy;
                phi[static_cast<std::size_t>(idx(i, 0, nx))] = (d / (d + wn)) * phi[static_cast<std::size_t>(idx(i, 1, nx))];
            }
        }
        {
            const Vec2 p(x, y1);
            const float wn = wind_fn(time_s, p).y();
            if (wn < 0.0f) {
                phi[static_cast<std::size_t>(idx(i, ny - 1, nx))] = 0.0f;
            } else {
                const float kappa = std::max(1.0e-6f, diffusivity_fn(time_s, p)(1, 1));
                const float d = kappa / dy;
                phi[static_cast<std::size_t>(idx(i, ny - 1, nx))] = (d / (d + wn)) * phi[static_cast<std::size_t>(idx(i, ny - 2, nx))];
            }
        }
    }
}
} // namespace

float AdjointSolver::stable_dt(
    float t_start_s, float t_end_s, const Config& cfg, const WindFn& wind_fn, const DiffusivityFn& diffusivity_fn) {
    const int nx = std::max(2, cfg.nx);
    const int ny = std::max(2, cfg.ny);
    const float inv_dx = static_cast<float>(nx - 1) / std::max(1.0e-6f, cfg.domain.x_max - cfg.domain.x_min);
    const float inv_dy = static_cast<float>(ny - 1) / std::max(1.0e-6f, cfg.domain.y_max - cfg.domain.y_min);

    // The presets vary smoothly in time, so sampling the start, middle and
    // end of the window on a strided grid bounds the rate well enough.
    float max_rate = std::max(0.0f, cfg.deposition_rate);
    const int stride = std::max(1, std::min(nx, ny) / 24);
    for (const float t : {t_start_s, 0.5f * (t_start_s + t_end_s), t_end_s}) {
        for (int j = 0; j < ny; j += stride) {
            for (int i = 0; i < nx; i += stride) {
                const Vec2 p(
                    cfg.domain.x_min + static_cast<float>(i) / inv_dx, cfg.domain.y_min + static_cast<float>(j) / inv_dy);
                const Vec2 w = wind_fn(t, p);
                const Mat2 d = diffusivity_fn(t, p);
                const float rate = std::abs(w.x()) * inv_dx + std::abs(w.y()) * inv_dy
                    + 2.0f * (d(0, 0) * inv_dx * inv_dx + d(1, 1) * inv_dy * inv_dy + std::abs(d(0, 1)) * inv_dx * inv_dy)
                    + std::max(0.0f, cfg.deposition_rate);
                max_rate = std::max(max_rate, rate);
            }
        }
    }
    // 0.8 leaves headroom for field extrema between the sampled nodes/times.
    return max_rate > 0.0f ? 0.8f / max_rate : std::numeric_limits<float>::infinity();
}

AdjointSolver::Solution AdjointSolver::solve_backward(
    float t_start_s, float t_end_s, const Config& cfg, const WindFn& wind_fn, const DiffusivityFn& diffusivity_fn,
    const ForcingFn& forcing_fn,
    const std::vector<float>& initial_phi) const {
    Solution out;
    out.nx = std::max(2, cfg.nx);
    out.ny = std::max(2, cfg.ny);
    out.dx = (cfg.domain.x_max - cfg.domain.x_min) / static_cast<float>(out.nx - 1);
    out.dy = (cfg.domain.y_max - cfg.domain.y_min) / static_cast<float>(out.ny - 1);

    const float span = std::max(0.0f, t_end_s - t_start_s);
    const float dt_limit = std::min(std::max(1.0e-3f, cfg.dt_s), stable_dt(t_start_s, t_end_s, cfg, wind_fn, diffusivity_fn));
    const int steps = std::max(1, static_cast<int>(std::ceil(span / dt_limit)));
    const float dt = span / static_cast<float>(steps);

    const std::size_t n = static_cast<std::size_t>(out.nx * out.ny);
    std::vector<float> phi(n, 0.0f);
    if (!initial_phi.empty() && initial_phi.size() == n) {
        phi = initial_phi;
    }
    std::vector<float> next(n, 0.0f);
    std::vector<float> forcing(n, 0.0f);
    // Fields are evaluated once per node per step; the stencils below read
    // neighbors from these caches instead of re-evaluating the (sin/cos-heavy)
    // wind/diffusivity presets up to five times per node.
    std::vector<Vec2> w_cache(n, Vec2::Zero());
    std::vector<Mat2> d_cache(n, Mat2::Zero());

    out.snapshots.reserve(static_cast<std::size_t>(steps + 1));
    out.snapshots.push_back(Snapshot{t_end_s, phi});

    AdjointSolver::Config local_cfg = cfg;
    local_cfg.nx = out.nx;
    local_cfg.ny = out.ny;

    for (int k = 0; k < steps; ++k) {
        const float t = t_end_s - static_cast<float>(k) * dt;
        apply_adjoint_bc(phi, t, local_cfg, wind_fn, diffusivity_fn, out.dx, out.dy);
        forcing_fn(t, forcing);
        for (int j = 0; j < out.ny; ++j) {
            for (int i = 0; i < out.nx; ++i) {
                const Vec2 p(
                    cfg.domain.x_min + static_cast<float>(i) * out.dx, cfg.domain.y_min + static_cast<float>(j) * out.dy);
                const std::size_t c = static_cast<std::size_t>(idx(i, j, out.nx));
                w_cache[c] = wind_fn(t, p);
                d_cache[c] = diffusivity_fn(t, p);
            }
        }

        for (int j = 1; j < out.ny - 1; ++j) {
            for (int i = 1; i < out.nx - 1; ++i) {
                const std::size_t cidx = static_cast<std::size_t>(idx(i, j, out.nx));
                const float c = phi[cidx];
                const float cxm = sample_clamped(phi, i - 1, j, out.nx, out.ny);
                const float cxp = sample_clamped(phi, i + 1, j, out.nx, out.ny);
                const float cym = sample_clamped(phi, i, j - 1, out.nx, out.ny);
                const float cyp = sample_clamped(phi, i, j + 1, out.nx, out.ny);

                const Vec2& w = w_cache[cidx];

                // In reverse-time tau = T-t the adjoint advection equation is
                // phi_tau + (-w).grad(phi) = ..., so upwind against -w.
                const float dphi_dx = w.x() >= 0.0f ? (cxp - c) / out.dx : (c - cxm) / out.dx;
                const float dphi_dy = w.y() >= 0.0f ? (cyp - c) / out.dy : (c - cym) / out.dy;
                const float adv = w.x() * dphi_dx + w.y() * dphi_dy;

                auto c_at = [&](int ii, int jj) { return sample_clamped(phi, ii, jj, out.nx, out.ny); };
                auto d_at = [&](int ii, int jj) -> const Mat2& {
                    const int ci = std::clamp(ii, 0, out.nx - 1);
                    const int cj = std::clamp(jj, 0, out.ny - 1);
                    return d_cache[static_cast<std::size_t>(idx(ci, cj, out.nx))];
                };

                float diff = 0.0f;
                if (cfg.diffusion_mode == DiffusionMode::ScalarizedTrace) {
                    const float kappa_c = std::max(0.0f, 0.5f * d_at(i, j).trace());
                    const float kappa_e = 0.5f * (kappa_c + std::max(0.0f, 0.5f * d_at(i + 1, j).trace()));
                    const float kappa_w = 0.5f * (kappa_c + std::max(0.0f, 0.5f * d_at(i - 1, j).trace()));
                    const float kappa_n = 0.5f * (kappa_c + std::max(0.0f, 0.5f * d_at(i, j + 1).trace()));
                    const float kappa_s = 0.5f * (kappa_c + std::max(0.0f, 0.5f * d_at(i, j - 1).trace()));

                    const float diff_x = (kappa_e * (cxp - c) - kappa_w * (c - cxm)) / (out.dx * out.dx);
                    const float diff_y = (kappa_n * (cyp - c) - kappa_s * (c - cym)) / (out.dy * out.dy);
                    diff = diff_x + diff_y;
                } else {
                    const Mat2 de = 0.5f * (d_at(i, j) + d_at(i + 1, j));
                    const Mat2 dw = 0.5f * (d_at(i - 1, j) + d_at(i, j));
                    const Mat2 dn = 0.5f * (d_at(i, j) + d_at(i, j + 1));
                    const Mat2 ds = 0.5f * (d_at(i, j - 1) + d_at(i, j));

                    const float cx_e = (c_at(i + 1, j) - c_at(i, j)) / out.dx;
                    const float cy_e = (c_at(i, j + 1) - c_at(i, j - 1) + c_at(i + 1, j + 1) - c_at(i + 1, j - 1))
                        / (4.0f * out.dy);
                    const float fx_e = de(0, 0) * cx_e + de(0, 1) * cy_e;

                    const float cx_w = (c_at(i, j) - c_at(i - 1, j)) / out.dx;
                    const float cy_w = (c_at(i - 1, j + 1) - c_at(i - 1, j - 1) + c_at(i, j + 1) - c_at(i, j - 1))
                        / (4.0f * out.dy);
                    const float fx_w = dw(0, 0) * cx_w + dw(0, 1) * cy_w;

                    const float cx_n = (c_at(i + 1, j) - c_at(i - 1, j) + c_at(i + 1, j + 1) - c_at(i - 1, j + 1))
                        / (4.0f * out.dx);
                    const float cy_n = (c_at(i, j + 1) - c_at(i, j)) / out.dy;
                    const float fy_n = dn(1, 0) * cx_n + dn(1, 1) * cy_n;

                    const float cx_s = (c_at(i + 1, j - 1) - c_at(i - 1, j - 1) + c_at(i + 1, j) - c_at(i - 1, j))
                        / (4.0f * out.dx);
                    const float cy_s = (c_at(i, j) - c_at(i, j - 1)) / out.dy;
                    const float fy_s = ds(1, 0) * cx_s + ds(1, 1) * cy_s;

                    diff = (fx_e - fx_w) / out.dx + (fy_n - fy_s) / out.dy;
                }

                const float rhs = adv + diff - cfg.deposition_rate * c + forcing[cidx];
                next[cidx] = std::max(0.0f, c + dt * rhs);
            }
        }

        for (int i = 0; i < out.nx; ++i) {
            next[static_cast<std::size_t>(idx(i, 0, out.nx))] = phi[static_cast<std::size_t>(idx(i, 0, out.nx))];
            next[static_cast<std::size_t>(idx(i, out.ny - 1, out.nx))]
                = phi[static_cast<std::size_t>(idx(i, out.ny - 1, out.nx))];
        }
        for (int j = 0; j < out.ny; ++j) {
            next[static_cast<std::size_t>(idx(0, j, out.nx))] = phi[static_cast<std::size_t>(idx(0, j, out.nx))];
            next[static_cast<std::size_t>(idx(out.nx - 1, j, out.nx))]
                = phi[static_cast<std::size_t>(idx(out.nx - 1, j, out.nx))];
        }

        const float t_prev = t_end_s - static_cast<float>(k + 1) * dt;
        apply_adjoint_bc(next, t_prev, local_cfg, wind_fn, diffusivity_fn, out.dx, out.dy);
        phi.swap(next);
        out.snapshots.push_back(Snapshot{t_prev, phi});
    }

    return out;
}

} // namespace atm
