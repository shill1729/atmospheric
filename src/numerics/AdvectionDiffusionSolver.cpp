#include "numerics/AdvectionDiffusionSolver.hpp"

#include <algorithm>

namespace atm {
namespace {
int wrap_index(int i, int n) {
    if (n <= 1) {
        return 0;
    }
    int out = i % n;
    if (out < 0) {
        out += n;
    }
    return out;
}

int reflect_index(int i, int n) {
    if (n <= 1) {
        return 0;
    }
    int out = i;
    const int hi = n - 1;
    while (out < 0 || out > hi) {
        if (out < 0) {
            out = -out;
        }
        if (out > hi) {
            out = 2 * hi - out;
        }
    }
    return out;
}
}

AdvectionDiffusionSolver::AdvectionDiffusionSolver(const DomainConfig& domain, float deposition_rate)
    : domain_(domain)
    , deposition_rate_(deposition_rate) {
    dx_ = (domain_.x_max - domain_.x_min) / static_cast<float>(std::max(1, domain_.nx - 1));
    dy_ = (domain_.y_max - domain_.y_min) / static_cast<float>(std::max(1, domain_.ny - 1));

    const int n = domain_.nx * domain_.ny;
    c_.assign(n, 0.0f);
    c_next_.assign(n, 0.0f);
    wind_cache_.assign(n, Vec2::Zero());
    diff_cache_.assign(n, Mat2::Identity());
    source_cache_.assign(n, 0.0f);
}

void AdvectionDiffusionSolver::reset() {
    std::fill(c_.begin(), c_.end(), 0.0f);
    std::fill(c_next_.begin(), c_next_.end(), 0.0f);
}

void AdvectionDiffusionSolver::step(
    float time_s, float dt, const Fields& fields, const SourceModel& source, BoundaryMode boundary_mode) {
    if (domain_.nx < 2 || domain_.ny < 2) {
        return;
    }

    const int n = domain_.nx * domain_.ny;
    if (static_cast<int>(wind_cache_.size()) != n) {
        wind_cache_.assign(n, Vec2::Zero());
        diff_cache_.assign(n, Mat2::Identity());
        source_cache_.assign(n, 0.0f);
    }

    for (int j = 0; j < domain_.ny; ++j) {
        for (int i = 0; i < domain_.nx; ++i) {
            const float x = domain_.x_min + static_cast<float>(i) * dx_;
            const float y = domain_.y_min + static_cast<float>(j) * dy_;
            const Vec2 p(x, y);
            const int k = idx(i, j);
            wind_cache_[k] = fields.wind(time_s, p);
            diff_cache_[k] = fields.diffusivity(time_s, p);
            source_cache_[k] = source.source_density(p);
        }
    }

    auto c_at = [&](int i, int j) { return sample(c_, i, j, boundary_mode); };
    auto d_at = [&](int i, int j) { return sample_diffusivity(diff_cache_, i, j, boundary_mode); };
    auto w_at = [&](int i, int j) { return sample_wind(wind_cache_, i, j, boundary_mode); };
    auto s_at = [&](int i, int j) { return sample_source(source_cache_, i, j, boundary_mode); };

    for (int j = 0; j < domain_.ny; ++j) {
        for (int i = 0; i < domain_.nx; ++i) {
            const float c = c_at(i, j);
            const float cxm = c_at(i - 1, j);
            const float cxp = c_at(i + 1, j);
            const float cym = c_at(i, j - 1);
            const float cyp = c_at(i, j + 1);

            // Conservative first-order upwind discretization of -div(w c).
            // This matters for the Fokker--Planck correspondence: the particle
            // drift is w + div(D), so its density evolves with -div(w c), not
            // merely -w.grad(c), whenever the configured wind has divergence.
            const float ue = 0.5f * (w_at(i, j).x() + w_at(i + 1, j).x());
            const float uw = 0.5f * (w_at(i - 1, j).x() + w_at(i, j).x());
            const float vn = 0.5f * (w_at(i, j).y() + w_at(i, j + 1).y());
            const float vs = 0.5f * (w_at(i, j - 1).y() + w_at(i, j).y());
            const float flux_e = ue * (ue >= 0.0f ? c : cxp);
            const float flux_w = uw * (uw >= 0.0f ? cxm : c);
            const float flux_n = vn * (vn >= 0.0f ? c : cyp);
            const float flux_s = vs * (vs >= 0.0f ? cym : c);
            const float adv = -(flux_e - flux_w) / dx_ - (flux_n - flux_s) / dy_;

            float diff = 0.0f;
            if (diffusion_mode_ == DiffusionMode::ScalarizedTrace) {
                const float kappa_c = std::max(0.0f, 0.5f * d_at(i, j).trace());
                const float kappa_e = 0.5f * (kappa_c + std::max(0.0f, 0.5f * d_at(i + 1, j).trace()));
                const float kappa_w = 0.5f * (kappa_c + std::max(0.0f, 0.5f * d_at(i - 1, j).trace()));
                const float kappa_n = 0.5f * (kappa_c + std::max(0.0f, 0.5f * d_at(i, j + 1).trace()));
                const float kappa_s = 0.5f * (kappa_c + std::max(0.0f, 0.5f * d_at(i, j - 1).trace()));

                const float diff_x = (kappa_e * (cxp - c) - kappa_w * (c - cxm)) / (dx_ * dx_);
                const float diff_y = (kappa_n * (cyp - c) - kappa_s * (c - cym)) / (dy_ * dy_);
                diff = diff_x + diff_y;
            } else {
                const Mat2 de = 0.5f * (d_at(i, j) + d_at(i + 1, j));
                const Mat2 dw = 0.5f * (d_at(i - 1, j) + d_at(i, j));
                const Mat2 dn = 0.5f * (d_at(i, j) + d_at(i, j + 1));
                const Mat2 ds = 0.5f * (d_at(i, j - 1) + d_at(i, j));

                const float cx_e = (c_at(i + 1, j) - c_at(i, j)) / dx_;
                const float cy_e = (c_at(i, j + 1) - c_at(i, j - 1) + c_at(i + 1, j + 1) - c_at(i + 1, j - 1))
                    / (4.0f * dy_);
                const float fx_e = de(0, 0) * cx_e + de(0, 1) * cy_e;

                const float cx_w = (c_at(i, j) - c_at(i - 1, j)) / dx_;
                const float cy_w = (c_at(i - 1, j + 1) - c_at(i - 1, j - 1) + c_at(i, j + 1) - c_at(i, j - 1))
                    / (4.0f * dy_);
                const float fx_w = dw(0, 0) * cx_w + dw(0, 1) * cy_w;

                const float cx_n = (c_at(i + 1, j) - c_at(i - 1, j) + c_at(i + 1, j + 1) - c_at(i - 1, j + 1))
                    / (4.0f * dx_);
                const float cy_n = (c_at(i, j + 1) - c_at(i, j)) / dy_;
                const float fy_n = dn(1, 0) * cx_n + dn(1, 1) * cy_n;

                const float cx_s = (c_at(i + 1, j - 1) - c_at(i - 1, j - 1) + c_at(i + 1, j) - c_at(i - 1, j))
                    / (4.0f * dx_);
                const float cy_s = (c_at(i, j) - c_at(i, j - 1)) / dy_;
                const float fy_s = ds(1, 0) * cx_s + ds(1, 1) * cy_s;

                diff = (fx_e - fx_w) / dx_ + (fy_n - fy_s) / dy_;
            }

            const float src = s_at(i, j);
            const float react = -deposition_rate_ * c;

            const float next = c + dt * (adv + diff + src + react);
            c_next_[idx(i, j)] = std::max(0.0f, next);
        }
    }

    c_.swap(c_next_);
}

void AdvectionDiffusionSolver::cycle_diffusion_mode(int direction) {
    int id = static_cast<int>(diffusion_mode_);
    const int n = 2;
    id = (id + direction) % n;
    if (id < 0) {
        id += n;
    }
    diffusion_mode_ = static_cast<DiffusionMode>(id);
}

AdvectionDiffusionSolver::DiffusionMode AdvectionDiffusionSolver::diffusion_mode() const {
    return diffusion_mode_;
}

void AdvectionDiffusionSolver::set_diffusion_mode(DiffusionMode mode) {
    diffusion_mode_ = mode;
}

std::string_view AdvectionDiffusionSolver::diffusion_mode_name() const {
    switch (diffusion_mode_) {
    case DiffusionMode::ScalarizedTrace:
        return "Scalarized tr(D)/2";
    case DiffusionMode::FullTensorFlux:
        return "Full Tensor Flux";
    }
    return "Full Tensor Flux";
}

int AdvectionDiffusionSolver::nx() const {
    return domain_.nx;
}

int AdvectionDiffusionSolver::ny() const {
    return domain_.ny;
}

float AdvectionDiffusionSolver::dx() const {
    return dx_;
}

float AdvectionDiffusionSolver::dy() const {
    return dy_;
}

const std::vector<float>& AdvectionDiffusionSolver::concentration() const {
    return c_;
}

float AdvectionDiffusionSolver::max_concentration() const {
    float mx = 0.0f;
    for (float v : c_) {
        mx = std::max(mx, v);
    }
    return mx;
}

float AdvectionDiffusionSolver::total_mass() const {
    float sum = 0.0f;
    for (float v : c_) {
        sum += v;
    }
    return sum * dx_ * dy_;
}

int AdvectionDiffusionSolver::idx(int i, int j) const {
    return j * domain_.nx + i;
}

int AdvectionDiffusionSolver::map_index(int i, int n, BoundaryMode boundary_mode) const {
    if (boundary_mode == BoundaryMode::Periodic) {
        return wrap_index(i, n);
    }
    if (boundary_mode == BoundaryMode::Reflecting) {
        return reflect_index(i, n);
    }
    return std::clamp(i, 0, n - 1);
}

float AdvectionDiffusionSolver::sample(const std::vector<float>& c, int i, int j, BoundaryMode boundary_mode) const {
    if (boundary_mode == BoundaryMode::Absorbing) {
        if (i < 0 || i >= domain_.nx || j < 0 || j >= domain_.ny) {
            return 0.0f;
        }
    } else if (boundary_mode == BoundaryMode::Periodic) {
        i = wrap_index(i, domain_.nx);
        j = wrap_index(j, domain_.ny);
    } else {
        i = reflect_index(i, domain_.nx);
        j = reflect_index(j, domain_.ny);
    }

    return c[idx(i, j)];
}

Mat2 AdvectionDiffusionSolver::sample_diffusivity(
    const std::vector<Mat2>& d, int i, int j, BoundaryMode boundary_mode) const {
    i = map_index(i, domain_.nx, boundary_mode);
    j = map_index(j, domain_.ny, boundary_mode);
    return d[idx(i, j)];
}

Vec2 AdvectionDiffusionSolver::sample_wind(const std::vector<Vec2>& w, int i, int j, BoundaryMode boundary_mode) const {
    i = map_index(i, domain_.nx, boundary_mode);
    j = map_index(j, domain_.ny, boundary_mode);
    return w[idx(i, j)];
}

float AdvectionDiffusionSolver::sample_source(
    const std::vector<float>& s, int i, int j, BoundaryMode boundary_mode) const {
    i = map_index(i, domain_.nx, boundary_mode);
    j = map_index(j, domain_.ny, boundary_mode);
    return s[idx(i, j)];
}

} // namespace atm
