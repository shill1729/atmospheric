#include "numerics/AdvectionDiffusionSolver.hpp"

#include <algorithm>

namespace atm {

AdvectionDiffusionSolver::AdvectionDiffusionSolver(const DomainConfig& domain, float deposition_rate)
    : domain_(domain)
    , deposition_rate_(deposition_rate) {
    dx_ = (domain_.x_max - domain_.x_min) / static_cast<float>(std::max(1, domain_.nx - 1));
    dy_ = (domain_.y_max - domain_.y_min) / static_cast<float>(std::max(1, domain_.ny - 1));

    const int n = domain_.nx * domain_.ny;
    c_.assign(n, 0.0f);
    c_next_.assign(n, 0.0f);
}

void AdvectionDiffusionSolver::reset() {
    std::fill(c_.begin(), c_.end(), 0.0f);
    std::fill(c_next_.begin(), c_next_.end(), 0.0f);
}

void AdvectionDiffusionSolver::step(float time_s, float dt, const Fields& fields, const SourceModel& source) {
    if (domain_.nx < 2 || domain_.ny < 2) {
        return;
    }

    for (int j = 0; j < domain_.ny; ++j) {
        for (int i = 0; i < domain_.nx; ++i) {
            const float x = domain_.x_min + static_cast<float>(i) * dx_;
            const float y = domain_.y_min + static_cast<float>(j) * dy_;
            const Vec2 p(x, y);

            const float c = sample(c_, i, j);
            const float cxm = sample(c_, i - 1, j);
            const float cxp = sample(c_, i + 1, j);
            const float cym = sample(c_, i, j - 1);
            const float cyp = sample(c_, i, j + 1);

            const Vec2 w = fields.wind(time_s, p);
            const float kappa_c = std::max(0.0f, fields.scalar_diffusivity(time_s, p));
            const float kappa_e = 0.5f
                * (kappa_c + std::max(0.0f, fields.scalar_diffusivity(time_s, Vec2(x + dx_, y))));
            const float kappa_w = 0.5f
                * (kappa_c + std::max(0.0f, fields.scalar_diffusivity(time_s, Vec2(x - dx_, y))));
            const float kappa_n = 0.5f
                * (kappa_c + std::max(0.0f, fields.scalar_diffusivity(time_s, Vec2(x, y + dy_))));
            const float kappa_s = 0.5f
                * (kappa_c + std::max(0.0f, fields.scalar_diffusivity(time_s, Vec2(x, y - dy_))));

            const float dc_dx = w.x() >= 0.0f ? (c - cxm) / dx_ : (cxp - c) / dx_;
            const float dc_dy = w.y() >= 0.0f ? (c - cym) / dy_ : (cyp - c) / dy_;
            const float adv = -(w.x() * dc_dx + w.y() * dc_dy);

            const float diff_x = (kappa_e * (cxp - c) - kappa_w * (c - cxm)) / (dx_ * dx_);
            const float diff_y = (kappa_n * (cyp - c) - kappa_s * (c - cym)) / (dy_ * dy_);
            const float diff = diff_x + diff_y;

            const float src = source.source_density(p);
            const float react = -deposition_rate_ * c;

            const float next = c + dt * (adv + diff + src + react);
            c_next_[idx(i, j)] = std::max(0.0f, next);
        }
    }

    c_.swap(c_next_);
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

float AdvectionDiffusionSolver::sample(const std::vector<float>& c, int i, int j) const {
    i = std::clamp(i, 0, domain_.nx - 1);
    j = std::clamp(j, 0, domain_.ny - 1);
    return c[idx(i, j)];
}

} // namespace atm
