#include "science/Fields.hpp"

#include <algorithm>
#include <cmath>

namespace atm {
namespace {
constexpr float PI = 3.1415926535f;
}

Fields::Fields(const DomainConfig& domain)
    : domain_(domain) {
}

Vec2 Fields::wind(float time_s, const Vec2& x) const {
    const float lx = domain_.x_max - domain_.x_min;
    const float ly = domain_.y_max - domain_.y_min;
    const float xs = (x.x() - domain_.x_min) / lx;
    const float ys = (x.y() - domain_.y_min) / ly;

    const float u = 18.0f + 10.0f * std::sin(2.0f * PI * ys + 0.05f * time_s)
        + 4.0f * std::cos(4.0f * PI * xs - 0.03f * time_s);
    const float v = 2.0f + 8.0f * std::sin(2.0f * PI * xs - 0.06f * time_s)
        + 2.0f * std::cos(3.0f * PI * ys + 0.04f * time_s);

    return Vec2(u, v);
}

Mat2 Fields::diffusivity(float time_s, const Vec2& x) const {
    const Vec2 w = wind(time_s, x);
    Vec2 e = w;
    const float n = e.norm();
    if (n < 1.0e-4f) {
        e = Vec2(1.0f, 0.0f);
    } else {
        e /= n;
    }
    const Vec2 e_perp(-e.y(), e.x());

    const float lx = domain_.x_max - domain_.x_min;
    const float ly = domain_.y_max - domain_.y_min;
    const float xs = (x.x() - domain_.x_min) / lx;
    const float ys = (x.y() - domain_.y_min) / ly;

    const float spatial = 1.0f + 0.45f * std::sin(2.0f * PI * xs + 0.03f * time_s)
        * std::cos(2.0f * PI * ys - 0.02f * time_s);
    const float k_parallel = 36.0f * spatial;
    const float k_cross = 18.0f * spatial;

    Mat2 out = Mat2::Zero();
    out += k_parallel * (e * e.transpose());
    out += k_cross * (e_perp * e_perp.transpose());
    return out;
}

Vec2 Fields::div_diffusivity(float time_s, const Vec2& x) const {
    const float lx = domain_.x_max - domain_.x_min;
    const float ly = domain_.y_max - domain_.y_min;
    const float hx = std::max(1.0f, 0.002f * lx);
    const float hy = std::max(1.0f, 0.002f * ly);

    auto sample = [&](float px, float py) {
        const float sx = std::clamp(px, domain_.x_min, domain_.x_max);
        const float sy = std::clamp(py, domain_.y_min, domain_.y_max);
        return diffusivity(time_s, Vec2(sx, sy));
    };

    const Mat2 d_px = sample(x.x() + hx, x.y());
    const Mat2 d_mx = sample(x.x() - hx, x.y());
    const Mat2 d_py = sample(x.x(), x.y() + hy);
    const Mat2 d_my = sample(x.x(), x.y() - hy);

    const float dxx_dx = (d_px(0, 0) - d_mx(0, 0)) / (2.0f * hx);
    const float dxy_dy = (d_py(0, 1) - d_my(0, 1)) / (2.0f * hy);
    const float dyx_dx = (d_px(1, 0) - d_mx(1, 0)) / (2.0f * hx);
    const float dyy_dy = (d_py(1, 1) - d_my(1, 1)) / (2.0f * hy);

    return Vec2(dxx_dx + dxy_dy, dyx_dx + dyy_dy);
}

} // namespace atm
