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
    switch (wind_preset_) {
    case WindPreset::JetShear:
        return wind_jet_shear(time_s, x);
    case WindPreset::VortexPair:
        return wind_vortex_pair(time_s, x);
    case WindPreset::Cellular:
        return wind_cellular(time_s, x);
    }
    return wind_jet_shear(time_s, x);
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

    const float spatial = 1.0f + 0.35f * std::sin(2.0f * PI * xs + 0.03f * time_s)
        * std::cos(2.0f * PI * ys - 0.02f * time_s);
    float k_parallel = 34.0f * spatial;
    float k_cross = 16.0f * spatial;
    if (wind_preset_ == WindPreset::VortexPair) {
        k_parallel *= 1.2f;
        k_cross *= 1.25f;
    } else if (wind_preset_ == WindPreset::Cellular) {
        k_parallel *= 1.15f;
        k_cross *= 1.4f;
    }

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

void Fields::cycle_wind_preset(int direction) {
    int id = static_cast<int>(wind_preset_);
    const int n = 3;
    id = (id + direction) % n;
    if (id < 0) {
        id += n;
    }
    wind_preset_ = static_cast<WindPreset>(id);
}

Fields::WindPreset Fields::wind_preset() const {
    return wind_preset_;
}

std::string_view Fields::wind_preset_name() const {
    switch (wind_preset_) {
    case WindPreset::JetShear:
        return "Jet Shear";
    case WindPreset::VortexPair:
        return "Vortex Pair";
    case WindPreset::Cellular:
        return "Cellular Vortices";
    }
    return "Jet Shear";
}

Vec2 Fields::wind_jet_shear(float time_s, const Vec2& x) const {
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

Vec2 Fields::wind_vortex_pair(float time_s, const Vec2& x) const {
    const float lx = domain_.x_max - domain_.x_min;
    const float ly = domain_.y_max - domain_.y_min;
    const Vec2 c1(domain_.x_min + 0.33f * lx, domain_.y_min + 0.5f * ly);
    const Vec2 c2(domain_.x_min + 0.67f * lx, domain_.y_min + 0.5f * ly);

    const float gamma = 2.0e6f;
    const float core2 = 1200.0f * 1200.0f;

    auto vortex_vel = [&](const Vec2& c, float g) {
        const Vec2 r = x - c;
        const float r2 = r.squaredNorm() + core2;
        const float coeff = g / (2.0f * PI * r2);
        return Vec2(-coeff * r.y(), coeff * r.x());
    };

    const float drift_u = 7.0f + 2.0f * std::sin(0.04f * time_s);
    const float drift_v = 1.2f * std::cos(0.03f * time_s);
    return Vec2(drift_u, drift_v) + vortex_vel(c1, gamma) + vortex_vel(c2, -gamma);
}

Vec2 Fields::wind_cellular(float time_s, const Vec2& x) const {
    const float lx = domain_.x_max - domain_.x_min;
    const float ly = domain_.y_max - domain_.y_min;
    const float xs = (x.x() - domain_.x_min) / lx;
    const float ys = (x.y() - domain_.y_min) / ly;

    const float amp = 9.5e4f;
    const float phase = 0.12f * time_s;
    const float dpsi_dy = amp * (2.0f * PI / ly) * std::sin(2.0f * PI * xs + phase) * std::cos(2.0f * PI * ys);
    const float dpsi_dx = amp * (2.0f * PI / lx) * std::cos(2.0f * PI * xs + phase) * std::sin(2.0f * PI * ys);
    const float u = dpsi_dy + 5.0f;
    const float v = -dpsi_dx;
    return Vec2(u, v);
}

} // namespace atm
