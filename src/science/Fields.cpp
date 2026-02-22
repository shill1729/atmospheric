#include "science/Fields.hpp"

#include <algorithm>
#include <cmath>

namespace atm {
namespace {
constexpr float PI = 3.1415926535f;

float normalized_x(const DomainConfig& d, const Vec2& x) {
    const float lx = std::max(1.0f, d.x_max - d.x_min);
    return (x.x() - d.x_min) / lx;
}

float normalized_y(const DomainConfig& d, const Vec2& x) {
    const float ly = std::max(1.0f, d.y_max - d.y_min);
    return (x.y() - d.y_min) / ly;
}

float scalar_spatial_kappa(const DomainConfig& d, float time_s, const Vec2& x) {
    const float xs = normalized_x(d, x);
    const float ys = normalized_y(d, x);
    const float mod = std::sin(2.0f * PI * xs + 0.05f * time_s) * std::cos(2.0f * PI * ys - 0.04f * time_s);
    return std::max(0.8f, 8.0f + 4.0f * mod);
}

Mat2 tensor_constant_spd() {
    Mat2 d;
    d << 6.0f, 2.0f, 2.0f, 4.0f;
    return d;
}

Mat2 tensor_diagonal_spd(const DomainConfig& d, float time_s, const Vec2& x) {
    const float xs = normalized_x(d, x);
    const float ys = normalized_y(d, x);
    const float d11
        = std::max(0.6f, 3.0f + 1.2f * std::sin(2.0f * PI * ys + 0.06f * time_s) + 0.8f * std::cos(2.0f * PI * xs));
    const float d22
        = std::max(0.8f, 6.0f + 1.8f * std::cos(2.0f * PI * xs - 0.05f * time_s) + 1.0f * std::sin(2.0f * PI * ys));
    Mat2 out = Mat2::Zero();
    out(0, 0) = d11;
    out(1, 1) = d22;
    return out;
}

Mat2 tensor_full_anisotropic_spd(const DomainConfig& d, float time_s, const Vec2& x) {
    const float xs = normalized_x(d, x);
    const float ys = normalized_y(d, x);

    const float lambda1
        = std::max(0.7f, 2.2f + 0.8f * std::sin(2.0f * PI * xs + 0.04f * time_s) + 0.6f * std::cos(2.0f * PI * ys));
    const float lambda2 = std::max(
        lambda1 + 0.5f, 7.5f + 1.6f * std::cos(2.0f * PI * ys - 0.03f * time_s) + 1.2f * std::sin(2.0f * PI * xs));

    const float theta = 0.7f * std::sin(2.0f * PI * xs - 0.05f * time_s) + 0.5f * std::cos(2.0f * PI * ys);
    const float ct = std::cos(theta);
    const float st = std::sin(theta);
    Mat2 r;
    r << ct, -st, st, ct;
    Mat2 lam = Mat2::Zero();
    lam(0, 0) = lambda1;
    lam(1, 1) = lambda2;
    return r * lam * r.transpose();
}
}

Fields::Fields(const DomainConfig& domain, const PhysicsConfig& physics)
    : domain_(domain)
    , constant_scalar_diffusivity_(std::max(1.0e-6f, physics.constant_scalar_diffusivity)) {
}

Vec2 Fields::wind(float time_s, const Vec2& x) const {
    switch (wind_preset_) {
    case WindPreset::JetShear:
        return wind_jet_shear(time_s, x);
    case WindPreset::VortexPair:
        return wind_vortex_pair(time_s, x);
    case WindPreset::Cellular:
        return wind_cellular(time_s, x);
    case WindPreset::Zero:
        return Vec2::Zero();
    }
    return wind_jet_shear(time_s, x);
}

Mat2 Fields::diffusivity(float time_s, const Vec2& x) const {
    switch (diffusivity_preset_) {
    case DiffusivityPreset::ConstantScalar:
        return constant_scalar_diffusivity_ * Mat2::Identity();
    case DiffusivityPreset::SpatialScalar:
        return scalar_spatial_kappa(domain_, time_s, x) * Mat2::Identity();
    case DiffusivityPreset::ConstantTensor:
        return tensor_constant_spd();
    case DiffusivityPreset::DiagonalTensor:
        return tensor_diagonal_spd(domain_, time_s, x);
    case DiffusivityPreset::FullAnisotropicTensor:
        return tensor_full_anisotropic_spd(domain_, time_s, x);
    case DiffusivityPreset::BrownianHalf:
        return 0.5f * Mat2::Identity();
    }
    return constant_scalar_diffusivity_ * Mat2::Identity();
}

Vec2 Fields::div_diffusivity(float time_s, const Vec2& x) const {
    const float lx = domain_.x_max - domain_.x_min;
    const float ly = domain_.y_max - domain_.y_min;
    const float hx = std::max(1.0f, 0.002f * lx);
    const float hy = std::max(1.0f, 0.002f * ly);

    auto sample_d = [&](float px, float py) {
        const float sx = std::clamp(px, domain_.x_min, domain_.x_max);
        const float sy = std::clamp(py, domain_.y_min, domain_.y_max);
        return diffusivity(time_s, Vec2(sx, sy));
    };

    const Mat2 dxp = sample_d(x.x() + hx, x.y());
    const Mat2 dxm = sample_d(x.x() - hx, x.y());
    const Mat2 dyp = sample_d(x.x(), x.y() + hy);
    const Mat2 dym = sample_d(x.x(), x.y() - hy);

    const float d11_dx = (dxp(0, 0) - dxm(0, 0)) / (2.0f * hx);
    const float d12_dy = (dyp(0, 1) - dym(0, 1)) / (2.0f * hy);
    const float d21_dx = (dxp(1, 0) - dxm(1, 0)) / (2.0f * hx);
    const float d22_dy = (dyp(1, 1) - dym(1, 1)) / (2.0f * hy);
    return Vec2(d11_dx + d12_dy, d21_dx + d22_dy);
}

float Fields::scalar_diffusivity(float time_s, const Vec2& x) const {
    if (diffusivity_preset_ == DiffusivityPreset::ConstantScalar) {
        return constant_scalar_diffusivity_;
    }
    if (diffusivity_preset_ == DiffusivityPreset::SpatialScalar) {
        return scalar_spatial_kappa(domain_, time_s, x);
    }
    if (diffusivity_preset_ == DiffusivityPreset::BrownianHalf) {
        return 0.5f;
    }
    return 0.5f * diffusivity(time_s, x).trace();
}

Vec2 Fields::grad_scalar_diffusivity(float time_s, const Vec2& x) const {
    if (diffusivity_preset_ == DiffusivityPreset::ConstantScalar) {
        return Vec2::Zero();
    }

    const float lx = domain_.x_max - domain_.x_min;
    const float ly = domain_.y_max - domain_.y_min;
    const float hx = std::max(1.0f, 0.002f * lx);
    const float hy = std::max(1.0f, 0.002f * ly);

    auto sample = [&](float px, float py) {
        const float sx = std::clamp(px, domain_.x_min, domain_.x_max);
        const float sy = std::clamp(py, domain_.y_min, domain_.y_max);
        return scalar_diffusivity(time_s, Vec2(sx, sy));
    };

    const float dk_dx = (sample(x.x() + hx, x.y()) - sample(x.x() - hx, x.y())) / (2.0f * hx);
    const float dk_dy = (sample(x.x(), x.y() + hy) - sample(x.x(), x.y() - hy)) / (2.0f * hy);
    return Vec2(dk_dx, dk_dy);
}

void Fields::cycle_wind_preset(int direction) {
    int id = static_cast<int>(wind_preset_);
    const int n = 4;
    id = (id + direction) % n;
    if (id < 0) {
        id += n;
    }
    wind_preset_ = static_cast<WindPreset>(id);
}

Fields::WindPreset Fields::wind_preset() const {
    return wind_preset_;
}

void Fields::set_wind_preset(WindPreset preset) {
    wind_preset_ = preset;
}

std::string_view Fields::wind_preset_name() const {
    switch (wind_preset_) {
    case WindPreset::JetShear:
        return "Jet Shear";
    case WindPreset::VortexPair:
        return "Vortex Pair";
    case WindPreset::Cellular:
        return "Cellular Vortices";
    case WindPreset::Zero:
        return "Zero Wind";
    }
    return "Jet Shear";
}

void Fields::cycle_diffusivity_preset(int direction) {
    int id = static_cast<int>(diffusivity_preset_);
    const int n = 6;
    id = (id + direction) % n;
    if (id < 0) {
        id += n;
    }
    diffusivity_preset_ = static_cast<DiffusivityPreset>(id);
}

Fields::DiffusivityPreset Fields::diffusivity_preset() const {
    return diffusivity_preset_;
}

void Fields::set_diffusivity_preset(DiffusivityPreset preset) {
    diffusivity_preset_ = preset;
}

std::string_view Fields::diffusivity_preset_name() const {
    switch (diffusivity_preset_) {
    case DiffusivityPreset::ConstantScalar:
        return "Constant Scalar";
    case DiffusivityPreset::SpatialScalar:
        return "Spatial Scalar";
    case DiffusivityPreset::ConstantTensor:
        return "Constant Tensor";
    case DiffusivityPreset::DiagonalTensor:
        return "Diagonal Tensor";
    case DiffusivityPreset::FullAnisotropicTensor:
        return "Full Anisotropic";
    case DiffusivityPreset::BrownianHalf:
        return "Brownian (k=0.5)";
    }
    return "Constant Scalar";
}

Vec2 Fields::wind_jet_shear(float time_s, const Vec2& x) const {
    const float lx = domain_.x_max - domain_.x_min;
    const float ly = domain_.y_max - domain_.y_min;
    const float xs = (x.x() - domain_.x_min) / lx;
    const float ys = (x.y() - domain_.y_min) / ly;

    const float u = 8.0f + 4.0f * std::sin(2.0f * PI * ys + 0.05f * time_s)
        + 1.5f * std::cos(4.0f * PI * xs - 0.03f * time_s);
    const float v = 0.8f + 3.0f * std::sin(2.0f * PI * xs - 0.06f * time_s)
        + 1.0f * std::cos(3.0f * PI * ys + 0.04f * time_s);
    return Vec2(u, v);
}

Vec2 Fields::wind_vortex_pair(float time_s, const Vec2& x) const {
    const float lx = domain_.x_max - domain_.x_min;
    const float ly = domain_.y_max - domain_.y_min;
    const Vec2 c1(domain_.x_min + 0.33f * lx, domain_.y_min + 0.5f * ly);
    const Vec2 c2(domain_.x_min + 0.67f * lx, domain_.y_min + 0.5f * ly);

    const float gamma = 6.0e5f;
    const float core2 = 1200.0f * 1200.0f;

    auto vortex_vel = [&](const Vec2& c, float g) {
        const Vec2 r = x - c;
        const float r2 = r.squaredNorm() + core2;
        const float coeff = g / (2.0f * PI * r2);
        return Vec2(-coeff * r.y(), coeff * r.x());
    };

    const float drift_u = 3.0f + 1.0f * std::sin(0.04f * time_s);
    const float drift_v = 0.6f * std::cos(0.03f * time_s);
    return Vec2(drift_u, drift_v) + vortex_vel(c1, gamma) + vortex_vel(c2, -gamma);
}

Vec2 Fields::wind_cellular(float time_s, const Vec2& x) const {
    const float lx = domain_.x_max - domain_.x_min;
    const float ly = domain_.y_max - domain_.y_min;
    const float xs = (x.x() - domain_.x_min) / lx;
    const float ys = (x.y() - domain_.y_min) / ly;

    const float amp = 2.5e4f;
    const float phase = 0.12f * time_s;
    const float dpsi_dy = amp * (2.0f * PI / ly) * std::sin(2.0f * PI * xs + phase) * std::cos(2.0f * PI * ys);
    const float dpsi_dx = amp * (2.0f * PI / lx) * std::cos(2.0f * PI * xs + phase) * std::sin(2.0f * PI * ys);
    const float u = dpsi_dy + 2.0f;
    const float v = -dpsi_dx;
    return Vec2(u, v);
}

} // namespace atm
