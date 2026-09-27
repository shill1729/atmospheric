#include "science/Fields.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <sstream>

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
    , constant_scalar_diffusivity_(std::max(1.0e-6f, physics.constant_scalar_diffusivity))
    , wind_scale_(std::max(0.0f, physics.wind_scale))
    , veering_(physics.veering) {
    if (physics.wind_preset >= 0 && physics.wind_preset <= static_cast<int>(WindPreset::JenningsReplay)) {
        wind_preset_ = static_cast<WindPreset>(physics.wind_preset);
    }
}

void Fields::set_time_stretch(float stretch) {
    time_stretch_ = stretch > 0.0f ? stretch : 1.0f;
}

void Fields::set_observed_wind(std::vector<ObservedWind> series) {
    observed_wind_ = std::move(series);
}

bool Fields::has_observed_wind() const {
    return !observed_wind_.empty();
}

void Fields::set_wind_scale(float scale) {
    wind_scale_ = std::max(0.0f, scale);
}

float Fields::wind_scale() const {
    return wind_scale_;
}

Vec2 Fields::wind(float time_s, const Vec2& x) const {
    return wind_scale_ * preset_wind(time_s, x);
}

Vec2 Fields::preset_wind(float time_s, const Vec2& x) const {
    switch (wind_preset_) {
    case WindPreset::JetShear:
        return wind_jet_shear(time_s, x);
    case WindPreset::VortexPair:
        return wind_vortex_pair(time_s, x);
    case WindPreset::ShearVortexBlend:
        return wind_shear_vortex_blend(time_s, x);
    case WindPreset::Cellular:
        return wind_cellular(time_s, x);
    case WindPreset::Zero:
        return Vec2::Zero();
    case WindPreset::Uniform:
        return wind_uniform();
    case WindPreset::SolidBodyRotation:
        return wind_solid_body_rotation(x);
    case WindPreset::VeeringUniform:
        return wind_veering_uniform(time_s, x);
    case WindPreset::JenningsReplay:
        return wind_jennings_replay(time_s);
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
    const int n = static_cast<int>(WindPreset::JenningsReplay) + 1;
    int id = static_cast<int>(wind_preset_);
    do {
        id = ((id + direction) % n + n) % n;
    } while (static_cast<WindPreset>(id) == WindPreset::JenningsReplay && !has_observed_wind());
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
    case WindPreset::ShearVortexBlend:
        return "Shear <-> Vortex";
    case WindPreset::Cellular:
        return "Cellular Vortices";
    case WindPreset::Zero:
        return "Zero Wind";
    case WindPreset::Uniform:
        return "Uniform";
    case WindPreset::SolidBodyRotation:
        return "Solid Body Rotation";
    case WindPreset::VeeringUniform:
        return "Veering Uniform";
    case WindPreset::JenningsReplay:
        return "Jennings Replay";
    }
    return "Jet Shear";
}

std::string Fields::wind_preset_details() const {
    std::ostringstream ss;
    ss << wind_preset_name();
    if (wind_preset_ == WindPreset::VeeringUniform) {
        ss << std::fixed << std::setprecision(2) << " (" << veering_.speed_m_s << " m/s from " << veering_.from_deg
           << " deg, veering " << veering_.rate_deg_per_h << " deg/h, perturbation " << veering_.perturbation << ")";
    } else if (wind_preset_ == WindPreset::JenningsReplay) {
        ss << " (hourly network-mean wind measured in wildfire_pm25_dataset.csv; held after its last hour)";
    }
    return ss.str();
}

bool Fields::wind_preset_from_name(std::string name, WindPreset& out) {
    for (char& c : name) {
        c = (c == '_' || c == ' ') ? '-' : static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    static const std::pair<const char*, WindPreset> kNames[] = {
        {"jet-shear", WindPreset::JetShear},
        {"vortex-pair", WindPreset::VortexPair},
        {"shear-vortex-blend", WindPreset::ShearVortexBlend},
        {"cellular", WindPreset::Cellular},
        {"zero", WindPreset::Zero},
        {"uniform", WindPreset::Uniform},
        {"solid-body-rotation", WindPreset::SolidBodyRotation},
        {"veering-uniform", WindPreset::VeeringUniform},
        {"jennings-replay", WindPreset::JenningsReplay},
    };
    for (const auto& [n, preset] : kNames) {
        if (name == n) {
            out = preset;
            return true;
        }
    }
    return false;
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

    // Magnitudes calibrated to the real wind_speed distribution (multimonth
    // dataset: median ~1.9 m/s, p90 ~4.4 m/s; see analysis/calibration_report.md).
    const float u = 2.0f + 1.0f * std::sin(2.0f * PI * ys + 0.05f * time_s)
        + 0.4f * std::cos(4.0f * PI * xs - 0.03f * time_s);
    const float v = 0.3f + 0.8f * std::sin(2.0f * PI * xs - 0.06f * time_s)
        + 0.3f * std::cos(3.0f * PI * ys + 0.04f * time_s);
    return Vec2(u, v);
}

Vec2 Fields::wind_vortex_pair(float time_s, const Vec2& x) const {
    const float lx = domain_.x_max - domain_.x_min;
    const float ly = domain_.y_max - domain_.y_min;
    const Vec2 c1(domain_.x_min + 0.33f * lx, domain_.y_min + 0.5f * ly);
    const Vec2 c2(domain_.x_min + 0.67f * lx, domain_.y_min + 0.5f * ly);

    // Peak circumferential speed ~10 m/s, above the real p90 (~4.4 m/s) so
    // the vortices are visible; see analysis/calibration_report.md.
    const float gamma = 1.5e5f;
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

    // Peak circulation speed ~5 m/s, near the real wind-speed p90; see
    // analysis/calibration_report.md.
    // const float amp = 3.0e3f;
    const float amp = 4.0e3f;
    const float phase = 0.12f * time_s;
    const float dpsi_dy = amp * (2.0f * PI / ly) * std::sin(2.0f * PI * xs + phase) * std::cos(2.0f * PI * ys);
    const float dpsi_dx = amp * (2.0f * PI / lx) * std::cos(2.0f * PI * xs + phase) * std::sin(2.0f * PI * ys);
    const float u = dpsi_dy + 2.0f;
    const float v = -dpsi_dx;
    return Vec2(u, v);
}

Vec2 Fields::wind_uniform() const {
    // Steady westerly with a slight +y (southward on screen) component, at the
    // real median wind speed (~2.2 m/s; see analysis/calibration_report.md).
    return Vec2(2.2f, 0.4f);
}

Vec2 Fields::wind_solid_body_rotation(const Vec2& x) const {
    // Rigid-body rotation about the domain centre.
    // Every fluid element completes one revolution in T = 2*pi/Omega seconds.
    // Omega = 0.0022 rad/s -> T ~ 2856 sim-s, giving ~5 m/s at the mid-edge
    // (r=2500 m), in line with real wind speeds.
    constexpr float OMEGA = 0.0022f;
    const float cx = 0.5f * (domain_.x_min + domain_.x_max);
    const float cy = 0.5f * (domain_.y_min + domain_.y_max);
    const float dx = x.x() - cx;
    const float dy = x.y() - cy;
    return Vec2(-OMEGA * dy, OMEGA * dx);
}

Vec2 Fields::wind_veering_uniform(float time_s, const Vec2& x) const {
    const float hours = time_s * time_stretch_ / 3600.0f;
    const float from_rad = (veering_.from_deg + veering_.rate_deg_per_h * hours) * PI / 180.0f;
    const float speed = veering_.speed_m_s;
    // "From" direction to a vector pointing downwind; domain +y is south.
    Vec2 w(-speed * std::sin(from_rad), speed * std::cos(from_rad));

    // Smooth, slowly drifting spatial variation of relative size `perturbation`:
    // a domain-scale mode plus a shorter one (~1/6 of the domain) so that
    // closely spaced sensors also see somewhat different winds.
    const float xs = normalized_x(domain_, x);
    const float ys = normalized_y(domain_, x);
    const float phase = 0.3f * hours;
    const float dx = std::sin(2.0f * PI * ys + phase) * std::cos(PI * xs)
        + std::sin(12.0f * PI * xs - phase) * std::cos(10.0f * PI * ys + phase);
    const float dy = std::cos(2.0f * PI * xs - phase) * std::sin(PI * ys)
        + std::cos(10.0f * PI * xs + phase) * std::sin(12.0f * PI * ys - phase);
    w.x() += veering_.perturbation * speed * dx;
    w.y() += veering_.perturbation * speed * dy;
    return w;
}

Vec2 Fields::wind_jennings_replay(float time_s) const {
    if (observed_wind_.empty()) {
        return Vec2::Zero();
    }
    const float t = time_s * time_stretch_;
    auto it = std::lower_bound(observed_wind_.begin(), observed_wind_.end(), t,
        [](const ObservedWind& w, float value) { return w.real_time_s < value; });
    ObservedWind w;
    if (it == observed_wind_.begin()) {
        w = observed_wind_.front();
    } else if (it == observed_wind_.end()) {
        w = observed_wind_.back();
    } else {
        const auto& b = *it;
        const auto& a = *(it - 1);
        const float f = (t - a.real_time_s) / std::max(1.0e-6f, b.real_time_s - a.real_time_s);
        w.u_east = a.u_east + f * (b.u_east - a.u_east);
        w.v_north = a.v_north + f * (b.v_north - a.v_north);
    }
    // Domain +y is south.
    return Vec2(w.u_east, -w.v_north);
}

Vec2 Fields::wind_shear_vortex_blend(float time_s, const Vec2& x) const {
    // One full cycle: hold shear -> linear ramp to vortex -> hold vortex -> linear ramp back.
    constexpr float PERIOD_S = 160.0f;
    const float phase = std::fmod(std::max(0.0f, time_s), PERIOD_S) / PERIOD_S;

    float alpha = 0.0f;
    if (phase < 0.25f) {
        alpha = 0.0f;
    } else if (phase < 0.5f) {
        alpha = (phase - 0.25f) / 0.25f;
    } else if (phase < 0.75f) {
        alpha = 1.0f;
    } else {
        alpha = 1.0f - (phase - 0.75f) / 0.25f;
    }

    const Vec2 shear = wind_jet_shear(time_s, x);
    const Vec2 vortex = wind_vortex_pair(time_s, x);
    return (1.0f - alpha) * shear + alpha * vortex;
}

} // namespace atm
