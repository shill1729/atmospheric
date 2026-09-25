#pragma once

#include <cstddef>

namespace atm {

// Clamp on the requested time scale (simulated seconds per wall second).
// The achieved rate is limited by NumericsConfig::frame_step_budget_ms.
constexpr float kMinTimeScale = 0.25f;
constexpr float kMaxTimeScale = 1.0e5f;

struct DomainConfig {
    float x_min = 0.0f;
    float x_max = 5000.0f;
    float y_min = 0.0f;
    float y_max = 5000.0f;
    int nx = 200;
    int ny = 200;
};
struct SourceConfig {
    float base_emission = 1.0f;
    // Fast interactive defaults. The Nov 2024 event's rise/decay constants are
    // ~15 h / ~42 h; see the README's "Faithful NY wildfire event export".
    float decay_rate = 0.03f;
    float lifespan = 100.0f;
    // Matches the ratio of real median inter-site spacing to network extent
    // (~1.9%) applied to the default domain span; see
    // analysis/scales.site_spacing_stats in analysis/calibration_report.md.
    float sigma = 95.0f;
    // SDE births are drawn from N(x_k, (f*sigma)^2 I) with f this fraction.
    // f = 1 matches the PDE source term s(t,x).
    float particle_spread_fraction = 1.0f;
    float particle_scale = 5.0f;
    int max_sources = 10;
};
struct PhysicsConfig {
    float deposition_rate = 0.01f;
    float constant_scalar_diffusivity = 6.0f;
    // Multiplies every wind preset. 1.0 keeps the magnitudes calibrated to
    // the real wind-speed distribution (see calibration_report.md); other
    // values change the advection/diffusion balance (Peclet number), and the
    // explicit stability limit on dt shrinks roughly as 1/wind_scale.
    float wind_scale = 1.0f;
};
struct NumericsConfig {
    float dt = 0.1f;
    float time_scale = 10.0f;
    std::size_t max_particles = 1000000;
    // Stepping stops each frame after frame_step_budget_ms of wall time or
    // max_substeps_per_frame steps, whichever comes first.
    int max_substeps_per_frame = 10000;
    float frame_step_budget_ms = 12.0f;
};

struct AppConfig {
    unsigned int window_width = 1440;
    unsigned int window_height = 860;
    float pde_fixed_color_scale = 1.0e-4f;
    float sensor_sample_period_s = 5.0f;
    // ~5% of a typical default-plume model concentration (quasi-steady peak
    // from analysis.forward_model), matching the relative_model_error assumed
    // by SourceEstimationConfig.
    float sensor_noise_std = 5.4e-6f;
    float sensor_physical_sample_period_s = 1.0f;
    float sensor_spatial_avg_radius_m = 40.0f;
    // 120 reports x 5 s = 10 min, long enough to include a plume's arrival
    // at the sensors.
    std::size_t sensor_history_capacity = 120;
    // Calibrated so a default source burst's quasi-steady peak model
    // concentration maps to the real network-mean event peak (~26 ug/m^3,
    // Nov 2024 wildfire) at mixing_height_m below. See
    // analysis/forward_model.solve_conc_scale and calibration_report.md.
    float concentration_scale_ug_per_m2 = 1.923e8f;
    // Typical daytime convective boundary-layer depth (500-1500 m range;
    // not derivable from these surface-only datasets).
    float mixing_height_m = 800.0f;
};

struct Config {
    DomainConfig domain;
    SourceConfig source;
    PhysicsConfig physics;
    NumericsConfig numerics;
    AppConfig app;
};

} // namespace atm
