#pragma once

#include <cstddef>

namespace atm {

// Runtime time-scale bounds (simulated seconds per wall second). The upper
// bound is deliberately generous: the achieved rate is limited by the
// per-frame step budget, not by this clamp.
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
    // decay_rate/lifespan are kept fast for interactive responsiveness (a
    // burst you can watch fully evolve in seconds). The real Nov 2024
    // wildfire event's rise/decay time constants are ~15h/~42h with a ~49h
    // duration above threshold -- see analysis/calibration_report.md and the
    // "Faithful NY wildfire event export" README example for a CLI override
    // that reproduces that scale for offline export runs.
    float decay_rate = 0.03f;
    float lifespan = 100.0f;
    // Matches the ratio of real median inter-site spacing to network extent
    // (~1.9%) applied to the default domain span; see
    // analysis/scales.site_spacing_stats in analysis/calibration_report.md.
    float sigma = 95.0f;
    // SDE particles are born at the source position plus N(0, (f*sigma)^2 I)
    // jitter, where f is this fraction. 1.0 samples births from exactly the
    // Gaussian s(t,x) the PDE injects; the original 0.02 births particles
    // almost at a point (a visually tight plume origin), so the SDE cloud
    // starts narrower than the PDE plume.
    // Was originally 0.02
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
    float dt = 0.01f;
    float time_scale = 10.0f;
    std::size_t max_particles = 1000000;
    // Hard cap on fixed steps per rendered frame. The practical limit is
    // frame_step_budget_ms: stepping stops once a frame has spent that much
    // wall time, so the achieved speed-up is whatever the CPU affords (the
    // HUD reports it next to the requested time scale).
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
    // by SourceEstimationConfig. The previous 1e-8 was ~0.01% of that peak,
    // i.e. an almost noiseless sensor.
    float sensor_noise_std = 5.4e-6f;
    float sensor_physical_sample_period_s = 1.0f;
    float sensor_spatial_avg_radius_m = 40.0f;
    // 120 reports x 5 s = 10 min of history: enough for the estimators to
    // see a plume's arrival at the sensors, which pins down the release
    // time far better than only the most recent ~2.5 min.
    std::size_t sensor_history_capacity = 120;
    // Calibrated so a default source burst's quasi-steady peak model
    // concentration maps to the real network-mean event peak (~26 ug/m^3,
    // Nov 2024 wildfire) at mixing_height_m below. See
    // analysis/forward_model.solve_conc_scale and calibration_report.md.
    float concentration_scale_ug_per_m2 = 1.923e8f;
    // Typical daytime convective boundary-layer depth (500-1500 m range;
    // not derivable from these surface-only datasets). The previous default
    // of 1.0 m was not a physically plausible mixing height.
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
