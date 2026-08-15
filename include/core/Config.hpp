#pragma once

#include <cstddef>

namespace atm {
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
    float particle_scale = 5.0f;
    int max_sources = 10;
};
struct PhysicsConfig {
    float deposition_rate = 0.01f;
    float constant_scalar_diffusivity = 6.0f;
};
struct NumericsConfig {
    float dt = 0.01f;
    float time_scale = 10.0f;
    std::size_t max_particles = 1000000;
    int max_substeps_per_frame = 20;
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
    std::size_t sensor_history_capacity = 30;
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
