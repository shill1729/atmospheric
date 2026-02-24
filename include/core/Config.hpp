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
    float decay_rate = 0.03f;
    float lifespan = 100.0f;
    float sigma = 50.0f;
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
    float sensor_noise_std = 1.0e-8f;
    float sensor_physical_sample_period_s = 1.0f;
    float sensor_spatial_avg_radius_m = 40.0f;
    std::size_t sensor_history_capacity = 30;
    float concentration_scale_ug_per_m2 = 1.0e5f;
    float mixing_height_m = 1.0f;
};

struct Config {
    DomainConfig domain;
    SourceConfig source;
    PhysicsConfig physics;
    NumericsConfig numerics;
    AppConfig app;
};

} // namespace atm
