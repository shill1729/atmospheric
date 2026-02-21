#pragma once

#include <cstddef>

namespace atm {

struct DomainConfig {
    float x_min = 0.0f;
    float x_max = 30000.0f;
    float y_min = 0.0f;
    float y_max = 30000.0f;
    int nx = 200;
    int ny = 200;
};

struct SourceConfig {
    float base_emission = 0.9f;
    float decay_rate = 0.03f;
    float lifespan = 45.0f;
    float sigma = 60.0f;
    float particle_scale = 5.0f;
    int max_sources = 6;
};

struct PhysicsConfig {
    float deposition_rate = 0.012f;
};

struct NumericsConfig {
    float dt = 0.08f;
    float time_scale = 10.0f;
    std::size_t max_particles = 80;
    int max_substeps_per_frame = 20;
};

struct AppConfig {
    unsigned int window_width = 1440;
    unsigned int window_height = 860;
};

struct Config {
    DomainConfig domain;
    SourceConfig source;
    PhysicsConfig physics;
    NumericsConfig numerics;
    AppConfig app;
};

} // namespace atm
