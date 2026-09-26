#include "adjoint/SourceEstimator.hpp"
#include "core/Config.hpp"
#include "core/Validation.hpp"
#include "numerics/AdvectionDiffusionSolver.hpp"
#include "numerics/ParticleSystem.hpp"
#include "science/Fields.hpp"
#include "sim/SensorManager.hpp"
#include "sim/Simulator.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace {
bool contains_message(const std::vector<std::string>& messages, const std::string& needle) {
    for (const auto& message : messages) {
        if (message.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}
}

int main() {
    int failures = 0;

    {
        atm::Config bad{};
        bad.source.sigma = 0.0f;
        bad.physics.deposition_rate = -0.1f;
        bad.app.sensor_history_capacity = 0;
        const auto errors = atm::validate_config(bad);
        if (!contains_message(errors, "source sigma")) {
            std::cerr << "Missing validation error for source sigma <= 0\n";
            ++failures;
        }
        if (!contains_message(errors, "deposition rate")) {
            std::cerr << "Missing validation error for negative deposition rate\n";
            ++failures;
        }
        if (!contains_message(errors, "sensor history capacity")) {
            std::cerr << "Missing validation error for sensor history capacity < 1\n";
            ++failures;
        }
    }

    {
        atm::DomainConfig domain;
        domain.x_min = 0.0f;
        domain.x_max = 1.0f;
        domain.y_min = 0.0f;
        domain.y_max = 1.0f;
        domain.nx = 8;
        domain.ny = 8;

        atm::SourceConfig source;
        source.particle_scale = 10.0f;
        source.sigma = 0.5f;

        atm::ParticleSystem particles(domain, source, 500);
        while (particles.boundary_mode() != atm::BoundaryMode::Absorbing) {
            particles.toggle_boundary_mode();
        }
        particles.emit(50.0f, 1.0f, atm::Vec2(-10.0f, -10.0f), 1.0f);

        if (particles.last_emitted_count() != 0) {
            std::cerr << "Emission accounting mismatch for absorbing boundary mode\n";
            ++failures;
        }
        if (!particles.particles().empty()) {
            std::cerr << "Particles should be empty when all births are outside absorbing domain\n";
            ++failures;
        }
    }

    {
        atm::DomainConfig domain;
        domain.x_min = 0.0f;
        domain.x_max = 10.0f;
        domain.y_min = 0.0f;
        domain.y_max = 10.0f;
        domain.nx = 8;
        domain.ny = 8;

        atm::PhysicsConfig physics;
        atm::Fields fields(domain, physics);
        const atm::Vec2 p(5.0f, 5.0f);
        const atm::Mat2 d = fields.diffusivity(0.0f, p);
        if (d(0, 0) <= 0.0f || d(1, 1) <= 0.0f) {
            std::cerr << "Unexpected non-positive diagonal diffusivity in default preset\n";
            ++failures;
        }
    }

    {
        atm::Config cfg{};
        cfg.domain.x_min = 0.0f;
        cfg.domain.x_max = 100.0f;
        cfg.domain.y_min = 0.0f;
        cfg.domain.y_max = 100.0f;
        cfg.domain.nx = 20;
        cfg.domain.ny = 20;
        cfg.numerics.dt = 0.1f;
        cfg.numerics.time_scale = 1.0f;
        cfg.numerics.max_substeps_per_frame = 1;
        cfg.physics.deposition_rate = 0.0f;
        cfg.source.base_emission = 20.0f;
        cfg.source.decay_rate = 0.0f;
        cfg.source.lifespan = cfg.numerics.dt;
        cfg.source.sigma = 10.0f;

        atm::Simulator sim(cfg);
        sim.set_source(atm::Vec2(50.0f, 50.0f));
        sim.step(cfg.numerics.dt);

        if (sim.pde_total_mass() <= 0.0f) {
            std::cerr << "PDE mass should increase on first step when source lifespan equals dt\n";
            ++failures;
        }
    }

    {
        atm::DomainConfig domain;
        domain.x_min = 0.0f;
        domain.x_max = 10.0f;
        domain.y_min = 0.0f;
        domain.y_max = 10.0f;
        domain.nx = 8;
        domain.ny = 8;

        atm::AdvectionDiffusionSolver pde(domain, 0.0f);
        atm::SensorManager sensors(0.5f, 0.0f, 3);
        sensors.add_sensor(atm::Vec2(5.0f, 5.0f), 0.0f);
        sensors.step(2.5f, pde, domain);

        if (sensors.sensors().empty() || sensors.sensors().front().history.size() != 3) {
            std::cerr << "Sensor history should respect configured capacity\n";
            ++failures;
        }

        sensors.set_history_capacity(2);
        if (sensors.sensors().front().history.size() != 2) {
            std::cerr << "Lowering history capacity should trim existing history\n";
            ++failures;
        }
    }

    // --- SDE/PDE/theory cross-check -----------------------------------
    // For zero wind and constant scalar diffusivity D = kappa*I, the
    // Fokker-Planck correspondence (drift = w + div(D) = 0, covariance = 2D)
    // predicts both the PDE concentration field and the SDE particle cloud
    // should have spatial variance growing at rate 2*kappa per axis. These
    // two checks verify the actual numerical solvers against that closed-form
    // theory, independently of each other.
    constexpr float kKappa = 4.0f;

    {
        atm::DomainConfig domain;
        domain.x_min = 0.0f;
        domain.x_max = 2400.0f;
        domain.y_min = 0.0f;
        domain.y_max = 2400.0f;
        domain.nx = 161;
        domain.ny = 161;

        atm::PhysicsConfig physics;
        physics.deposition_rate = 0.0f;
        physics.constant_scalar_diffusivity = kKappa;

        atm::Fields fields(domain, physics);
        fields.set_wind_preset(atm::Fields::WindPreset::Zero);
        fields.set_diffusivity_preset(atm::Fields::DiffusivityPreset::ConstantScalar);

        atm::SourceConfig source_cfg;
        source_cfg.base_emission = 1000.0f;
        source_cfg.decay_rate = 0.0f;
        source_cfg.lifespan = 1.0e6f;
        source_cfg.sigma = 30.0f;
        source_cfg.max_sources = 1;

        atm::SourceModel source_model(domain, source_cfg);
        source_model.activate(atm::Vec2(0.5f * (domain.x_min + domain.x_max), 0.5f * (domain.y_min + domain.y_max)));

        atm::AdvectionDiffusionSolver pde(domain, physics.deposition_rate);
        const float dt = 2.0f;

        // One-step burst: with a zero field, adv/diff contribute nothing, so
        // this deposits exactly dt * source_density as the initial condition.
        pde.step(0.0f, dt, fields, source_model, atm::BoundaryMode::Periodic);
        source_model.deactivate();

        auto variance_xy = [&](const std::vector<float>& c) {
            double mass = 0.0, mx = 0.0, my = 0.0;
            for (int j = 0; j < domain.ny; ++j) {
                for (int i = 0; i < domain.nx; ++i) {
                    const double x = domain.x_min + static_cast<double>(i) * pde.dx();
                    const double y = domain.y_min + static_cast<double>(j) * pde.dy();
                    const double w = c[static_cast<std::size_t>(j * domain.nx + i)];
                    mass += w;
                    mx += w * x;
                    my += w * y;
                }
            }
            mx /= mass;
            my /= mass;
            double vx = 0.0, vy = 0.0;
            for (int j = 0; j < domain.ny; ++j) {
                for (int i = 0; i < domain.nx; ++i) {
                    const double x = domain.x_min + static_cast<double>(i) * pde.dx();
                    const double y = domain.y_min + static_cast<double>(j) * pde.dy();
                    const double w = c[static_cast<std::size_t>(j * domain.nx + i)];
                    vx += w * (x - mx) * (x - mx);
                    vy += w * (y - my) * (y - my);
                }
            }
            return std::make_pair(vx / mass, vy / mass);
        };

        float t = dt;
        const auto [vx_a, vy_a] = variance_xy(pde.concentration());

        for (int k = 0; k < 20; ++k) {
            pde.step(t, dt, fields, source_model, atm::BoundaryMode::Periodic);
            t += dt;
        }
        const float t_b = t;
        const auto [vx_b, vy_b] = variance_xy(pde.concentration());

        for (int k = 0; k < 60; ++k) {
            pde.step(t, dt, fields, source_model, atm::BoundaryMode::Periodic);
            t += dt;
        }
        const float t_c = t;
        const auto [vx_c, vy_c] = variance_xy(pde.concentration());

        const float rate_x = static_cast<float>((vx_c - vx_b) / (t_c - t_b));
        const float rate_y = static_cast<float>((vy_c - vy_b) / (t_c - t_b));
        const float expected = 2.0f * kKappa;
        (void)vx_a;
        (void)vy_a;
        if (std::abs(rate_x - expected) > 0.25f * expected || std::abs(rate_y - expected) > 0.25f * expected) {
            std::cerr << "PDE variance growth rate mismatch: got (" << rate_x << ", " << rate_y << "), expected "
                      << expected << "\n";
            ++failures;
        }
    }

    {
        atm::DomainConfig domain;
        domain.x_min = 0.0f;
        domain.x_max = 4000.0f;
        domain.y_min = 0.0f;
        domain.y_max = 4000.0f;
        domain.nx = 8;
        domain.ny = 8;

        atm::PhysicsConfig physics;
        physics.constant_scalar_diffusivity = kKappa;

        atm::Fields fields(domain, physics);
        fields.set_wind_preset(atm::Fields::WindPreset::Zero);
        fields.set_diffusivity_preset(atm::Fields::DiffusivityPreset::ConstantScalar);

        atm::SourceConfig source_cfg;
        source_cfg.sigma = 50.0f;
        source_cfg.particle_scale = 1.0f;

        atm::ParticleSystem particles(domain, source_cfg, 25000);
        particles.emit(20000.0f, 1.0f, atm::Vec2(2000.0f, 2000.0f), 1.0f);

        if (particles.particles().size() < 15000) {
            std::cerr << "SDE burst emission produced too few particles for a stable variance check\n";
            ++failures;
        } else {
            // Births are spread N(0, (particle_spread_fraction * sigma)^2), so
            // measure variance growth from the post-emission variance rather
            // than assuming a point release.
            auto variance_xy = [&]() {
                double mx = 0.0, my = 0.0;
                for (const auto& p : particles.particles()) {
                    mx += p.x();
                    my += p.y();
                }
                const double n = static_cast<double>(particles.particles().size());
                mx /= n;
                my /= n;
                double vx = 0.0, vy = 0.0;
                for (const auto& p : particles.particles()) {
                    vx += (p.x() - mx) * (p.x() - mx);
                    vy += (p.y() - my) * (p.y() - my);
                }
                return std::make_pair(vx / n, vy / n);
            };
            const auto [vx0, vy0] = variance_xy();

            const float dt = 2.0f;
            const int steps = 50;
            float t = 0.0f;
            for (int k = 0; k < steps; ++k) {
                particles.step(t, dt, fields, 0.0f);
                t += dt;
            }
            const auto [vx1, vy1] = variance_xy();
            const double vx = vx1 - vx0;
            const double vy = vy1 - vy0;

            const float expected = 2.0f * kKappa;
            const float rate_x = static_cast<float>(vx) / t;
            const float rate_y = static_cast<float>(vy) / t;
            if (std::abs(rate_x - expected) > 0.2f * expected || std::abs(rate_y - expected) > 0.2f * expected) {
                std::cerr << "SDE variance growth rate mismatch: got (" << rate_x << ", " << rate_y << "), expected "
                          << expected << "\n";
                ++failures;
            }
        }
    }

    // --- Source estimation methods: known-source recovery ---------------
    // Builds a small deterministic scenario (zero wind, a source at a known
    // location, a ring of noiseless sensors) and checks that all three
    // SourceEstimationMethod implementations recover a location near the
    // true source, rather than crashing, returning NaNs, or silently
    // reporting insufficient_signal.
    {
        atm::Config cfg{};
        cfg.domain.x_min = 0.0f;
        cfg.domain.x_max = 3000.0f;
        cfg.domain.y_min = 0.0f;
        cfg.domain.y_max = 3000.0f;
        cfg.domain.nx = 80;
        cfg.domain.ny = 80;
        cfg.physics.deposition_rate = 0.0f;
        cfg.physics.constant_scalar_diffusivity = 6.0f;
        cfg.source.base_emission = 5.0f;
        cfg.source.decay_rate = 0.0f;
        cfg.source.lifespan = 1.0e6f;
        cfg.source.sigma = 60.0f;
        cfg.source.particle_scale = 0.0f; // sensors read the PDE field only; skip SDE work entirely
        cfg.numerics.dt = 1.0f;
        cfg.numerics.time_scale = 1.0f;
        cfg.numerics.max_substeps_per_frame = 1;

        atm::Simulator sim(cfg);
        while (sim.wind_preset() != atm::Fields::WindPreset::Zero) {
            sim.cycle_wind_model(1);
        }

        const atm::Vec2 true_source(1500.0f, 1500.0f);
        sim.set_source(true_source);

        atm::SensorManager sensor_manager(20.0f, 0.0f, 50, 5.0f, 0.0f);
        constexpr int kNumSensors = 5;
        constexpr float kRadius = 150.0f;
        for (int i = 0; i < kNumSensors; ++i) {
            const float angle = 2.0f * 3.14159265f * static_cast<float>(i) / static_cast<float>(kNumSensors);
            const atm::Vec2 pos(
                true_source.x() + kRadius * std::cos(angle), true_source.y() + kRadius * std::sin(angle));
            sensor_manager.add_sensor(pos, sim.time_s());
        }

        constexpr int kSteps = 400;
        for (int k = 0; k < kSteps; ++k) {
            sim.step(cfg.numerics.dt);
            sensor_manager.step(sim.time_s(), sim.pde(), cfg.domain);
        }

        atm::SourceEstimationConfig est_cfg;
        est_cfg.detection_threshold = 1.0e-12f;
        est_cfg.gaussian_sigma = cfg.source.sigma;
        est_cfg.adjoint_grid_nx = 48;
        est_cfg.adjoint_grid_ny = 48;
        est_cfg.max_lookback_s = 500.0f;
        est_cfg.max_samples_per_sensor = 100;
        est_cfg.adjoint_dt_s = 1.0f;
        est_cfg.candidate_count = 4;
        est_cfg.refinement_levels = 2;
        est_cfg.search_grid_nx = 10;
        est_cfg.search_grid_ny = 10;
        est_cfg.search_time_count = 6;
        est_cfg.response_quadrature_points = 6;
        est_cfg.relative_model_error = 0.05f;
        est_cfg.amplitude_ridge = 1.0e-8f;

        atm::SourceEstimator estimator(est_cfg);
        const auto results = estimator.estimate_all(sensor_manager.sensors(), sim);

        if (results.size() != 3) {
            std::cerr << "estimate_all should return exactly 3 results (one per method)\n";
            ++failures;
        }

        // AdjointBacktracking and RegularizedLeastSquares must land within
        // kTolerance_m of the source. BayesianGrid works on a coarse grid, so
        // its check is that the kCredibleSigma credible region covers it.
        constexpr float kTolerance_m = 400.0f;
        constexpr float kCredibleSigma = 3.0f;
        constexpr float kMinStd_m = 50.0f;
        for (const auto& r : results) {
            const std::string_view name = atm::source_estimation_method_name(r.method);
            if (!r.success || r.insufficient_signal) {
                std::cerr << "SEM method '" << name << "' failed to produce an estimate: " << r.message << "\n";
                ++failures;
                continue;
            }
            if (std::isnan(r.x_star.x()) || std::isnan(r.x_star.y()) || std::isnan(r.t_star_s)) {
                std::cerr << "SEM method '" << name << "' produced NaN output\n";
                ++failures;
                continue;
            }
            if (r.nx < 2 || r.ny < 2 || r.p_star.size() != static_cast<std::size_t>(r.nx * r.ny)) {
                std::cerr << "SEM method '" << name << "' produced no posterior heatmap (p_star)\n";
                ++failures;
            }
            const float dist = (r.x_star - true_source).norm();
            if (r.method == atm::SourceEstimationMethod::BayesianGrid) {
                const float dx = std::abs(r.x_star.x() - true_source.x());
                const float dy = std::abs(r.x_star.y() - true_source.y());
                const float allowed_x = kCredibleSigma * std::max(kMinStd_m, r.x_std_m);
                const float allowed_y = kCredibleSigma * std::max(kMinStd_m, r.y_std_m);
                if (dx > allowed_x || dy > allowed_y) {
                    std::cerr << "SEM method '" << name << "' localized to (" << r.x_star.x() << ", "
                              << r.x_star.y() << ") with std (" << r.x_std_m << ", " << r.y_std_m
                              << "); true source (" << true_source.x() << ", " << true_source.y()
                              << ") falls outside its " << kCredibleSigma << "-sigma credible region\n";
                    ++failures;
                }
                continue;
            }
            if (dist > kTolerance_m) {
                std::cerr << "SEM method '" << name << "' localized to (" << r.x_star.x() << ", " << r.x_star.y()
                          << "), " << dist << " m from the true source (" << true_source.x() << ", "
                          << true_source.y() << "); expected within " << kTolerance_m << " m\n";
                ++failures;
            }
        }
    }

    // --- Noise-only sensors must not count as a detection ------------------
    // Clamped sensor noise has a positive mean. With the noise floor applied,
    // sensors that never saw a plume must give insufficient_signal from every
    // method.
    {
        atm::Config cfg{};
        cfg.domain.nx = 40;
        cfg.domain.ny = 40;
        cfg.numerics.dt = 1.0f;
        cfg.numerics.max_substeps_per_frame = 1;
        atm::Simulator sim(cfg);

        atm::SensorManager sensor_manager(5.0f, 1.6e-5f, 360, 1.0f, 40.0f);
        for (int i = 0; i < 10; ++i) {
            sensor_manager.add_sensor(atm::Vec2(500.0f + 400.0f * static_cast<float>(i), 2500.0f), sim.time_s());
        }
        for (int k = 0; k < 600; ++k) {
            sim.step(cfg.numerics.dt);
            sensor_manager.step(sim.time_s(), sim.pde(), cfg.domain);
        }

        std::size_t reports = 0;
        for (const auto& s : sensor_manager.sensors()) {
            reports += s.history.size();
        }
        atm::SourceEstimator estimator;
        estimator.set_noise_floor(sensor_manager.noise_detection_floor(reports));
        for (const auto& r : estimator.estimate_all(sensor_manager.sensors(), sim)) {
            if (!r.insufficient_signal) {
                std::cerr << "SEM method '" << atm::source_estimation_method_name(r.method)
                          << "' treated sensor noise as a detection (no source was ever placed)\n";
                ++failures;
            }
        }
    }

    if (failures > 0) {
        std::cerr << "Self-check failed with " << failures << " issue(s)\n";
        return 1;
    }

    std::cout << "Self-check passed\n";
    return 0;
}
