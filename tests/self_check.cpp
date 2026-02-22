#include "core/Config.hpp"
#include "core/Validation.hpp"
#include "numerics/ParticleSystem.hpp"
#include "science/Fields.hpp"

#include <iostream>
#include <string>
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
        const auto errors = atm::validate_config(bad);
        if (!contains_message(errors, "source sigma")) {
            std::cerr << "Missing validation error for source sigma <= 0\n";
            ++failures;
        }
        if (!contains_message(errors, "deposition rate")) {
            std::cerr << "Missing validation error for negative deposition rate\n";
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
        particles.toggle_boundary_mode();
        particles.toggle_boundary_mode();
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

    if (failures > 0) {
        std::cerr << "Self-check failed with " << failures << " issue(s)\n";
        return 1;
    }

    std::cout << "Self-check passed\n";
    return 0;
}
