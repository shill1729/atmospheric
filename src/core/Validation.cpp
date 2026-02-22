#include "core/Validation.hpp"

#include <cmath>

namespace atm {
namespace {
bool is_finite_positive(float value) {
    return std::isfinite(value) && value > 0.0f;
}

bool is_finite_non_negative(float value) {
    return std::isfinite(value) && value >= 0.0f;
}
}

std::vector<std::string> validate_config(const Config& config) {
    std::vector<std::string> errors;

    if (config.domain.nx < 2 || config.domain.ny < 2) {
        errors.emplace_back("grid dimensions must be >= 2");
    }

    if (!std::isfinite(config.domain.x_min) || !std::isfinite(config.domain.x_max) || !std::isfinite(config.domain.y_min)
        || !std::isfinite(config.domain.y_max)) {
        errors.emplace_back("domain bounds must be finite numbers");
    } else if (config.domain.x_max <= config.domain.x_min || config.domain.y_max <= config.domain.y_min) {
        errors.emplace_back("domain bounds must satisfy x_max > x_min and y_max > y_min");
    }

    if (!is_finite_positive(config.numerics.dt) || !is_finite_positive(config.numerics.time_scale)) {
        errors.emplace_back("dt and time-scale must be positive finite numbers");
    }

    if (config.numerics.max_particles < 1) {
        errors.emplace_back("max-particles must be >= 1");
    }

    if (config.numerics.max_substeps_per_frame < 1) {
        errors.emplace_back("max_substeps_per_frame must be >= 1");
    }

    if (!is_finite_non_negative(config.physics.deposition_rate)) {
        errors.emplace_back("deposition rate must be a finite number >= 0");
    }

    if (!is_finite_non_negative(config.source.base_emission)) {
        errors.emplace_back("source emission must be a finite number >= 0");
    }

    if (!is_finite_non_negative(config.source.decay_rate)) {
        errors.emplace_back("source decay rate must be a finite number >= 0");
    }

    if (!is_finite_positive(config.source.lifespan)) {
        errors.emplace_back("source lifespan must be a finite number > 0");
    }

    if (!is_finite_positive(config.source.sigma)) {
        errors.emplace_back("source sigma must be a finite number > 0");
    }

    if (!is_finite_positive(config.source.particle_scale)) {
        errors.emplace_back("source particle_scale must be a finite number > 0");
    }

    if (config.source.max_sources < 1) {
        errors.emplace_back("source-max must be >= 1");
    }

    if (config.app.window_width < 1 || config.app.window_height < 1) {
        errors.emplace_back("window size must be >= 1x1");
    }

    return errors;
}

} // namespace atm
