#include "app/Application.hpp"
#include "core/Config.hpp"

#include <exception>
#include <iostream>
#include <string>

namespace {

void print_help(const char* exe) {
    std::cout
        << "Usage: " << exe << " [options]\n\n"
        << "Options:\n"
        << "  --help                    Show this help message\n"
        << "  --window-width N          Window width in pixels\n"
        << "  --window-height N         Window height in pixels\n"
        << "  --grid-nx N               PDE grid cells in x\n"
        << "  --grid-ny N               PDE grid cells in y\n"
        << "  --dt X                    Fixed simulation step (seconds)\n"
        << "  --time-scale X            Simulated seconds per wall-second\n"
        << "  --max-particles N         Maximum SDE particles\n"
        << "  --deposition X            Deposition/killing rate lambda\n"
        << "  --source-emission X       Initial source emission rate\n"
        << "  --source-decay X          Source emission decay rate\n"
        << "  --source-lifespan X       Source active lifespan (seconds)\n"
        << "  --source-sigma X          Source spatial spread (meters)\n";
}

unsigned int parse_uint(const std::string& v, const std::string& flag) {
    try {
        return static_cast<unsigned int>(std::stoul(v));
    } catch (const std::exception&) {
        throw std::runtime_error("invalid value for " + flag + ": " + v);
    }
}

std::size_t parse_size(const std::string& v, const std::string& flag) {
    try {
        return static_cast<std::size_t>(std::stoull(v));
    } catch (const std::exception&) {
        throw std::runtime_error("invalid value for " + flag + ": " + v);
    }
}

int parse_int(const std::string& v, const std::string& flag) {
    try {
        return std::stoi(v);
    } catch (const std::exception&) {
        throw std::runtime_error("invalid value for " + flag + ": " + v);
    }
}

float parse_float(const std::string& v, const std::string& flag) {
    try {
        return std::stof(v);
    } catch (const std::exception&) {
        throw std::runtime_error("invalid value for " + flag + ": " + v);
    }
}

} // namespace

int main(int argc, char** argv) {
    atm::Config config{};

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto need_value = [&](const std::string& flag) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error("missing value for " + flag);
            }
            return argv[++i];
        };

        if (arg == "--help") {
            print_help(argv[0]);
            return 0;
        }
        if (arg == "--window-width") {
            config.app.window_width = parse_uint(need_value(arg), arg);
            continue;
        }
        if (arg == "--window-height") {
            config.app.window_height = parse_uint(need_value(arg), arg);
            continue;
        }
        if (arg == "--grid-nx") {
            config.domain.nx = parse_int(need_value(arg), arg);
            continue;
        }
        if (arg == "--grid-ny") {
            config.domain.ny = parse_int(need_value(arg), arg);
            continue;
        }
        if (arg == "--dt") {
            config.numerics.dt = parse_float(need_value(arg), arg);
            continue;
        }
        if (arg == "--time-scale") {
            config.numerics.time_scale = parse_float(need_value(arg), arg);
            continue;
        }
        if (arg == "--max-particles") {
            config.numerics.max_particles = parse_size(need_value(arg), arg);
            continue;
        }
        if (arg == "--deposition") {
            config.physics.deposition_rate = parse_float(need_value(arg), arg);
            continue;
        }
        if (arg == "--source-emission") {
            config.source.base_emission = parse_float(need_value(arg), arg);
            continue;
        }
        if (arg == "--source-decay") {
            config.source.decay_rate = parse_float(need_value(arg), arg);
            continue;
        }
        if (arg == "--source-lifespan") {
            config.source.lifespan = parse_float(need_value(arg), arg);
            continue;
        }
        if (arg == "--source-sigma") {
            config.source.sigma = parse_float(need_value(arg), arg);
            continue;
        }

        std::cerr << "Unknown option: " << arg << "\n\n";
        print_help(argv[0]);
        return 1;
    }

    if (config.domain.nx < 2 || config.domain.ny < 2) {
        std::cerr << "grid dimensions must be >= 2\n";
        return 1;
    }
    if (config.numerics.dt <= 0.0f || config.numerics.time_scale <= 0.0f) {
        std::cerr << "dt and time-scale must be positive\n";
        return 1;
    }

    atm::Application app(config);
    app.run();
    return 0;
}
