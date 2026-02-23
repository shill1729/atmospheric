#include "app/Application.hpp"

#include <algorithm>

namespace atm {

Application::Application(const Config& config)
    : controller_(config)
    , menu_model_(controller_.current_settings())
    , sensor_manager_(config.app.sensor_sample_period_s, config.app.sensor_noise_std, config.app.sensor_history_capacity)
    , window_(sf::VideoMode({config.app.window_width, config.app.window_height}), "Atmospheric Tool - Phase 1")
    , font_("fonts/arial.ttf") {
    window_.setFramerateLimit(60);

    const float pad = 24.0f;
    const float top_offset = 286.0f;
    const float panel_width = (static_cast<float>(config.app.window_width) - 3.0f * pad) * 0.5f;
    const float panel_height = static_cast<float>(config.app.window_height) - top_offset - 2.0f * pad;

    left_panel_ = sf::FloatRect({pad, top_offset}, {panel_width, panel_height});
    right_panel_ = sf::FloatRect({2.0f * pad + panel_width, top_offset}, {panel_width, panel_height});
    menu_rows_ = 11;
    pde_fixed_color_scale_ = std::max(1.0e-8f, config.app.pde_fixed_color_scale);
    pde_color_scale_runtime_ = pde_fixed_color_scale_;
    concentration_scale_ug_per_m2_ = std::max(1.0e-12f, config.app.concentration_scale_ug_per_m2);
    mixing_height_m_ = std::max(1.0e-6f, config.app.mixing_height_m);
}

void Application::run() {
    while (window_.isOpen()) {
        process_events();
        const float frame_dt = frame_clock_.restart().asSeconds();
        update(frame_dt);
        render();
    }
}

void Application::update(float frame_dt) {
    sim().step(frame_dt);
    sensor_manager_.step(sim().time_s(), sim().pde(), sim().config().domain);
}

bool Application::left_panel_contains(const sf::Vector2i& pixel) const {
    const sf::Vector2f p(static_cast<float>(pixel.x), static_cast<float>(pixel.y));
    return left_panel_.contains(p);
}

bool Application::right_panel_contains(const sf::Vector2i& pixel) const {
    const sf::Vector2f p(static_cast<float>(pixel.x), static_cast<float>(pixel.y));
    return right_panel_.contains(p);
}

Vec2 Application::left_panel_pixel_to_domain(const sf::Vector2i& pixel) const {
    float x_min = 0.0f;
    float x_max = 0.0f;
    float y_min = 0.0f;
    float y_max = 0.0f;
    left_view_bounds(x_min, x_max, y_min, y_max);

    const float sx = (static_cast<float>(pixel.x) - left_panel_.position.x) / left_panel_.size.x;
    const float sy = (static_cast<float>(pixel.y) - left_panel_.position.y) / left_panel_.size.y;

    const float x = x_min + std::clamp(sx, 0.0f, 1.0f) * (x_max - x_min);
    const float y = y_min + std::clamp(sy, 0.0f, 1.0f) * (y_max - y_min);
    return Vec2(x, y);
}

Vec2 Application::right_panel_pixel_to_domain(const sf::Vector2i& pixel) const {
    const auto& d = sim().config().domain;
    const float sx = (static_cast<float>(pixel.x) - right_panel_.position.x) / right_panel_.size.x;
    const float sy = (static_cast<float>(pixel.y) - right_panel_.position.y) / right_panel_.size.y;

    const float x = d.x_min + std::clamp(sx, 0.0f, 1.0f) * (d.x_max - d.x_min);
    const float y = d.y_min + std::clamp(sy, 0.0f, 1.0f) * (d.y_max - d.y_min);
    return Vec2(x, y);
}

sf::Vector2f Application::domain_to_left_panel(const Vec2& x) const {
    float x_min = 0.0f;
    float x_max = 0.0f;
    float y_min = 0.0f;
    float y_max = 0.0f;
    left_view_bounds(x_min, x_max, y_min, y_max);

    const float sx = (x.x() - x_min) / (x_max - x_min);
    const float sy = (x.y() - y_min) / (y_max - y_min);

    return sf::Vector2f(
        left_panel_.position.x + std::clamp(sx, 0.0f, 1.0f) * left_panel_.size.x,
        left_panel_.position.y + std::clamp(sy, 0.0f, 1.0f) * left_panel_.size.y);
}

sf::Vector2f Application::domain_to_right_panel(const Vec2& x) const {
    const auto& d = sim().config().domain;
    const float sx = (x.x() - d.x_min) / (d.x_max - d.x_min);
    const float sy = (x.y() - d.y_min) / (d.y_max - d.y_min);

    return sf::Vector2f(
        right_panel_.position.x + std::clamp(sx, 0.0f, 1.0f) * right_panel_.size.x,
        right_panel_.position.y + std::clamp(sy, 0.0f, 1.0f) * right_panel_.size.y);
}

void Application::left_view_bounds(float& x_min, float& x_max, float& y_min, float& y_max) const {
    const auto& d = sim().config().domain;
    x_min = d.x_min;
    x_max = d.x_max;
    y_min = d.y_min;
    y_max = d.y_max;

    if (!sim().brownian_heat_case()) {
        return;
    }

    const float zoom = 2.4f;
    float cx = 0.5f * (d.x_min + d.x_max);
    float cy = 0.5f * (d.y_min + d.y_max);
    if (sim().source().is_active()) {
        const auto& active = sim().source().active_sources();
        cx = active.back().position.x();
        cy = active.back().position.y();
    }

    const float width = (d.x_max - d.x_min) / zoom;
    const float height = (d.y_max - d.y_min) / zoom;

    x_min = cx - 0.5f * width;
    x_max = cx + 0.5f * width;
    y_min = cy - 0.5f * height;
    y_max = cy + 0.5f * height;

    if (x_min < d.x_min) {
        x_max += (d.x_min - x_min);
        x_min = d.x_min;
    }
    if (x_max > d.x_max) {
        x_min -= (x_max - d.x_max);
        x_max = d.x_max;
    }
    if (y_min < d.y_min) {
        y_max += (d.y_min - y_min);
        y_min = d.y_min;
    }
    if (y_max > d.y_max) {
        y_min -= (y_max - d.y_max);
        y_max = d.y_max;
    }

    x_min = std::max(x_min, d.x_min);
    y_min = std::max(y_min, d.y_min);
    x_max = std::min(x_max, d.x_max);
    y_max = std::min(y_max, d.y_max);
}

Simulator& Application::sim() {
    return controller_.simulator();
}

const Simulator& Application::sim() const {
    return controller_.simulator();
}

} // namespace atm
