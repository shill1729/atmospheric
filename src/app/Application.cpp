#include "app/Application.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace atm {
namespace {
sf::Color heat_color(float v) {
    const float t = std::clamp(v, 0.0f, 1.0f);
    const float r = std::clamp(3.0f * t - 1.0f, 0.0f, 1.0f);
    const float g = std::clamp(3.0f * t, 0.0f, 1.0f);
    const float b = std::clamp(2.0f - 3.0f * t, 0.0f, 1.0f);
    return sf::Color(
        static_cast<std::uint8_t>(255.0f * r),
        static_cast<std::uint8_t>(255.0f * g),
        static_cast<std::uint8_t>(255.0f * b),
        230);
}

float mass_scale(MassDisplayUnit unit) {
    return unit == MassDisplayUnit::MicrogramsPerSquareMeter ? 1.0e6f : 1.0f;
}

const char* mass_unit_label(MassDisplayUnit unit) {
    return unit == MassDisplayUnit::MicrogramsPerSquareMeter ? "ug/m^2" : "g/m^2";
}

const char* boundary_mode_label(BoundaryMode mode) {
    switch (mode) {
    case BoundaryMode::Periodic:
        return "periodic";
    case BoundaryMode::Reflecting:
        return "reflecting";
    case BoundaryMode::Absorbing:
        return "absorbing";
    }
    return "periodic";
}

}

Application::Application(const Config& config)
    : controller_(config)
    , menu_model_(controller_.current_settings())
    , sensor_manager_(config.app.sensor_sample_period_s, config.app.sensor_noise_std)
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
}

void Application::run() {
    while (window_.isOpen()) {
        process_events();
        const float frame_dt = frame_clock_.restart().asSeconds();
        update(frame_dt);
        render();
    }
}

void Application::process_events() {
    while (const std::optional event = window_.pollEvent()) {
        if (event->is<sf::Event::Closed>()) {
            window_.close();
            continue;
        }

        if (const auto* key = event->getIf<sf::Event::KeyPressed>()) {
            if (key->code == sf::Keyboard::Key::F1) {
                help_open_ = !help_open_;
                continue;
            }
            if (key->code == sf::Keyboard::Key::Escape) {
                menu_open_ = !menu_open_;
                continue;
            }

            if (menu_open_) {
                if (key->code == sf::Keyboard::Key::Up) {
                    menu_index_ = std::max(0, menu_index_ - 1);
                } else if (key->code == sf::Keyboard::Key::Down) {
                    menu_index_ = std::min(menu_rows_ - 1, menu_index_ + 1);
                } else if (key->code == sf::Keyboard::Key::Left) {
                    apply_menu_adjustment(-1);
                } else if (key->code == sf::Keyboard::Key::Right || key->code == sf::Keyboard::Key::Enter) {
                    apply_menu_adjustment(1);
                }
                continue;
            }

            if (key->code == sf::Keyboard::Key::Space) {
                sim().toggle_paused();
            }
            if (key->code == sf::Keyboard::Key::R) {
                sim().reset();
                sensor_manager_.clear();
                clear_source_estimation();
            }
            if (key->code == sf::Keyboard::Key::B) {
                sim().toggle_boundary_mode();
            }
            if (key->code == sf::Keyboard::Key::LBracket) {
                sim().scale_time(0.8f);
            }
            if (key->code == sf::Keyboard::Key::RBracket) {
                sim().scale_time(1.25f);
            }
            if (key->code == sf::Keyboard::Key::Backslash) {
                sim().reset_time_scale();
            }
            if (key->code == sf::Keyboard::Key::W) {
                sim().cycle_wind_model(1);
            }
            if (key->code == sf::Keyboard::Key::K) {
                sim().cycle_diffusion_model(1);
            }
            if (key->code == sf::Keyboard::Key::P) {
                sim().cycle_pde_diffusion_mode(1);
                menu_model_.sync_from_current(controller_.current_settings());
            }
            if (key->code == sf::Keyboard::Key::C) {
                pde_auto_color_scale_ = !pde_auto_color_scale_;
            }
            if (key->code == sf::Keyboard::Key::U) {
                mass_unit_ = (mass_unit_ == MassDisplayUnit::MicrogramsPerSquareMeter)
                    ? MassDisplayUnit::GramsPerSquareMeter
                    : MassDisplayUnit::MicrogramsPerSquareMeter;
            }
            if (key->code == sf::Keyboard::Key::H) {
                sim().toggle_brownian_heat_case();
            }
            if (key->code == sf::Keyboard::Key::E) {
                run_source_estimation();
            }
            if (key->code == sf::Keyboard::Key::J) {
                show_adjoint_overlay_ = !show_adjoint_overlay_;
            }
        }

        if (const auto* click = event->getIf<sf::Event::MouseButtonPressed>()) {
            if (click->button == sf::Mouse::Button::Left) {
                const TopToolbarClickResult toolbar_click
                    = top_toolbar_.handle_click(click->position, menu_model_, controller_);
                if (toolbar_click.settings_changed) {
                    const RuntimeSettings settings = controller_.current_settings();
                    pde_fixed_color_scale_ = std::max(1.0e-8f, settings.pde_fixed_color_scale);
                    pde_color_scale_runtime_ = pde_fixed_color_scale_;
                    sensor_manager_.set_sample_period(settings.sensor_sample_period_s);
                    sensor_manager_.set_noise_std(settings.sensor_noise_std);
                }
                if (toolbar_click.recreated_simulator) {
                    sensor_manager_.clear();
                    clear_source_estimation();
                    menu_open_ = false;
                }
                if (toolbar_click.request_source_estimate) {
                    run_source_estimation();
                }
                if (toolbar_click.consumed) {
                    continue;
                }
            }
            if (click->button == sf::Mouse::Button::Left && left_panel_contains(click->position)) {
                sim().set_source(left_panel_pixel_to_domain(click->position));
            } else if (click->button == sf::Mouse::Button::Left && right_panel_contains(click->position)) {
                sensor_manager_.add_sensor(right_panel_pixel_to_domain(click->position), sim().time_s());
            }
        }
    }
}

void Application::update(float frame_dt) {
    sim().step(frame_dt);
    sensor_manager_.step(sim().time_s(), sim().pde(), sim().config().domain);
}

void Application::render() {
    window_.clear(sf::Color(15, 19, 25));
    const bool toolbar_open = top_toolbar_.has_open_menu(menu_model_);

    sf::RectangleShape left_rect({left_panel_.size.x, left_panel_.size.y});
    left_rect.setPosition({left_panel_.position.x, left_panel_.position.y});
    left_rect.setFillColor(sf::Color(17, 30, 44));
    left_rect.setOutlineColor(sf::Color(88, 140, 178));
    left_rect.setOutlineThickness(2.0f);
    window_.draw(left_rect);

    sf::RectangleShape right_rect({right_panel_.size.x, right_panel_.size.y});
    right_rect.setPosition({right_panel_.position.x, right_panel_.position.y});
    right_rect.setFillColor(sf::Color(24, 24, 24));
    right_rect.setOutlineColor(sf::Color(110, 110, 110));
    right_rect.setOutlineThickness(2.0f);
    window_.draw(right_rect);
    draw_pde_heatmap();
    draw_adjoint_overlay();
    draw_sensor_overlay();

    if (show_wind_) {
        draw_wind_field();
    }

    const auto& pts = sim().particles().particles();
    const auto& trails = sim().particles().trails();
    std::size_t line_segments = 0;
    for (const auto& trail : trails) {
        if (trail.size() > 1) {
            line_segments += trail.size() - 1;
        }
    }

    sf::VertexArray tails(sf::PrimitiveType::Lines, line_segments * 2);
    std::size_t vi = 0;
    const float panel_diag = std::sqrt(left_panel_.size.x * left_panel_.size.x + left_panel_.size.y * left_panel_.size.y);
    const float max_segment_px = 0.35f * panel_diag;
    for (const auto& trail : trails) {
        if (trail.size() <= 1) {
            continue;
        }
        for (std::size_t k = 1; k < trail.size(); ++k) {
            const sf::Vector2f p_prev = domain_to_left_panel(trail[k - 1]);
            const sf::Vector2f p_curr = domain_to_left_panel(trail[k]);
            const float seg_dx = p_curr.x - p_prev.x;
            const float seg_dy = p_curr.y - p_prev.y;
            const float seg_len = std::sqrt(seg_dx * seg_dx + seg_dy * seg_dy);
            if (seg_len > max_segment_px) {
                continue;
            }

            const float a = static_cast<float>(k) / static_cast<float>(trail.size() - 1);
            const std::uint8_t alpha0 = static_cast<std::uint8_t>(20.0f + 90.0f * a);
            const std::uint8_t alpha1 = static_cast<std::uint8_t>(30.0f + 130.0f * a);
            tails[vi].position = p_prev;
            tails[vi].color = sf::Color(80, 225, 120, alpha0);
            ++vi;
            tails[vi].position = p_curr;
            tails[vi].color = sf::Color(120, 255, 150, alpha1);
            ++vi;
        }
    }
    tails.resize(vi);
    window_.draw(tails);

    sf::CircleShape particle(2.0f);
    particle.setOrigin({2.0f, 2.0f});
    particle.setOutlineThickness(0.6f);
    particle.setOutlineColor(sf::Color(210, 255, 220, 180));
    particle.setFillColor(sf::Color(90, 220, 120, 135));
    for (const Vec2& p : pts) {
        particle.setPosition(domain_to_left_panel(p));
        window_.draw(particle);
    }

    if (sim().source().is_active()) {
        sf::CircleShape marker(3.2f);
        marker.setOrigin({3.2f, 3.2f});
        marker.setFillColor(sf::Color(255, 80, 80, 190));
        marker.setOutlineColor(sf::Color::White);
        marker.setOutlineThickness(0.8f);

        for (const auto& src : sim().source().active_sources()) {
            marker.setPosition(domain_to_left_panel(src.position));
            window_.draw(marker);
        }
    }

    if (source_estimation_.has_result) {
        sf::CircleShape marker(4.0f);
        marker.setOrigin({4.0f, 4.0f});
        marker.setFillColor(sf::Color(255, 220, 40, 235));
        marker.setOutlineColor(sf::Color::Black);
        marker.setOutlineThickness(1.0f);
        marker.setPosition(domain_to_left_panel(source_estimation_.x_star));
        window_.draw(marker);
        marker.setPosition(domain_to_right_panel(source_estimation_.x_star));
        window_.draw(marker);
    }

    sf::Text left_label(font_, "SDE Particle Panel", 16);
    left_label.setPosition({left_panel_.position.x + 10.0f, left_panel_.position.y + 8.0f});
    left_label.setFillColor(sf::Color(180, 210, 230));
    window_.draw(left_label);

    sf::Text right_label(font_, "PDE Concentration Heatmap", 16);
    right_label.setPosition({right_panel_.position.x + 10.0f, right_panel_.position.y + 8.0f});
    right_label.setFillColor(sf::Color(185, 185, 185));
    window_.draw(right_label);

    if (!toolbar_open) {
        draw_hud_cards();
        draw_control_strip();
    }
    if (menu_open_) {
        draw_menu_overlay();
    }
    if (help_open_) {
        draw_help_overlay();
    }
    if (toolbar_open) {
        sf::RectangleShape dim({static_cast<float>(window_.getSize().x), static_cast<float>(window_.getSize().y)});
        dim.setFillColor(sf::Color(6, 10, 16, 120));
        window_.draw(dim);
    }
    top_toolbar_.draw(window_, font_, menu_model_);
    top_toolbar_.draw_active_menu(window_, font_, menu_model_);
    window_.display();
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

void Application::draw_wind_field() {
    const auto& d = sim().config().domain;
    float x_min = 0.0f;
    float x_max = 0.0f;
    float y_min = 0.0f;
    float y_max = 0.0f;
    left_view_bounds(x_min, x_max, y_min, y_max);

    const int nx = std::clamp(d.nx / 12, 8, 16);
    const int ny = std::clamp(d.ny / 12, 6, 12);
    const float dx = (x_max - x_min) / static_cast<float>(nx - 1);
    const float dy = (y_max - y_min) / static_cast<float>(ny - 1);

    struct WindSample {
        Vec2 x;
        Vec2 w;
        float norm = 0.0f;
    };
    std::vector<WindSample> samples;
    samples.reserve(static_cast<std::size_t>(nx * ny));

    float max_norm = 0.0f;
    for (int iy = 0; iy < ny; ++iy) {
        for (int ix = 0; ix < nx; ++ix) {
            WindSample s;
            s.x = Vec2(x_min + ix * dx, y_min + iy * dy);
            s.w = sim().wind_at(s.x);
            s.norm = s.w.norm();
            max_norm = std::max(max_norm, s.norm);
            samples.push_back(s);
        }
    }

    if (samples.empty()) {
        return;
    }

    sf::VertexArray arrows(sf::PrimitiveType::Lines);
    arrows.resize(samples.size() * 6);
    const sf::Color c(120, 200, 255, 150);
    const float min_len_px = 8.0f;
    const float max_len_px = 30.0f;
    const float inv_max = max_norm > 1.0e-8f ? (1.0f / max_norm) : 0.0f;

    std::size_t i = 0;
    for (const auto& s : samples) {
        const sf::Vector2f p0 = domain_to_left_panel(s.x);

        sf::Vector2f dir(1.0f, 0.0f);
        if (s.norm > 1.0e-8f) {
            const sf::Vector2f p_unit = domain_to_left_panel(s.x + s.w);
            sf::Vector2f pix_vec = p_unit - p0;
            const float pix_norm = std::sqrt(pix_vec.x * pix_vec.x + pix_vec.y * pix_vec.y);
            if (pix_norm > 1.0e-6f) {
                dir = pix_vec / pix_norm;
            }
        }

        const float strength = s.norm * inv_max;
        const float len = min_len_px + (max_len_px - min_len_px) * strength;
        const sf::Vector2f p1 = p0 + dir * len;
        const sf::Vector2f perp(-dir.y, dir.x);
        const float wing_back = std::max(4.0f, 0.42f * len);
        const float wing_side = std::max(2.0f, 0.22f * len);
        const sf::Vector2f p2 = p1 - dir * wing_back + perp * wing_side;
        const sf::Vector2f p3 = p1 - dir * wing_back - perp * wing_side;

        arrows[i].position = p0;
        arrows[i].color = c;
        ++i;
        arrows[i].position = p1;
        arrows[i].color = c;
        ++i;

        arrows[i].position = p1;
        arrows[i].color = c;
        ++i;
        arrows[i].position = p2;
        arrows[i].color = c;
        ++i;

        arrows[i].position = p1;
        arrows[i].color = c;
        ++i;
        arrows[i].position = p3;
        arrows[i].color = c;
        ++i;
    }

    window_.draw(arrows);
}

void Application::draw_hud_cards() {
    const auto& source = sim().source();
    const auto& particles = sim().particles();
    const float mscale = mass_scale(mass_unit_);
    const float sde_mass = sim().sde_total_mass() * mscale;
    const float pde_mass = sim().pde_total_mass() * mscale;
    const float ratio = sim().mass_ratio_sde_to_pde();

    auto draw_card = [&](float x, float y, float w, float h, const std::string& title, const std::string& body) {
        sf::RectangleShape card({w, h});
        card.setPosition({x, y});
        card.setFillColor(sf::Color(12, 18, 28, 190));
        card.setOutlineThickness(1.0f);
        card.setOutlineColor(sf::Color(72, 112, 150, 190));
        window_.draw(card);

        sf::Text t(font_, title, 14);
        t.setPosition({x + 10.0f, y + 8.0f});
        t.setFillColor(sf::Color(186, 220, 238));
        window_.draw(t);

        sf::Text b(font_, body, 12);
        b.setPosition({x + 10.0f, y + 30.0f});
        b.setLineSpacing(0.95f);
        b.setFillColor(sf::Color(216, 228, 238));
        window_.draw(b);
    };

    std::ostringstream status;
    status << std::fixed << std::setprecision(2)
           << "t: " << sim().time_s() << " s"
           << "\nspeed: x" << sim().time_scale()
           << "\nstate: " << (sim().paused() ? "paused" : "running")
           << "\nwind: " << sim().wind_model_name()
           << "\ndiff: " << sim().diffusion_model_name()
           << "\nPDE diff: " << sim().pde_diffusion_mode_name()
           << "\nPDE color: " << (pde_auto_color_scale_ ? "Auto" : "Fixed")
           << "\nBH case: " << (sim().brownian_heat_case() ? "ON" : "off");

    std::ostringstream source_text;
    source_text << std::scientific << std::setprecision(2)
                << "q_total(t): " << source.emission_rate()
                << "\nemitted: " << sim().last_emitted_total() << " / step"
                << "\nsources: " << source.active_count() << " / " << source.max_sources()
                << "\nnewest: "
                << (source.is_active()
                        ? (std::to_string(static_cast<int>(source.newest_age_s())) + "s / "
                            + std::to_string(static_cast<int>(source.lifespan_s())) + "s")
                        : std::string("inactive"));

    std::ostringstream physics;
    physics << std::fixed << std::setprecision(2)
            << "particles: " << particles.particles().size()
            << "\nBC: " << boundary_mode_label(sim().boundary_mode())
            << "\nPDE max c: " << std::scientific << std::setprecision(3) << sim().pde().max_concentration()
            << "\nM_sde: " << std::scientific << std::setprecision(2) << sde_mass
            << "\nM_pde: " << std::scientific << std::setprecision(2) << pde_mass
            << "\nratio: " << std::fixed << std::setprecision(3) << ratio << " (" << mass_unit_label(mass_unit_)
            << ")";

    const float top = 38.0f;
    const float left = 24.0f;
    const float gap = 12.0f;
    const float card_w = 250.0f;
    const float card_h = 170.0f;
    draw_card(left, top, card_w, card_h, "Status", status.str());
    draw_card(left + card_w + gap, top, card_w, card_h, "Source", source_text.str());
    draw_card(left + 2.0f * (card_w + gap), top, card_w, card_h, "Physics", physics.str());
}

void Application::draw_control_strip() {
    sf::RectangleShape strip({980.0f, 44.0f});
    strip.setPosition({24.0f, 214.0f});
    strip.setFillColor(sf::Color(10, 16, 24, 188));
    strip.setOutlineThickness(1.0f);
    strip.setOutlineColor(sf::Color(70, 108, 145, 180));
    window_.draw(strip);

    sf::Text control_text(
        font_,
        "Toolbar: File | Options | PDE | L-click source/sensor | E estimate | J adjoint view | F1 controls",
        13);
    control_text.setPosition({34.0f, 218.0f});
    control_text.setFillColor(sf::Color(168, 192, 210));
    window_.draw(control_text);

    const std::string status = source_estimation_.status.empty()
        ? "Estimator: idle"
        : ("Estimator: " + source_estimation_.status);
    sf::Text status_text(font_, status, 12);
    status_text.setPosition({34.0f, 235.0f});
    status_text.setFillColor(sf::Color(196, 216, 232));
    window_.draw(status_text);
}


void Application::draw_menu_overlay() {
    sf::RectangleShape dim({static_cast<float>(window_.getSize().x), static_cast<float>(window_.getSize().y)});
    dim.setFillColor(sf::Color(6, 10, 16, 170));
    window_.draw(dim);

    const sf::Vector2f panel_size(620.0f, 620.0f);
    const sf::Vector2f panel_pos(
        0.5f * (static_cast<float>(window_.getSize().x) - panel_size.x),
        0.5f * (static_cast<float>(window_.getSize().y) - panel_size.y));

    sf::RectangleShape panel(panel_size);
    panel.setPosition(panel_pos);
    panel.setFillColor(sf::Color(20, 28, 40, 235));
    panel.setOutlineThickness(2.0f);
    panel.setOutlineColor(sf::Color(110, 150, 186, 240));
    window_.draw(panel);

    sf::Text title(font_, "Preferences", 22);
    title.setPosition({panel_pos.x + 20.0f, panel_pos.y + 14.0f});
    title.setFillColor(sf::Color(210, 234, 250));
    window_.draw(title);

    std::array<std::string, 11> rows;
    std::ostringstream speed;
    speed << std::fixed << std::setprecision(2) << "Simulation Speed      x" << sim().time_scale();
    rows[0] = speed.str();
    rows[1] = std::string("Wind Model            ") + std::string(sim().wind_model_name());
    rows[2] = std::string("Diffusivity Model     ") + std::string(sim().diffusion_model_name());
    rows[3] = std::string("PDE Diffusion         ") + std::string(sim().pde_diffusion_mode_name());
    rows[4] = std::string("Boundary Mode         ") + boundary_mode_label(sim().boundary_mode());
    rows[5] = std::string("Wind Vectors          ") + (show_wind_ ? "On" : "Off");
    rows[6] = std::string("Brownian/Heat Case    ") + (sim().brownian_heat_case() ? "ON" : "Off");
    rows[7] = std::string("Mass Units            ") + mass_unit_label(mass_unit_);
    rows[8] = std::string("PDE Color Scale       ") + (pde_auto_color_scale_ ? "Auto" : "Fixed");
    rows[9] = "Trail Length          " + std::to_string(sim().trail_length());
    rows[10] = "Reset Simulation";

    const float start_y = panel_pos.y + 62.0f;
    for (int i = 0; i < static_cast<int>(rows.size()); ++i) {
        sf::RectangleShape row_bg({panel_size.x - 36.0f, 42.0f});
        row_bg.setPosition({panel_pos.x + 18.0f, start_y + 48.0f * i});
        row_bg.setFillColor(i == menu_index_ ? sf::Color(58, 92, 126, 210) : sf::Color(30, 40, 56, 170));
        window_.draw(row_bg);

        sf::Text row(font_, rows[i], 18);
        row.setPosition({panel_pos.x + 30.0f, start_y + 48.0f * i + 8.0f});
        row.setFillColor(i == menu_index_ ? sf::Color(236, 245, 252) : sf::Color(186, 206, 222));
        window_.draw(row);
    }

    sf::Text hint(font_, "Up/Down: select   Left/Right/Enter: adjust   Esc: close", 14);
    hint.setPosition({panel_pos.x + 20.0f, panel_pos.y + panel_size.y - 28.0f});
    hint.setFillColor(sf::Color(174, 196, 214));
    window_.draw(hint);
}

void Application::draw_help_overlay() {
    sf::RectangleShape dim({static_cast<float>(window_.getSize().x), static_cast<float>(window_.getSize().y)});
    dim.setFillColor(sf::Color(4, 8, 14, 192));
    window_.draw(dim);

    const sf::Vector2f panel_size(760.0f, 430.0f);
    const sf::Vector2f panel_pos(
        0.5f * (static_cast<float>(window_.getSize().x) - panel_size.x),
        0.5f * (static_cast<float>(window_.getSize().y) - panel_size.y));

    sf::RectangleShape panel(panel_size);
    panel.setPosition(panel_pos);
    panel.setFillColor(sf::Color(16, 24, 36, 238));
    panel.setOutlineThickness(2.0f);
    panel.setOutlineColor(sf::Color(108, 150, 188, 236));
    window_.draw(panel);

    sf::Text title(font_, "Controls", 24);
    title.setPosition({panel_pos.x + 20.0f, panel_pos.y + 14.0f});
    title.setFillColor(sf::Color(214, 234, 248));
    window_.draw(title);

    const std::string body =
        "Primary\n"
        "Top bar : File / Options / PDE menus\n"
        "L-click left panel  : place source\n"
        "L-click right panel : place sensor\n"
        "E       : run source estimation (paused)\n"
        "J       : toggle adjoint overlay\n"
        "Space   : pause/resume\n"
        "R       : reset simulation\n"
        "Esc     : open/close preferences\n"
        "F1      : open/close this controls window\n"
        "\n"
        "Models\n"
        "W       : cycle wind field model\n"
        "K       : cycle SDE diffusivity model\n"
        "P       : cycle PDE diffusion mode\n"
        "C       : toggle PDE color scale mode\n"
        "B       : cycle boundary condition\n"
        "H       : toggle Brownian/Heat special case\n"
        "U       : toggle mass units\n"
        "\n"
        "Time\n"
        "[       : decrease simulation speed\n"
        "]       : increase simulation speed\n"
        "\\       : reset speed to x1";

    sf::Text text(font_, body, 18);
    text.setPosition({panel_pos.x + 24.0f, panel_pos.y + 56.0f});
    text.setLineSpacing(1.1f);
    text.setFillColor(sf::Color(188, 208, 226));
    window_.draw(text);

    sf::Text hint(font_, "Press F1 to close", 14);
    hint.setPosition({panel_pos.x + panel_size.x - 140.0f, panel_pos.y + panel_size.y - 28.0f});
    hint.setFillColor(sf::Color(170, 194, 214));
    window_.draw(hint);
}

void Application::apply_menu_adjustment(int direction) {
    switch (menu_index_) {
    case 0:
        sim().scale_time(direction > 0 ? 1.25f : 0.8f);
        break;
    case 1:
        sim().cycle_wind_model(direction >= 0 ? 1 : -1);
        break;
    case 2:
        sim().cycle_diffusion_model(direction >= 0 ? 1 : -1);
        break;
    case 3:
        sim().cycle_pde_diffusion_mode(direction >= 0 ? 1 : -1);
        break;
    case 4:
        sim().toggle_boundary_mode();
        break;
    case 5:
        show_wind_ = !show_wind_;
        break;
    case 6:
        sim().toggle_brownian_heat_case();
        break;
    case 7:
        mass_unit_ = (mass_unit_ == MassDisplayUnit::MicrogramsPerSquareMeter)
            ? MassDisplayUnit::GramsPerSquareMeter
            : MassDisplayUnit::MicrogramsPerSquareMeter;
        break;
    case 8:
        pde_auto_color_scale_ = !pde_auto_color_scale_;
        break;
    case 9:
        sim().adjust_trail_length(direction > 0 ? 1 : -1);
        break;
    case 10:
        sim().reset();
        sensor_manager_.clear();
        clear_source_estimation();
        break;
    default:
        break;
    }
}

void Application::draw_pde_heatmap() {
    const auto& pde = sim().pde();
    const auto& c = pde.concentration();
    if (c.empty() || pde.nx() < 2 || pde.ny() < 2) {
        return;
    }

    const float max_c = pde.max_concentration();
    if (pde_auto_color_scale_) {
        const float target = std::max(1.0e-9f, 0.15f * max_c);
        pde_color_scale_runtime_ = 0.92f * pde_color_scale_runtime_ + 0.08f * target;
    } else {
        pde_color_scale_runtime_ = pde_fixed_color_scale_;
    }
    const float denom = std::log1p(std::max(pde_color_scale_runtime_, 1.0e-10f));
    
    const int nx = pde.nx();
    const int ny = pde.ny();
    const float sx = right_panel_.size.x / static_cast<float>(nx - 1);
    const float sy = right_panel_.size.y / static_cast<float>(ny - 1);

    sf::VertexArray mesh(sf::PrimitiveType::Triangles, static_cast<std::size_t>((nx - 1) * (ny - 1) * 6));
    std::size_t vi = 0;
    for (int j = 0; j < ny - 1; ++j) {
        for (int i = 0; i < nx - 1; ++i) {
            auto idx = [nx](int ii, int jj) { return jj * nx + ii; };
            const float c00 = c[idx(i, j)];
            const float c10 = c[idx(i + 1, j)];
            const float c01 = c[idx(i, j + 1)];
            const float c11 = c[idx(i + 1, j + 1)];

            const float n00 = std::clamp(std::log1p(c00) / denom, 0.0f, 1.0f);
            const float n10 = std::clamp(std::log1p(c10) / denom, 0.0f, 1.0f);
            const float n01 = std::clamp(std::log1p(c01) / denom, 0.0f, 1.0f);
            const float n11 = std::clamp(std::log1p(c11) / denom, 0.0f, 1.0f);

            const sf::Vector2f p00(right_panel_.position.x + i * sx, right_panel_.position.y + j * sy);
            const sf::Vector2f p10(right_panel_.position.x + (i + 1) * sx, right_panel_.position.y + j * sy);
            const sf::Vector2f p01(right_panel_.position.x + i * sx, right_panel_.position.y + (j + 1) * sy);
            const sf::Vector2f p11(
                right_panel_.position.x + (i + 1) * sx, right_panel_.position.y + (j + 1) * sy);

            mesh[vi].position = p00;
            mesh[vi].color = heat_color(n00);
            ++vi;
            mesh[vi].position = p10;
            mesh[vi].color = heat_color(n10);
            ++vi;
            mesh[vi].position = p11;
            mesh[vi].color = heat_color(n11);
            ++vi;

            mesh[vi].position = p00;
            mesh[vi].color = heat_color(n00);
            ++vi;
            mesh[vi].position = p11;
            mesh[vi].color = heat_color(n11);
            ++vi;
            mesh[vi].position = p01;
            mesh[vi].color = heat_color(n01);
            ++vi;
        }
    }
    window_.draw(mesh);
}

void Application::draw_sensor_overlay() {
    const auto& sensors = sensor_manager_.sensors();
    if (sensors.empty()) {
        return;
    }

    sf::CircleShape marker(3.5f);
    marker.setOrigin({3.5f, 3.5f});
    marker.setFillColor(sf::Color(252, 230, 128, 220));
    marker.setOutlineThickness(1.0f);
    marker.setOutlineColor(sf::Color(20, 26, 32, 240));

    int sensor_id = 1;
    for (const auto& sensor : sensors) {
        const sf::Vector2f p = domain_to_right_panel(sensor.position);
        marker.setPosition(p);
        window_.draw(marker);

        std::ostringstream ss;
        ss << "S" << sensor_id << " (" << std::fixed << std::setprecision(0) << sensor.position.x() << ","
           << sensor.position.y() << ") ";
        if (!sensor.history.empty()) {
            const auto& obs = sensor.history.back();
            ss << "c=" << std::scientific << std::setprecision(2) << obs.noisy_concentration;
        } else {
            ss << "c=n/a";
        }

        sf::Text label(font_, ss.str(), 11);
        const float max_x = right_panel_.position.x + right_panel_.size.x - 180.0f;
        const float label_x = std::clamp(p.x + 7.0f, right_panel_.position.x + 2.0f, max_x);
        const float label_y = std::max(right_panel_.position.y + 2.0f, p.y - 13.0f);
        label.setPosition({label_x, label_y});
        label.setFillColor(sf::Color(255, 250, 204, 235));
        window_.draw(label);
        ++sensor_id;
    }
}

void Application::draw_adjoint_overlay() {
    if (!show_adjoint_overlay_ || !source_estimation_.has_result || source_estimation_.p_star.empty()
        || source_estimation_.nx < 2 || source_estimation_.ny < 2) {
        return;
    }

    const int nx = source_estimation_.nx;
    const int ny = source_estimation_.ny;
    const auto& p = source_estimation_.p_star;
    float pmax = 0.0f;
    for (float v : p) {
        pmax = std::max(pmax, v);
    }
    if (pmax <= 1.0e-16f) {
        return;
    }

    const float sx = right_panel_.size.x / static_cast<float>(nx - 1);
    const float sy = right_panel_.size.y / static_cast<float>(ny - 1);
    sf::VertexArray mesh(sf::PrimitiveType::Triangles, static_cast<std::size_t>((nx - 1) * (ny - 1) * 6));
    std::size_t vi = 0;
    auto i2 = [nx](int i, int j) { return static_cast<std::size_t>(j * nx + i); };

    for (int j = 0; j < ny - 1; ++j) {
        for (int i = 0; i < nx - 1; ++i) {
            const float n00 = std::clamp(p[i2(i, j)] / pmax, 0.0f, 1.0f);
            const float n10 = std::clamp(p[i2(i + 1, j)] / pmax, 0.0f, 1.0f);
            const float n01 = std::clamp(p[i2(i, j + 1)] / pmax, 0.0f, 1.0f);
            const float n11 = std::clamp(p[i2(i + 1, j + 1)] / pmax, 0.0f, 1.0f);

            auto c = [](float t) {
                return sf::Color(
                    static_cast<std::uint8_t>(255.0f * std::clamp(t, 0.0f, 1.0f)),
                    static_cast<std::uint8_t>(180.0f * std::clamp(1.0f - t, 0.0f, 1.0f)),
                    80,
                    static_cast<std::uint8_t>(120.0f * std::clamp(t, 0.0f, 1.0f)));
            };

            const sf::Vector2f p00(right_panel_.position.x + i * sx, right_panel_.position.y + j * sy);
            const sf::Vector2f p10(right_panel_.position.x + (i + 1) * sx, right_panel_.position.y + j * sy);
            const sf::Vector2f p01(right_panel_.position.x + i * sx, right_panel_.position.y + (j + 1) * sy);
            const sf::Vector2f p11(
                right_panel_.position.x + (i + 1) * sx, right_panel_.position.y + (j + 1) * sy);

            mesh[vi].position = p00;
            mesh[vi].color = c(n00);
            ++vi;
            mesh[vi].position = p10;
            mesh[vi].color = c(n10);
            ++vi;
            mesh[vi].position = p11;
            mesh[vi].color = c(n11);
            ++vi;
            mesh[vi].position = p00;
            mesh[vi].color = c(n00);
            ++vi;
            mesh[vi].position = p11;
            mesh[vi].color = c(n11);
            ++vi;
            mesh[vi].position = p01;
            mesh[vi].color = c(n01);
            ++vi;
        }
    }
    window_.draw(mesh);
}

void Application::run_source_estimation() {
    if (!sim().paused()) {
        source_estimation_.status = "Pause simulation before running estimation.";
        return;
    }

    const auto result = source_estimator_.estimate(sensor_manager_.sensors(), sim());
    source_estimation_.status = result.message;
    if (!result.success) {
        source_estimation_.has_result = false;
        return;
    }

    source_estimation_.has_result = true;
    source_estimation_.x_star = result.x_star;
    source_estimation_.t_star_s = result.t_star_s;
    source_estimation_.nx = result.nx;
    source_estimation_.ny = result.ny;
    source_estimation_.p_star = result.p_star;
}

void Application::clear_source_estimation() {
    source_estimation_ = SourceEstimationView{};
}

Simulator& Application::sim() {
    return controller_.simulator();
}

const Simulator& Application::sim() const {
    return controller_.simulator();
}

} // namespace atm
