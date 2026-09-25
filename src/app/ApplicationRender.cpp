#include "app/Application.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
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

const char* mass_unit_label(MassDisplayUnit unit) {
    return unit == MassDisplayUnit::MicrogramsPerSquareMeter ? "ug/m^2" : "g/m^2";
}

float concentration_display_factor_ug_per_m3(float ug_per_m2_scale, float mixing_height_m) {
    return ug_per_m2_scale / std::max(1.0e-6f, mixing_height_m);
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

sf::Color adjoint_color(float t) {
    const float u = std::clamp(t, 0.0f, 1.0f);
    return sf::Color(
        static_cast<std::uint8_t>(255.0f * u),
        static_cast<std::uint8_t>(220.0f * (1.0f - u)),
        80,
        static_cast<std::uint8_t>(40.0f + 200.0f * u));
}

sf::Vector2f centered_position(const sf::RenderWindow& window, const sf::Vector2f& panel_size) {
    return sf::Vector2f(
        0.5f * (static_cast<float>(window.getSize().x) - panel_size.x),
        0.5f * (static_cast<float>(window.getSize().y) - panel_size.y));
}

sf::Vector2f draw_modal_panel(
    sf::RenderWindow& window, const sf::Vector2f& panel_size, const sf::Color& dim_color, const sf::Color& panel_color,
    const sf::Color& outline_color) {
    sf::RectangleShape dim({static_cast<float>(window.getSize().x), static_cast<float>(window.getSize().y)});
    dim.setFillColor(dim_color);
    window.draw(dim);

    const sf::Vector2f panel_pos = centered_position(window, panel_size);
    sf::RectangleShape panel(panel_size);
    panel.setPosition(panel_pos);
    panel.setFillColor(panel_color);
    panel.setOutlineThickness(2.0f);
    panel.setOutlineColor(outline_color);
    window.draw(panel);
    return panel_pos;
}

template <typename SampleFn, typename ColorFn>
sf::VertexArray make_scalar_field_mesh(
    const sf::FloatRect& panel, int nx, int ny, const SampleFn& sample_normalized, const ColorFn& color_map) {
    const float sx = panel.size.x / static_cast<float>(nx - 1);
    const float sy = panel.size.y / static_cast<float>(ny - 1);
    sf::VertexArray mesh(sf::PrimitiveType::Triangles, static_cast<std::size_t>((nx - 1) * (ny - 1) * 6));

    std::size_t vi = 0;
    for (int j = 0; j < ny - 1; ++j) {
        for (int i = 0; i < nx - 1; ++i) {
            const float n00 = std::clamp(sample_normalized(i, j), 0.0f, 1.0f);
            const float n10 = std::clamp(sample_normalized(i + 1, j), 0.0f, 1.0f);
            const float n01 = std::clamp(sample_normalized(i, j + 1), 0.0f, 1.0f);
            const float n11 = std::clamp(sample_normalized(i + 1, j + 1), 0.0f, 1.0f);

            const sf::Vector2f p00(panel.position.x + i * sx, panel.position.y + j * sy);
            const sf::Vector2f p10(panel.position.x + (i + 1) * sx, panel.position.y + j * sy);
            const sf::Vector2f p01(panel.position.x + i * sx, panel.position.y + (j + 1) * sy);
            const sf::Vector2f p11(panel.position.x + (i + 1) * sx, panel.position.y + (j + 1) * sy);

            mesh[vi].position = p00;
            mesh[vi].color = color_map(n00);
            ++vi;
            mesh[vi].position = p10;
            mesh[vi].color = color_map(n10);
            ++vi;
            mesh[vi].position = p11;
            mesh[vi].color = color_map(n11);
            ++vi;

            mesh[vi].position = p00;
            mesh[vi].color = color_map(n00);
            ++vi;
            mesh[vi].position = p11;
            mesh[vi].color = color_map(n11);
            ++vi;
            mesh[vi].position = p01;
            mesh[vi].color = color_map(n01);
            ++vi;
        }
    }
    return mesh;
}

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

    sf::Text right_label(font_, "PDE Concentration Heatmap (ug/m^3)", 16);
    right_label.setPosition({right_panel_.position.x + 10.0f, right_panel_.position.y + 8.0f});
    right_label.setFillColor(sf::Color(185, 185, 185));
    window_.draw(right_label);

    if (!toolbar_open) {
        draw_hud_cards();
        draw_control_strip();
        ecs_ui_scene_.draw(window_, font_, ecs_theme_);
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
    top_toolbar_.draw(window_, font_, menu_model_, data_recorder_.is_recording());
    top_toolbar_.draw_active_menu(window_, font_, menu_model_, data_recorder_.is_recording());
    window_.display();
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

EstimateButtonLayout Application::estimate_button_layout() const {
    const float top = 38.0f;
    const float left = 24.0f;
    const float gap = 12.0f;
    const float card_w = 250.0f;
    const float card_h = 180.0f;

    EstimateButtonLayout layout;
    layout.card = sf::FloatRect({left + 2.0f * (card_w + gap), top}, {card_w + 30.0f, card_h});
    layout.prev_button
        = sf::FloatRect({layout.card.position.x + 10.0f, layout.card.position.y + 52.0f}, {36.0f, 26.0f});
    layout.next_button
        = sf::FloatRect({layout.card.position.x + layout.card.size.x - 46.0f, layout.card.position.y + 52.0f}, {36.0f, 26.0f});
    layout.run_button = sf::FloatRect(
        {layout.card.position.x + 10.0f, layout.card.position.y + 86.0f}, {layout.card.size.x - 20.0f, 38.0f});
    return layout;
}

void Application::draw_hud_cards() {
    const auto& source = sim().source();
    const auto& particles = sim().particles();
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
    const float c_factor = concentration_display_factor_ug_per_m3(concentration_scale_ug_per_m2_, mixing_height_m_);
    const float pde_max_ug_m3 = sim().pde().max_concentration() * c_factor;
    status << std::fixed << std::setprecision(2)
           << "t: " << sim().time_s() << " s"
           << "\nspeed: x" << sim().time_scale()
           << "\nstate: " << (sim().paused() ? "paused" : "running")
           << "\nwind: " << sim().wind_model_name()
           << "\ndiff: " << sim().diffusion_model_name()
           << "\nPDE diff: " << sim().pde_diffusion_mode_name()
           << "\nBC: " << boundary_mode_label(sim().boundary_mode())
           << "\nPDE max c: " << std::scientific << std::setprecision(2) << pde_max_ug_m3 << " ug/m^3";

    std::ostringstream source_text;
    const float emitted_per_step = sim().last_emission_rate_per_second() * sim().config().numerics.dt;
    source_text << std::scientific << std::setprecision(2)
                << "q_total(t): " << source.emission_rate()
                << "\nemitted: " << emitted_per_step << " / step"
                << "\nsources: " << source.active_count() << " / " << source.max_sources()
                << "\nparticles: " << particles.particles().size()
                << "\nnewest: "
                << (source.is_active()
                        ? (std::to_string(static_cast<int>(source.newest_age_s())) + "s / "
                            + std::to_string(static_cast<int>(source.lifespan_s())) + "s")
                        : std::string("inactive"))
                << "\nM_sde/M_pde: " << std::fixed << std::setprecision(3) << ratio << " (" << mass_unit_label(mass_unit_)
                << ")";

    const float top = 38.0f;
    const float left = 24.0f;
    const float gap = 12.0f;
    const float card_w = 250.0f;
    const float card_h = 180.0f;
    draw_card(left, top, card_w, card_h, "Status", status.str());
    draw_card(left + card_w + gap, top, card_w, card_h, "Source", source_text.str());

    // --- Estimate card: a first-class button, not just a File-menu entry
    // or a keyboard shortcut, since running a source estimate is the whole
    // point of placing sensors.
    const EstimateButtonLayout est = estimate_button_layout();
    {
        sf::RectangleShape card({est.card.size.x, est.card.size.y});
        card.setPosition(est.card.position);
        card.setFillColor(sf::Color(12, 18, 28, 190));
        card.setOutlineThickness(1.0f);
        card.setOutlineColor(sf::Color(72, 112, 150, 190));
        window_.draw(card);

        sf::Text title(font_, "Estimate", 14);
        title.setPosition({est.card.position.x + 10.0f, est.card.position.y + 8.0f});
        title.setFillColor(sf::Color(186, 220, 238));
        window_.draw(title);

        const bool has_methods = !estimation_results_.empty();
        std::ostringstream method_line;
        if (has_methods) {
            const auto& r = estimation_results_[static_cast<std::size_t>(selected_estimation_method_)];
            method_line << "[" << (selected_estimation_method_ + 1) << "/" << estimation_results_.size() << "] "
                        << source_estimation_method_name(r.method);
        } else {
            method_line << "No estimate run yet";
        }
        sf::Text method_text(font_, method_line.str(), 12);
        method_text.setPosition({est.card.position.x + 10.0f, est.card.position.y + 30.0f});
        method_text.setFillColor(sf::Color(216, 228, 238));
        window_.draw(method_text);

        auto draw_button = [&](const sf::FloatRect& rect, const std::string& text, bool emphasized) {
            sf::RectangleShape b(rect.size);
            b.setPosition(rect.position);
            b.setFillColor(emphasized ? sf::Color(52, 116, 78, 245) : sf::Color(52, 76, 102, 240));
            b.setOutlineThickness(1.0f);
            b.setOutlineColor(emphasized ? sf::Color(108, 170, 126, 235) : sf::Color(112, 148, 184, 235));
            window_.draw(b);

            sf::Text label(font_, text, 13);
            const auto bounds = label.getLocalBounds();
            label.setPosition(
                {rect.position.x + 0.5f * (rect.size.x - bounds.size.x) - bounds.position.x,
                    rect.position.y + 0.5f * (rect.size.y - bounds.size.y) - bounds.position.y});
            label.setFillColor(sf::Color(230, 241, 252));
            window_.draw(label);
        };

        draw_button(est.prev_button, "<", false);
        draw_button(est.next_button, ">", false);
        draw_button(est.run_button, sim().paused() ? "RUN ESTIMATE (E)" : "PAUSE FIRST", sim().paused());

        std::ostringstream result_line;
        if (has_methods) {
            const auto& r = estimation_results_[static_cast<std::size_t>(selected_estimation_method_)];
            if (r.success && !r.insufficient_signal) {
                result_line << std::fixed << std::setprecision(0) << "x*=(" << r.x_star.x() << ", " << r.x_star.y()
                            << ")\nt*=" << r.t_star_s << " s";
                if (r.method != SourceEstimationMethod::AdjointBacktracking) {
                    result_line << std::setprecision(2) << "  q0=" << r.q0;
                }
            } else {
                result_line << r.message;
            }
        }
        sf::Text result_text(font_, result_line.str(), 12);
        result_text.setPosition({est.card.position.x + 10.0f, est.card.position.y + 132.0f});
        result_text.setLineSpacing(0.95f);
        result_text.setFillColor(sf::Color(216, 228, 238));
        window_.draw(result_text);
    }
}

void Application::draw_control_strip() {
    sf::RectangleShape strip({760.0f, 60.0f});
    strip.setPosition({24.0f, 226.0f});
    strip.setFillColor(sf::Color(10, 16, 24, 188));
    strip.setOutlineThickness(1.0f);
    strip.setOutlineColor(sf::Color(70, 108, 145, 180));
    window_.draw(strip);

    sf::Text control_text(
        font_,
        "Toolbar: File | Source | Numerics | Sensors | Display | PDE | E estimate | M cycle method | L load NY sites | N NY preset | F1 help",
        13);
    control_text.setPosition({34.0f, 230.0f});
    control_text.setFillColor(sf::Color(168, 192, 210));
    window_.draw(control_text);

    const std::string estimator_status = source_estimation_.status.empty()
        ? "Estimator: idle"
        : ("Estimator: " + source_estimation_.status);
    std::ostringstream line2;
    line2 << estimator_status;
    if (source_estimation_.has_error_m) {
        line2 << " | error=" << std::fixed << std::setprecision(1) << source_estimation_.error_m << " m";
    }
    if (!recording_status_.empty()) {
        line2 << "  |  Rec: " << recording_status_;
    }
    sf::Text line2_text(font_, line2.str(), 12);
    line2_text.setPosition({34.0f, 247.0f});
    line2_text.setFillColor(sf::Color(196, 216, 232));
    window_.draw(line2_text);

    if (!sites_status_.empty()) {
        sf::Text line3_text(font_, sites_status_, 12);
        line3_text.setPosition({34.0f, 262.0f});
        line3_text.setFillColor(sf::Color(180, 230, 190));
        window_.draw(line3_text);
    }
}


void Application::draw_menu_overlay() {
    const sf::Vector2f panel_size(620.0f, 620.0f);
    const sf::Vector2f panel_pos = draw_modal_panel(
        window_, panel_size, sf::Color(6, 10, 16, 170), sf::Color(20, 28, 40, 235), sf::Color(110, 150, 186, 240));

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
    const sf::Vector2f panel_size(760.0f, 430.0f);
    const sf::Vector2f panel_pos = draw_modal_panel(
        window_, panel_size, sf::Color(4, 8, 14, 192), sf::Color(16, 24, 36, 238), sf::Color(108, 150, 188, 236));

    sf::Text title(font_, "Controls", 24);
    title.setPosition({panel_pos.x + 20.0f, panel_pos.y + 14.0f});
    title.setFillColor(sf::Color(214, 234, 248));
    window_.draw(title);

    const std::string body =
        "Primary\n"
        "Top bar : File / Source / Numerics / Sensors / Display / PDE\n"
        "L-click left panel  : place source\n"
        "L-click right panel : place sensor\n"
        "E       : run source estimation (paused)\n"
        "J       : toggle adjoint overlay\n"
        "L       : load NY wildfire sensor network (clears current sensors)\n"
        "N       : apply NY observation preset (phys 300 s, avg 3600 s)\n"
        "Space   : pause/resume\n"
        "R       : reset simulation (clears sensors)\n"
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
    const auto idx = [nx](int ii, int jj) { return jj * nx + ii; };
    const auto sample = [&](int i, int j) { return std::log1p(c[idx(i, j)]) / denom; };
    const sf::VertexArray mesh
        = make_scalar_field_mesh(right_panel_, nx, ny, sample, [](float n) { return heat_color(n); });
    window_.draw(mesh);
}

void Application::draw_sensor_overlay() {
    const auto& sensors = sensor_manager_.sensors();
    if (sensors.empty()) {
        return;
    }
    const float c_factor = concentration_display_factor_ug_per_m3(concentration_scale_ug_per_m2_, mixing_height_m_);

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
        ss << "S" << sensor_id << ": ";
        if (sensor.history.empty()) {
            ss << "N/A";
        } else {
            const float noisy_ug_m3 = sensor.history.back().noisy_concentration * c_factor;
            ss << std::scientific << std::setprecision(2) << noisy_ug_m3 << " ug/m^3";
        }
        sf::Text label(font_, ss.str(), 11);
        const sf::FloatRect bounds = label.getLocalBounds();
        const float min_x = right_panel_.position.x + 2.0f;
        const float max_x = right_panel_.position.x + right_panel_.size.x - bounds.size.x - 6.0f;
        const float min_y = right_panel_.position.y + 2.0f;
        const float max_y = right_panel_.position.y + right_panel_.size.y - bounds.size.y - 6.0f;
        const float label_x = std::clamp(p.x + 7.0f, min_x, max_x);
        const float label_y = std::clamp(p.y - 13.0f, min_y, max_y);
        label.setPosition({label_x, label_y});
        label.setFillColor(sf::Color(255, 250, 204, 235));
        window_.draw(label);
        ++sensor_id;
    }
}

void Application::draw_adjoint_overlay() {
    if (!show_adjoint_overlay_) {
        return;
    }

    if (feynman_kac_anim_.active && !feynman_kac_anim_.solution.snapshots.empty()) {
        const int nx = feynman_kac_anim_.solution.nx;
        const int ny = feynman_kac_anim_.solution.ny;
        if (nx < 2 || ny < 2) {
            return;
        }
        const int frame = std::clamp(
            feynman_kac_anim_.frame, 0, static_cast<int>(feynman_kac_anim_.solution.snapshots.size()) - 1);
        const auto& phi = feynman_kac_anim_.solution.snapshots[static_cast<std::size_t>(frame)].phi;
        float pmax = 0.0f;
        for (float v : phi) {
            pmax = std::max(pmax, v);
        }
        if (pmax <= 1.0e-16f) {
            return;
        }
        auto i2 = [nx](int i, int j) { return static_cast<std::size_t>(j * nx + i); };
        constexpr float OVERLAY_GAIN = 2.25f;
        const auto sample = [&](int i, int j) { return OVERLAY_GAIN * (phi[i2(i, j)] / pmax); };
        const sf::VertexArray mesh
            = make_scalar_field_mesh(right_panel_, nx, ny, sample, [](float n) { return adjoint_color(n); });
        window_.draw(mesh);
        return;
    }

    if (!source_estimation_.has_result || source_estimation_.p_star.empty()
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

    auto i2 = [nx](int i, int j) { return static_cast<std::size_t>(j * nx + i); };
    constexpr float OVERLAY_GAIN = 2.25f;
    const auto sample = [&](int i, int j) { return OVERLAY_GAIN * (p[i2(i, j)] / pmax); };
    const sf::VertexArray mesh
        = make_scalar_field_mesh(right_panel_, nx, ny, sample, [](float n) { return adjoint_color(n); });
    window_.draw(mesh);
}

} // namespace atm
