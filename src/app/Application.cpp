#include "app/Application.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace atm {

Application::Application(const Config& config)
    : controller_(config)
    , menu_model_(controller_.current_settings())
    , sensor_manager_(
          config.app.sensor_sample_period_s,
          config.app.sensor_noise_std,
          config.app.sensor_history_capacity,
          config.app.sensor_physical_sample_period_s,
          config.app.sensor_spatial_avg_radius_m)
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
    ecs_theme_ = ui::make_default_retro_theme();
    build_ecs_ui();

    // Georeference the domain to the NY network from the start (when the
    // dataset is present), so hand-placed and L-loaded sensors export in the
    // same lat/lon frame and Datetime_UTC always anchors to the real event.
    std::string georef_err;
    sites_status_ = load_ny_georeference(nullptr, nullptr, georef_err)
        ? "Georeferenced to NY network: exports use lat/lon"
        : "No NY georeference (" + georef_err + "): exports use x/y";
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
    // Sensors sample after every fixed step, so at large speed-ups each
    // physical sample still sees the field at (within dt of) its own time.
    sim().step(frame_dt, [this] { sensor_manager_.step(sim().time_s(), sim().pde(), sim().config().domain); });

    if (data_recorder_.is_recording()) {
        const auto& sensors = sensor_manager_.sensors();
        for (const auto& report : sensor_manager_.pending_reports()) {
            if (report.sensor_index < sensors.size()) {
                const auto& sensor = sensors[report.sensor_index];
                const Vec2 wind = sim().wind_at_time(report.time_s, sensor.position);
                data_recorder_.add_reading(
                    report.sensor_index, sensor.position, sensor.label, report.time_s,
                    report.noisy_concentration,
                    concentration_scale_ug_per_m2_, mixing_height_m_, wind);
            }
        }
    }
    sensor_manager_.clear_pending_reports();

    if (feynman_kac_anim_.active && sim().paused() && !feynman_kac_anim_.solution.snapshots.empty()) {
        feynman_kac_anim_.wall_accum_s += frame_dt;
        while (feynman_kac_anim_.wall_accum_s >= feynman_kac_anim_.wall_time_per_frame_s) {
            feynman_kac_anim_.wall_accum_s -= feynman_kac_anim_.wall_time_per_frame_s;
            ++feynman_kac_anim_.frame;
            if (feynman_kac_anim_.frame >= static_cast<int>(feynman_kac_anim_.solution.snapshots.size())) {
                feynman_kac_anim_.frame = 0;
            }
        }
    }

    sync_ecs_ui_state();
}

void Application::build_ecs_ui() {
    ecs_ui_scene_.clear();
    if (!show_ecs_quick_panel_) {
        return;
    }

    const sf::Vector2u window_size = window_.getSize();
    const float panel_w = 250.0f;
    const float panel_h = 196.0f;
    const float panel_x = static_cast<float>(window_size.x) - panel_w - 22.0f;
    const float panel_y = 42.0f;

    auto& registry = ecs_ui_scene_.registry();
    ui::add_panel(registry, sf::FloatRect({panel_x, panel_y}, {panel_w, panel_h}), 200, ecs_theme_.panel_face);
    ecs_panel_title_ = ui::add_label(
        registry,
        sf::FloatRect({panel_x + 8.0f, panel_y + 6.0f}, {panel_w - 16.0f, 20.0f}),
        201,
        "Quick Actions",
        sf::Vector2f(0.0f, 0.0f),
        14,
        ecs_theme_.text_primary);

    const float button_x = panel_x + 12.0f;
    const float button_w = panel_w - 24.0f;
    ecs_pause_button_ = ui::add_button(
        registry,
        sf::FloatRect({button_x, panel_y + 34.0f}, {button_w, 28.0f}),
        202,
        "Pause",
        "toggle_pause",
        ecs_theme_);
    ecs_reset_button_ = ui::add_button(
        registry,
        sf::FloatRect({button_x, panel_y + 68.0f}, {button_w, 28.0f}),
        202,
        "Reset",
        "reset_simulation",
        ecs_theme_);
    ecs_estimate_button_ = ui::add_button(
        registry,
        sf::FloatRect({button_x, panel_y + 124.0f}, {button_w, 28.0f}),
        202,
        "Estimate Source",
        "estimate_source",
        ecs_theme_);

    const float pde_x = panel_x - 352.0f;
    const float pde_y = 42.0f;
    const float pde_w = 336.0f;
    const float pde_h = 196.0f;
    ui::add_panel(registry, sf::FloatRect({pde_x, pde_y}, {pde_w, pde_h}), 200, ecs_theme_.panel_face);
    ecs_pde_title_ = ui::add_label(
        registry,
        sf::FloatRect({pde_x + 8.0f, pde_y + 6.0f}, {pde_w - 16.0f, 20.0f}),
        201,
        "PDE Settings",
        sf::Vector2f(0.0f, 0.0f),
        14,
        ecs_theme_.text_primary);

    const float row_start = pde_y + 36.0f;
    const float row_h = 30.0f;
    const float label_x = pde_x + 14.0f;
    const float value_x = pde_x + 130.0f;
    const float minus_x = pde_x + pde_w - 64.0f;
    const float plus_x = pde_x + pde_w - 36.0f;

    ui::add_label(
        registry,
        sf::FloatRect({label_x, row_start + row_h * 0.0f}, {100.0f, 18.0f}),
        201,
        "Grid Nx",
        sf::Vector2f(0.0f, 0.0f),
        13,
        ecs_theme_.text_primary);
    ecs_pde_nx_value_ = ui::add_label(
        registry,
        sf::FloatRect({value_x, row_start + row_h * 0.0f}, {110.0f, 18.0f}),
        201,
        "-",
        sf::Vector2f(0.0f, 0.0f),
        13,
        ecs_theme_.accent);
    ui::add_button(
        registry, sf::FloatRect({minus_x, row_start + row_h * 0.0f - 3.0f}, {22.0f, 22.0f}), 202, "-", "pde_nx_dec",
        ecs_theme_);
    ui::add_button(
        registry, sf::FloatRect({plus_x, row_start + row_h * 0.0f - 3.0f}, {22.0f, 22.0f}), 202, "+", "pde_nx_inc",
        ecs_theme_);

    ui::add_label(
        registry,
        sf::FloatRect({label_x, row_start + row_h * 1.0f}, {100.0f, 18.0f}),
        201,
        "Grid Ny",
        sf::Vector2f(0.0f, 0.0f),
        13,
        ecs_theme_.text_primary);
    ecs_pde_ny_value_ = ui::add_label(
        registry,
        sf::FloatRect({value_x, row_start + row_h * 1.0f}, {110.0f, 18.0f}),
        201,
        "-",
        sf::Vector2f(0.0f, 0.0f),
        13,
        ecs_theme_.accent);
    ui::add_button(
        registry, sf::FloatRect({minus_x, row_start + row_h * 1.0f - 3.0f}, {22.0f, 22.0f}), 202, "-", "pde_ny_dec",
        ecs_theme_);
    ui::add_button(
        registry, sf::FloatRect({plus_x, row_start + row_h * 1.0f - 3.0f}, {22.0f, 22.0f}), 202, "+", "pde_ny_inc",
        ecs_theme_);

    ui::add_label(
        registry,
        sf::FloatRect({label_x, row_start + row_h * 2.0f}, {100.0f, 18.0f}),
        201,
        "dt",
        sf::Vector2f(0.0f, 0.0f),
        13,
        ecs_theme_.text_primary);
    ecs_pde_dt_value_ = ui::add_label(
        registry,
        sf::FloatRect({value_x, row_start + row_h * 2.0f}, {110.0f, 18.0f}),
        201,
        "-",
        sf::Vector2f(0.0f, 0.0f),
        13,
        ecs_theme_.accent);
    ui::add_button(
        registry, sf::FloatRect({minus_x, row_start + row_h * 2.0f - 3.0f}, {22.0f, 22.0f}), 202, "-", "pde_dt_dec",
        ecs_theme_);
    ui::add_button(
        registry, sf::FloatRect({plus_x, row_start + row_h * 2.0f - 3.0f}, {22.0f, 22.0f}), 202, "+", "pde_dt_inc",
        ecs_theme_);

    ui::add_label(
        registry,
        sf::FloatRect({label_x, row_start + row_h * 3.0f}, {100.0f, 18.0f}),
        201,
        "Diffusion",
        sf::Vector2f(0.0f, 0.0f),
        13,
        ecs_theme_.text_primary);
    ecs_pde_mode_value_ = ui::add_label(
        registry,
        sf::FloatRect({value_x, row_start + row_h * 3.0f}, {130.0f, 18.0f}),
        201,
        "-",
        sf::Vector2f(0.0f, 0.0f),
        13,
        ecs_theme_.accent);
    ui::add_button(
        registry,
        sf::FloatRect({minus_x, row_start + row_h * 3.0f - 3.0f}, {22.0f, 22.0f}),
        202,
        "<",
        "pde_mode_prev",
        ecs_theme_);
    ui::add_button(
        registry,
        sf::FloatRect({plus_x, row_start + row_h * 3.0f - 3.0f}, {22.0f, 22.0f}),
        202,
        ">",
        "pde_mode_next",
        ecs_theme_);

    ui::add_button(
        registry,
        sf::FloatRect({pde_x + 14.0f, pde_y + pde_h - 32.0f}, {88.0f, 24.0f}),
        202,
        "Apply",
        "pde_apply",
        ecs_theme_);
    ui::add_button(
        registry,
        sf::FloatRect({pde_x + 108.0f, pde_y + pde_h - 32.0f}, {88.0f, 24.0f}),
        202,
        "Revert",
        "pde_revert",
        ecs_theme_);

    const float opt_x = pde_x - 352.0f;
    const float opt_y = 42.0f;
    const float opt_w = 336.0f;
    const float opt_h = 196.0f;
    ui::add_panel(registry, sf::FloatRect({opt_x, opt_y}, {opt_w, opt_h}), 200, ecs_theme_.panel_face);
    ecs_opt_title_ = ui::add_label(
        registry,
        sf::FloatRect({opt_x + 8.0f, opt_y + 6.0f}, {opt_w - 16.0f, 20.0f}),
        201,
        "Runtime Options",
        sf::Vector2f(0.0f, 0.0f),
        14,
        ecs_theme_.text_primary);

    const float opt_row_start = opt_y + 36.0f;
    const float opt_row_h = 22.0f;
    const float opt_label_x = opt_x + 14.0f;
    const float opt_value_x = opt_x + 150.0f;
    const float opt_minus_x = opt_x + opt_w - 64.0f;
    const float opt_plus_x = opt_x + opt_w - 36.0f;
    auto add_opt_row = [&](int row, const char* label, ui::Entity& value_entity, const char* dec_action,
                           const char* inc_action) {
        const float y = opt_row_start + opt_row_h * static_cast<float>(row);
        ui::add_label(
            registry,
            sf::FloatRect({opt_label_x, y}, {126.0f, 18.0f}),
            201,
            label,
            sf::Vector2f(0.0f, 0.0f),
            13,
            ecs_theme_.text_primary);
        value_entity = ui::add_label(
            registry,
            sf::FloatRect({opt_value_x, y}, {90.0f, 18.0f}),
            201,
            "-",
            sf::Vector2f(0.0f, 0.0f),
            13,
            ecs_theme_.accent);
        ui::add_button(
            registry, sf::FloatRect({opt_minus_x, y - 1.0f}, {22.0f, 20.0f}), 202, "-", dec_action, ecs_theme_);
        ui::add_button(
            registry, sf::FloatRect({opt_plus_x, y - 1.0f}, {22.0f, 20.0f}), 202, "+", inc_action, ecs_theme_);
    };
    add_opt_row(0, "Time Scale", ecs_opt_speed_value_, "opt_speed_dec", "opt_speed_inc");
    add_opt_row(1, "Max Particles", ecs_opt_particles_value_, "opt_particles_dec", "opt_particles_inc");
    add_opt_row(2, "Deposition", ecs_opt_deposition_value_, "opt_deposition_dec", "opt_deposition_inc");
    add_opt_row(3, "Source Emission", ecs_opt_emission_value_, "opt_emission_dec", "opt_emission_inc");
    add_opt_row(4, "Source Lifespan", ecs_opt_lifespan_value_, "opt_lifespan_dec", "opt_lifespan_inc");
    add_opt_row(5, "Fixed PDE Scale", ecs_opt_fixed_scale_value_, "opt_scale_dec", "opt_scale_inc");

    ui::add_button(
        registry,
        sf::FloatRect({opt_x + 14.0f, opt_y + opt_h - 32.0f}, {88.0f, 24.0f}),
        202,
        "Apply",
        "settings_apply",
        ecs_theme_);
    ui::add_button(
        registry,
        sf::FloatRect({opt_x + 108.0f, opt_y + opt_h - 32.0f}, {88.0f, 24.0f}),
        202,
        "Revert",
        "settings_revert",
        ecs_theme_);

    sync_ecs_ui_state();
}

void Application::sync_ecs_ui_state() {
    auto& registry = ecs_ui_scene_.registry();
    if (auto* button = registry.find_button(ecs_pause_button_)) {
        button->text = sim().paused() ? "Resume" : "Pause";
        button->pressed = false;
    }
    if (auto* button = registry.find_button(ecs_reset_button_)) {
        button->pressed = false;
    }
    if (auto* button = registry.find_button(ecs_estimate_button_)) {
        button->pressed = false;
        button->enabled = sim().paused();
    }

    const RuntimeSettings& pending = menu_model_.pending_settings();
    if (auto* label = registry.find_label(ecs_pde_nx_value_)) {
        label->text = std::to_string(pending.pde_grid_nx);
    }
    if (auto* label = registry.find_label(ecs_pde_ny_value_)) {
        label->text = std::to_string(pending.pde_grid_ny);
    }
    if (auto* label = registry.find_label(ecs_pde_dt_value_)) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3) << pending.dt;
        label->text = ss.str();
    }
    if (auto* label = registry.find_label(ecs_pde_mode_value_)) {
        label->text = pending.pde_diffusion_mode == AdvectionDiffusionSolver::DiffusionMode::ScalarizedTrace
            ? "Scalarized"
            : "Full Tensor";
    }
    if (auto* label = registry.find_label(ecs_opt_speed_value_)) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2) << pending.time_scale;
        label->text = ss.str();
    }
    if (auto* label = registry.find_label(ecs_opt_particles_value_)) {
        label->text = std::to_string(pending.max_particles);
    }
    if (auto* label = registry.find_label(ecs_opt_deposition_value_)) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(4) << pending.deposition_rate;
        label->text = ss.str();
    }
    if (auto* label = registry.find_label(ecs_opt_emission_value_)) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3) << pending.source_base_emission;
        label->text = ss.str();
    }
    if (auto* label = registry.find_label(ecs_opt_lifespan_value_)) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(1) << pending.source_lifespan;
        label->text = ss.str();
    }
    if (auto* label = registry.find_label(ecs_opt_fixed_scale_value_)) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(4) << pending.pde_fixed_color_scale;
        label->text = ss.str();
    }
}

void Application::handle_ecs_action(const ui::UiEvent& event) {
    if (event.action == "toggle_pause") {
        sim().toggle_paused();
        return;
    }
    if (event.action == "reset_simulation") {
        sim().reset();
        sensor_manager_.clear();
        clear_source_estimation();
        has_last_source_click_ = false;
        source_click_count_since_reset_ = 0;
        return;
    }
    if (event.action == "estimate_source") {
        run_source_estimation();
        return;
    }
    if (event.action == "pde_nx_dec") {
        menu_model_.adjust_pde_grid_nx(-2);
        return;
    }
    if (event.action == "pde_nx_inc") {
        menu_model_.adjust_pde_grid_nx(2);
        return;
    }
    if (event.action == "pde_ny_dec") {
        menu_model_.adjust_pde_grid_ny(-2);
        return;
    }
    if (event.action == "pde_ny_inc") {
        menu_model_.adjust_pde_grid_ny(2);
        return;
    }
    if (event.action == "pde_dt_dec") {
        menu_model_.adjust_dt(-0.01f);
        return;
    }
    if (event.action == "pde_dt_inc") {
        menu_model_.adjust_dt(0.01f);
        return;
    }
    if (event.action == "pde_mode_prev") {
        menu_model_.cycle_pde_diffusion_mode(-1);
        return;
    }
    if (event.action == "pde_mode_next") {
        menu_model_.cycle_pde_diffusion_mode(1);
        return;
    }
    if (event.action == "pde_revert" || event.action == "settings_revert") {
        menu_model_.discard_changes();
        return;
    }
    if (event.action == "pde_apply" || event.action == "settings_apply") {
        const ApplySettingsReport report = controller_.apply_settings(menu_model_.pending_settings());
        if (report.changed) {
            menu_model_.sync_from_current(controller_.current_settings());
        }
        apply_settings_report(report);
        return;
    }
    if (event.action == "opt_speed_dec") {
        menu_model_.adjust_time_scale(-0.25f);
        return;
    }
    if (event.action == "opt_speed_inc") {
        menu_model_.adjust_time_scale(0.25f);
        return;
    }
    if (event.action == "opt_particles_dec") {
        menu_model_.adjust_max_particles(-10);
        return;
    }
    if (event.action == "opt_particles_inc") {
        menu_model_.adjust_max_particles(10);
        return;
    }
    if (event.action == "opt_deposition_dec") {
        menu_model_.adjust_deposition_rate(-0.0025f);
        return;
    }
    if (event.action == "opt_deposition_inc") {
        menu_model_.adjust_deposition_rate(0.0025f);
        return;
    }
    if (event.action == "opt_emission_dec") {
        menu_model_.adjust_source_base_emission(-0.05f);
        return;
    }
    if (event.action == "opt_emission_inc") {
        menu_model_.adjust_source_base_emission(0.05f);
        return;
    }
    if (event.action == "opt_lifespan_dec") {
        menu_model_.adjust_source_lifespan(-1.0f);
        return;
    }
    if (event.action == "opt_lifespan_inc") {
        menu_model_.adjust_source_lifespan(1.0f);
        return;
    }
    if (event.action == "opt_scale_dec") {
        menu_model_.adjust_pde_fixed_color_scale(-1.0e-4f);
        return;
    }
    if (event.action == "opt_scale_inc") {
        menu_model_.adjust_pde_fixed_color_scale(1.0e-4f);
    }
}

void Application::apply_settings_report(const ApplySettingsReport& report) {
    if (!report.changed) {
        return;
    }

    const RuntimeSettings settings = controller_.current_settings();
    pde_fixed_color_scale_ = std::max(1.0e-8f, settings.pde_fixed_color_scale);
    pde_color_scale_runtime_ = pde_fixed_color_scale_;
    sensor_manager_.set_sample_period(settings.sensor_sample_period_s);
    sensor_manager_.set_noise_std(settings.sensor_noise_std);
    sensor_manager_.set_history_capacity(static_cast<std::size_t>(settings.sensor_history_capacity));

    if (report.recreated_simulator) {
        sensor_manager_.clear();
        clear_source_estimation();
        menu_open_ = false;
        has_last_source_click_ = false;
        source_click_count_since_reset_ = 0;
    }
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
