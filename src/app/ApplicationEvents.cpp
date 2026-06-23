#include "app/Application.hpp"

#include <algorithm>
#include <optional>

namespace atm {

void Application::process_events() {
    while (const std::optional event = window_.pollEvent()) {
        if (event->is<sf::Event::Closed>()) {
            window_.close();
            continue;
        }

        if (const auto* moved = event->getIf<sf::Event::MouseMoved>()) {
            ecs_ui_scene_.handle_mouse_move(
                sf::Vector2f(static_cast<float>(moved->position.x), static_cast<float>(moved->position.y)));
        }

        if (const auto* key = event->getIf<sf::Event::KeyPressed>()) {
            if (top_toolbar_.handle_key_input(key->code, menu_model_)) {
                continue;
            }

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
                has_last_source_click_ = false;
                source_click_count_since_reset_ = 0;
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

        if (const auto* text = event->getIf<sf::Event::TextEntered>()) {
            if (top_toolbar_.handle_text_input(text->unicode, menu_model_)) {
                continue;
            }
        }

        if (const auto* click = event->getIf<sf::Event::MouseButtonPressed>()) {
            if (click->button == sf::Mouse::Button::Left) {
                std::vector<ui::UiEvent> ui_events;
                if (ecs_ui_scene_.handle_left_press(
                        sf::Vector2f(static_cast<float>(click->position.x), static_cast<float>(click->position.y)),
                        ui_events)) {
                    for (const auto& ui_event : ui_events) {
                        handle_ecs_action(ui_event);
                    }
                    sync_ecs_ui_state();
                    continue;
                }

                const TopToolbarClickResult toolbar_click
                    = top_toolbar_.handle_click(click->position, menu_model_, controller_);
                if (toolbar_click.settings_changed) {
                    ApplySettingsReport report;
                    report.changed = true;
                    report.recreated_simulator = toolbar_click.recreated_simulator;
                    apply_settings_report(report);
                }
                if (toolbar_click.request_source_estimate) {
                    run_source_estimation();
                }
                if (toolbar_click.request_apply_queued_settings) {
                    const ApplySettingsReport report = controller_.apply_settings(menu_model_.pending_settings());
                    if (report.changed) {
                        menu_model_.sync_from_current(controller_.current_settings());
                    }
                    apply_settings_report(report);
                }
                if (toolbar_click.request_revert_queued_settings) {
                    menu_model_.discard_changes();
                }
                if (toolbar_click.request_restore_defaults) {
                    const ApplySettingsReport report = controller_.restore_launch_defaults();
                    menu_model_.sync_from_current(controller_.current_settings());
                    apply_settings_report(report);
                }
                if (toolbar_click.request_toggle_recording) {
                    if (data_recorder_.is_recording()) {
                        data_recorder_.stop_recording();
                        recording_status_ = "Stopped. "
                            + std::to_string(data_recorder_.total_readings()) + " readings captured.";
                    } else {
                        data_recorder_.start_recording();
                        recording_status_ = "Recording...";
                    }
                }
                if (toolbar_click.request_export_csv) {
                    if (!data_recorder_.has_data()) {
                        recording_status_ = "No data to export. Record a session first.";
                    } else {
                        const std::string path = data_recorder_.export_csv(
                            ".",
                            std::string(sim().wind_model_name()),
                            std::string(sim().diffusion_model_name()),
                            std::string(sim().pde_diffusion_mode_name()),
                            sim().config().numerics.dt,
                            sim().time_scale(),
                            sensor_manager_.sample_period());
                        recording_status_ = path.empty() ? "Export failed (IO error)." : "Saved: " + path;
                    }
                }
                if (toolbar_click.consumed) {
                    sync_ecs_ui_state();
                    continue;
                }
            }
            if (click->button == sf::Mouse::Button::Left && left_panel_contains(click->position)) {
                const Vec2 source_pos = left_panel_pixel_to_domain(click->position);
                const std::size_t before = sim().source().active_count();
                sim().set_source(source_pos);
                const std::size_t after = sim().source().active_count();
                if (after > before) {
                    last_source_click_ = source_pos;
                    has_last_source_click_ = true;
                    ++source_click_count_since_reset_;
                }
            } else if (click->button == sf::Mouse::Button::Left && right_panel_contains(click->position)) {
                sensor_manager_.add_sensor(right_panel_pixel_to_domain(click->position), sim().time_s());
            }
        }
    }
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
        has_last_source_click_ = false;
        source_click_count_since_reset_ = 0;
        break;
    default:
        break;
    }
}

void Application::run_source_estimation() {
    if (!sim().paused()) {
        source_estimation_.status = "Pause simulation before running.";
        return;
    }

    const auto& sensors = sensor_manager_.sensors();
    if (sensors.empty()) {
        source_estimation_.status = "No sensors placed.";
        return;
    }

    const float t_now = sim().time_s();
    const auto& domain = sim().config().domain;
    const auto& ecfg = source_estimator_.config();

    const int nx = std::max(2, ecfg.adjoint_grid_nx);
    const int ny = std::max(2, ecfg.adjoint_grid_ny);
    const float dx = (domain.x_max - domain.x_min) / static_cast<float>(nx - 1);
    const float dy = (domain.y_max - domain.y_min) / static_cast<float>(ny - 1);
    const float cell_area = dx * dy;
    const float sigma = std::max(1.0f, ecfg.gaussian_sigma);
    const float inv_2s2 = 1.0f / (2.0f * sigma * sigma);
    const std::size_t n = static_cast<std::size_t>(nx * ny);

    std::vector<float> initial_phi(n, 0.0f);
    bool any_signal = false;

    for (const auto& sensor : sensors) {
        if (sensor.history.empty()) {
            continue;
        }
        const float w = std::max(0.0f, sensor.history.back().noisy_concentration);
        if (w < ecfg.detection_threshold) {
            continue;
        }
        any_signal = true;

        for (int j = 0; j < ny; ++j) {
            const float y = domain.y_min + static_cast<float>(j) * dy;
            for (int i = 0; i < nx; ++i) {
                const float x = domain.x_min + static_cast<float>(i) * dx;
                const float ddx = x - sensor.position.x();
                const float ddy = y - sensor.position.y();
                initial_phi[static_cast<std::size_t>(j * nx + i)]
                    += w * std::exp(-(ddx * ddx + ddy * ddy) * inv_2s2);
            }
        }
    }

    if (!any_signal) {
        source_estimation_.status = "No sensor above detection threshold.";
        feynman_kac_anim_.active = false;
        return;
    }

    float phi_integ = 0.0f;
    for (float v : initial_phi) {
        phi_integ += v * cell_area;
    }
    if (phi_integ > 1.0e-12f) {
        const float inv = 1.0f / phi_integ;
        for (float& v : initial_phi) {
            v *= inv;
        }
    }

    AdjointSolver::Config solve_cfg;
    solve_cfg.domain = domain;
    solve_cfg.nx = nx;
    solve_cfg.ny = ny;
    solve_cfg.dt_s = std::max(0.05f, ecfg.adjoint_dt_s);
    solve_cfg.deposition_rate = sim().config().physics.deposition_rate;
    solve_cfg.diffusion_mode = sim().pde_diffusion_mode() == AdvectionDiffusionSolver::DiffusionMode::FullTensorFlux
        ? AdjointSolver::DiffusionMode::FullTensorFlux
        : AdjointSolver::DiffusionMode::ScalarizedTrace;

    auto wind_fn = [&](float time_s, const Vec2& p) { return sim().wind_at_time(time_s, p); };
    auto diffusivity_fn = [&](float time_s, const Vec2& p) { return sim().diffusivity_at_time(time_s, p); };
    auto no_forcing = [](float, std::vector<float>&) {};

    const float t_start = std::max(0.0f, t_now - std::max(1.0f, ecfg.max_lookback_s));

    AdjointSolver solver;
    feynman_kac_anim_.solution
        = solver.solve_backward(t_start, t_now, solve_cfg, wind_fn, diffusivity_fn, no_forcing, initial_phi);
    feynman_kac_anim_.frame = 0;
    feynman_kac_anim_.wall_accum_s = 0.0f;
    feynman_kac_anim_.active = true;

    source_estimation_.has_result = false;
    source_estimation_.has_error_m = false;
    source_estimation_.status = "FK anim: "
        + std::to_string(static_cast<int>(feynman_kac_anim_.solution.snapshots.size())) + " frames";
}

void Application::clear_source_estimation() {
    source_estimation_ = SourceEstimationView{};
    feynman_kac_anim_ = FeynmanKacAnimation{};
}

} // namespace atm
