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
        source_estimation_.status = "Pause simulation before running estimation.";
        return;
    }

    const auto result = source_estimator_.estimate(sensor_manager_.sensors(), sim());
    source_estimation_.status = result.message;
    if (!result.success) {
        source_estimation_.has_result = false;
        source_estimation_.has_error_m = false;
        return;
    }

    source_estimation_.has_result = true;
    source_estimation_.x_star = result.x_star;
    source_estimation_.t_star_s = result.t_star_s;
    source_estimation_.nx = result.nx;
    source_estimation_.ny = result.ny;
    source_estimation_.p_star = result.p_star;
    source_estimation_.has_error_m = false;
    source_estimation_.error_m = 0.0f;
    if (has_last_source_click_ && source_click_count_since_reset_ == 1) {
        source_estimation_.has_error_m = true;
        source_estimation_.error_m = (source_estimation_.x_star - last_source_click_).norm();
    }
}

void Application::clear_source_estimation() {
    source_estimation_ = SourceEstimationView{};
}

} // namespace atm
