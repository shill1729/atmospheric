#include "ui/TopToolbar.hpp"

#include <array>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

namespace atm {
namespace {
constexpr float TOP_TOOLBAR_HEIGHT = 30.0f;
constexpr float TOP_TOOLBAR_BUTTON_W = 88.0f;
constexpr float TOP_TOOLBAR_BUTTON_H = 24.0f;

sf::FloatRect toolbar_button_rect(int index) {
    const float x = 10.0f + static_cast<float>(index) * (TOP_TOOLBAR_BUTTON_W + 8.0f);
    const float y = 3.0f;
    return sf::FloatRect({x, y}, {TOP_TOOLBAR_BUTTON_W, TOP_TOOLBAR_BUTTON_H});
}

sf::FloatRect pde_menu_rect() {
    return sf::FloatRect({toolbar_button_rect(2).position.x + 16.0f, TOP_TOOLBAR_HEIGHT + 8.0f}, {300.0f, 184.0f});
}

bool handle_pde_menu_click(
    const sf::Vector2i& pixel, const sf::FloatRect& panel, MenuModel& menu_model, SimulationController& controller,
    TopToolbarClickResult& out) {
    const float x0 = panel.position.x + panel.size.x - 56.0f;
    const float x1 = x0 - 24.0f;
    const float y_nx = panel.position.y + 38.0f;
    const float y_ny = panel.position.y + 66.0f;
    const float y_dt = panel.position.y + 94.0f;
    const float y_mode = panel.position.y + 122.0f;
    const sf::Vector2f p(static_cast<float>(pixel.x), static_cast<float>(pixel.y));

    auto hit_small = [&](float x, float y) { return sf::FloatRect({x, y}, {20.0f, 18.0f}).contains(p); };

    if (hit_small(x1, y_nx)) {
        menu_model.adjust_pde_grid_nx(-2);
        out.consumed = true;
        return true;
    }
    if (hit_small(x0, y_nx)) {
        menu_model.adjust_pde_grid_nx(2);
        out.consumed = true;
        return true;
    }
    if (hit_small(x1, y_ny)) {
        menu_model.adjust_pde_grid_ny(-2);
        out.consumed = true;
        return true;
    }
    if (hit_small(x0, y_ny)) {
        menu_model.adjust_pde_grid_ny(2);
        out.consumed = true;
        return true;
    }
    if (hit_small(x1, y_dt)) {
        menu_model.adjust_dt(-0.01f);
        out.consumed = true;
        return true;
    }
    if (hit_small(x0, y_dt)) {
        menu_model.adjust_dt(0.01f);
        out.consumed = true;
        return true;
    }
    if (hit_small(x1, y_mode)) {
        menu_model.cycle_pde_diffusion_mode(-1);
        out.consumed = true;
        return true;
    }
    if (hit_small(x0, y_mode)) {
        menu_model.cycle_pde_diffusion_mode(1);
        out.consumed = true;
        return true;
    }

    const sf::FloatRect apply({panel.position.x + 12.0f, panel.position.y + panel.size.y - 34.0f}, {88.0f, 24.0f});
    const sf::FloatRect cancel({panel.position.x + 108.0f, panel.position.y + panel.size.y - 34.0f}, {88.0f, 24.0f});
    if (apply.contains(p)) {
        const ApplySettingsReport report = controller.apply_settings(menu_model.pending_settings());
        out.consumed = true;
        if (report.changed) {
            out.settings_changed = true;
            out.recreated_simulator = report.recreated_simulator;
            menu_model.sync_from_current(controller.current_settings());
        }
        return true;
    }
    if (cancel.contains(p)) {
        menu_model.discard_changes();
        out.consumed = true;
        return true;
    }

    out.consumed = true;
    return true;
}
} // namespace

void TopToolbar::draw(sf::RenderWindow& window, const sf::Font& font, const MenuModel& menu_model) const {
    sf::RectangleShape bar({static_cast<float>(window.getSize().x), TOP_TOOLBAR_HEIGHT});
    bar.setPosition({0.0f, 0.0f});
    bar.setFillColor(sf::Color(16, 22, 31, 240));
    bar.setOutlineThickness(1.0f);
    bar.setOutlineColor(sf::Color(72, 102, 132, 220));
    window.draw(bar);

    const std::array<std::pair<MenuModel::TopMenu, const char*>, 3> items{{
        {MenuModel::TopMenu::File, "File"},
        {MenuModel::TopMenu::Options, "Options"},
        {MenuModel::TopMenu::Pde, "PDE"},
    }};

    for (int i = 0; i < static_cast<int>(items.size()); ++i) {
        const sf::FloatRect r = toolbar_button_rect(i);
        sf::RectangleShape button({r.size.x, r.size.y});
        button.setPosition({r.position.x, r.position.y});
        const bool active = menu_model.active_top_menu() == items[i].first;
        button.setFillColor(active ? sf::Color(58, 90, 124, 240) : sf::Color(28, 38, 52, 220));
        button.setOutlineThickness(1.0f);
        button.setOutlineColor(sf::Color(98, 132, 166, 220));
        window.draw(button);

        sf::Text text(font, items[i].second, 14);
        text.setPosition({r.position.x + 10.0f, r.position.y + 3.0f});
        text.setFillColor(sf::Color(218, 232, 245));
        window.draw(text);
    }
}

void TopToolbar::draw_active_menu(sf::RenderWindow& window, const sf::Font& font, const MenuModel& menu_model) const {
    if (menu_model.active_top_menu() != MenuModel::TopMenu::Pde) {
        return;
    }

    const sf::FloatRect panel = pde_menu_rect();
    sf::RectangleShape bg({panel.size.x, panel.size.y});
    bg.setPosition({panel.position.x, panel.position.y});
    bg.setFillColor(sf::Color(20, 30, 44, 244));
    bg.setOutlineThickness(1.0f);
    bg.setOutlineColor(sf::Color(96, 134, 170, 230));
    window.draw(bg);

    const RuntimeSettings& pending = menu_model.pending_settings();
    sf::Text title(font, "PDE Settings", 15);
    title.setPosition({panel.position.x + 10.0f, panel.position.y + 8.0f});
    title.setFillColor(sf::Color(220, 236, 250));
    window.draw(title);

    auto draw_row = [&](float y, const std::string& label, const std::string& value) {
        sf::Text l(font, label, 14);
        l.setPosition({panel.position.x + 12.0f, y});
        l.setFillColor(sf::Color(196, 214, 232));
        window.draw(l);
        sf::Text v(font, value, 14);
        v.setPosition({panel.position.x + 168.0f, y});
        v.setFillColor(sf::Color(233, 244, 255));
        window.draw(v);
    };

    draw_row(panel.position.y + 38.0f, "Grid Nx", std::to_string(pending.pde_grid_nx));
    draw_row(panel.position.y + 66.0f, "Grid Ny", std::to_string(pending.pde_grid_ny));
    {
        std::ostringstream dt;
        dt << std::fixed << std::setprecision(3) << pending.dt;
        draw_row(panel.position.y + 94.0f, "dt", dt.str());
    }
    draw_row(
        panel.position.y + 122.0f,
        "Diffusion",
        pending.pde_diffusion_mode == AdvectionDiffusionSolver::DiffusionMode::ScalarizedTrace ? "Scalarized"
                                                                                                : "Full Tensor");

    auto draw_small_button = [&](float x, float y, const char* label) {
        sf::RectangleShape b({20.0f, 18.0f});
        b.setPosition({x, y});
        b.setFillColor(sf::Color(52, 76, 102, 240));
        b.setOutlineThickness(1.0f);
        b.setOutlineColor(sf::Color(112, 148, 184, 235));
        window.draw(b);
        sf::Text t(font, label, 13);
        t.setPosition({x + 6.0f, y - 1.0f});
        t.setFillColor(sf::Color(228, 241, 252));
        window.draw(t);
    };
    const float btn_x = panel.position.x + panel.size.x - 56.0f;
    draw_small_button(btn_x, panel.position.y + 38.0f, "+");
    draw_small_button(btn_x - 24.0f, panel.position.y + 38.0f, "-");
    draw_small_button(btn_x, panel.position.y + 66.0f, "+");
    draw_small_button(btn_x - 24.0f, panel.position.y + 66.0f, "-");
    draw_small_button(btn_x, panel.position.y + 94.0f, "+");
    draw_small_button(btn_x - 24.0f, panel.position.y + 94.0f, "-");
    draw_small_button(btn_x, panel.position.y + 122.0f, ">");
    draw_small_button(btn_x - 24.0f, panel.position.y + 122.0f, "<");

    sf::RectangleShape apply({88.0f, 24.0f});
    apply.setPosition({panel.position.x + 12.0f, panel.position.y + panel.size.y - 34.0f});
    apply.setFillColor(menu_model.dirty() ? sf::Color(48, 120, 76, 245) : sf::Color(50, 70, 58, 220));
    apply.setOutlineThickness(1.0f);
    apply.setOutlineColor(sf::Color(108, 170, 126, 230));
    window.draw(apply);
    sf::Text apply_text(font, "Apply", 14);
    apply_text.setPosition({panel.position.x + 37.0f, panel.position.y + panel.size.y - 31.0f});
    apply_text.setFillColor(sf::Color(234, 249, 238));
    window.draw(apply_text);

    sf::RectangleShape cancel({88.0f, 24.0f});
    cancel.setPosition({panel.position.x + 108.0f, panel.position.y + panel.size.y - 34.0f});
    cancel.setFillColor(sf::Color(68, 80, 92, 235));
    cancel.setOutlineThickness(1.0f);
    cancel.setOutlineColor(sf::Color(118, 136, 154, 230));
    window.draw(cancel);
    sf::Text cancel_text(font, "Revert", 14);
    cancel_text.setPosition({panel.position.x + 129.0f, panel.position.y + panel.size.y - 31.0f});
    cancel_text.setFillColor(sf::Color(226, 232, 238));
    window.draw(cancel_text);
}

TopToolbarClickResult TopToolbar::handle_click(
    const sf::Vector2i& pixel, MenuModel& menu_model, SimulationController& controller) const {
    TopToolbarClickResult out;
    const sf::Vector2f p(static_cast<float>(pixel.x), static_cast<float>(pixel.y));
    const bool had_open_menu = menu_model.active_top_menu() != MenuModel::TopMenu::None;
    for (int i = 0; i < 3; ++i) {
        if (toolbar_button_rect(i).contains(p)) {
            if (i == 0) {
                menu_model.toggle_top_menu(MenuModel::TopMenu::File);
            } else if (i == 1) {
                menu_model.toggle_top_menu(MenuModel::TopMenu::Options);
            } else {
                menu_model.toggle_top_menu(MenuModel::TopMenu::Pde);
            }
            out.consumed = true;
            return out;
        }
    }

    if (menu_model.active_top_menu() == MenuModel::TopMenu::Pde && pde_menu_rect().contains(p)) {
        handle_pde_menu_click(pixel, pde_menu_rect(), menu_model, controller, out);
        return out;
    }

    if (p.y <= TOP_TOOLBAR_HEIGHT || menu_model.active_top_menu() != MenuModel::TopMenu::None) {
        menu_model.close_all();
        out.consumed = true;
        return out;
    }
    if (had_open_menu) {
        out.consumed = true;
    }
    return out;
}

bool TopToolbar::has_open_menu(const MenuModel& menu_model) const {
    return menu_model.active_top_menu() != MenuModel::TopMenu::None;
}

} // namespace atm
