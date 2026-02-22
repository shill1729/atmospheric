#pragma once

#include "sim/SimulationController.hpp"
#include "ui/MenuModel.hpp"

#include <SFML/Graphics.hpp>

namespace atm {

struct TopToolbarClickResult {
    bool consumed = false;
    bool settings_changed = false;
    bool recreated_simulator = false;
    bool request_source_estimate = false;
};

class TopToolbar {
public:
    void draw(sf::RenderWindow& window, const sf::Font& font, const MenuModel& menu_model) const;
    void draw_active_menu(sf::RenderWindow& window, const sf::Font& font, const MenuModel& menu_model) const;
    TopToolbarClickResult handle_click(
        const sf::Vector2i& pixel, MenuModel& menu_model, SimulationController& controller) const;

    bool has_open_menu(const MenuModel& menu_model) const;
};

} // namespace atm
