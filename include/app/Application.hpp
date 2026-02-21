#pragma once

#include "core/Config.hpp"
#include "core/Types.hpp"
#include "sim/Simulator.hpp"

#include <SFML/Graphics.hpp>

namespace atm {

class Application {
public:
    explicit Application(const Config& config);

    void run();

private:
    void process_events();
    void update(float frame_dt);
    void render();
    void draw_wind_field();
    void draw_pde_heatmap();
    void draw_hud_cards();
    void draw_menu_overlay();
    void apply_menu_adjustment(int direction);

    bool left_panel_contains(const sf::Vector2i& pixel) const;
    Vec2 left_panel_pixel_to_domain(const sf::Vector2i& pixel) const;
    sf::Vector2f domain_to_left_panel(const Vec2& x) const;

    Simulator simulator_;

    sf::RenderWindow window_;
    sf::Font font_;

    sf::FloatRect left_panel_;
    sf::FloatRect right_panel_;

    sf::Clock frame_clock_;
    bool menu_open_ = false;
    bool show_wind_ = true;
    int menu_index_ = 0;
    int menu_rows_ = 0;
};

} // namespace atm
