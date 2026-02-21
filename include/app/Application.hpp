#pragma once

#include "core/Config.hpp"
#include "core/Types.hpp"
#include "sim/Simulator.hpp"

#include <SFML/Graphics.hpp>

namespace atm {

enum class MassDisplayUnit {
    GramsPerSquareMeter = 0,
    MicrogramsPerSquareMeter = 1
};

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
    void left_view_bounds(float& x_min, float& x_max, float& y_min, float& y_max) const;

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
    float pde_display_max_ = 1.0e-8f;
    float last_sim_time_s_ = 0.0f;
    MassDisplayUnit mass_unit_ = MassDisplayUnit::MicrogramsPerSquareMeter;
};

} // namespace atm
