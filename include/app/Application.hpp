#pragma once

#include "core/Config.hpp"
#include "core/Types.hpp"
#include "sim/SensorManager.hpp"
#include "sim/SimulationController.hpp"
#include "ui/TopToolbar.hpp"
#include "ui/MenuModel.hpp"

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
    void draw_sensor_overlay();
    void draw_hud_cards();
    void draw_control_strip();
    void draw_menu_overlay();
    void draw_help_overlay();
    void apply_menu_adjustment(int direction);
    Simulator& sim();
    const Simulator& sim() const;
    void left_view_bounds(float& x_min, float& x_max, float& y_min, float& y_max) const;

    bool left_panel_contains(const sf::Vector2i& pixel) const;
    Vec2 left_panel_pixel_to_domain(const sf::Vector2i& pixel) const;
    sf::Vector2f domain_to_left_panel(const Vec2& x) const;
    bool right_panel_contains(const sf::Vector2i& pixel) const;
    Vec2 right_panel_pixel_to_domain(const sf::Vector2i& pixel) const;
    sf::Vector2f domain_to_right_panel(const Vec2& x) const;

    SimulationController controller_;
    MenuModel menu_model_;
    TopToolbar top_toolbar_;
    SensorManager sensor_manager_;

    sf::RenderWindow window_;
    sf::Font font_;

    sf::FloatRect left_panel_;
    sf::FloatRect right_panel_;

    sf::Clock frame_clock_;
    bool menu_open_ = false;
    bool help_open_ = false;
    bool show_wind_ = true;
    bool pde_auto_color_scale_ = true;
    float pde_fixed_color_scale_ = 1.0e-4f;
    float pde_color_scale_runtime_ = 4.0e-4f;
    int menu_index_ = 0;
    int menu_rows_ = 0;
    MassDisplayUnit mass_unit_ = MassDisplayUnit::MicrogramsPerSquareMeter;
};

} // namespace atm
