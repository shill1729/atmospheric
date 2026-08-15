#pragma once

#include "adjoint/SourceEstimator.hpp"
#include "core/Config.hpp"
#include "core/Types.hpp"
#include "export/DataRecorder.hpp"
#include "io/SiteLoader.hpp"
#include "sim/SensorManager.hpp"
#include "sim/SimulationController.hpp"
#include "ui/RetroTheme.hpp"
#include "ui/TopToolbar.hpp"
#include "ui/UiEcs.hpp"
#include "ui/MenuModel.hpp"

#include <SFML/Graphics.hpp>
#include <string>
#include <vector>

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
    void draw_adjoint_overlay();
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
    void run_source_estimation();
    void clear_source_estimation();
    void sync_source_estimation_view();
    void cycle_estimation_method(int direction);
    void load_ny_sites();
    void apply_ny_sensor_preset();
    void build_ecs_ui();
    void sync_ecs_ui_state();
    void handle_ecs_action(const ui::UiEvent& event);
    void apply_settings_report(const ApplySettingsReport& report);

    struct SourceEstimationView {
        bool has_result = false;
        Vec2 x_star = Vec2::Zero();
        float t_star_s = 0.0f;
        int nx = 0;
        int ny = 0;
        std::vector<float> p_star;
        std::string status;
        bool has_error_m = false;
        float error_m = 0.0f;
    };

    struct FeynmanKacAnimation {
        bool active = false;
        AdjointSolver::Solution solution;
        int frame = 0;
        float wall_time_per_frame_s = 1.0f / 15.0f;
        float wall_accum_s = 0.0f;
    };

    SimulationController controller_;
    MenuModel menu_model_;
    TopToolbar top_toolbar_;
    ui::UiScene ecs_ui_scene_;
    ui::RetroTheme ecs_theme_;
    SensorManager sensor_manager_;
    SourceEstimator source_estimator_;
    SourceEstimationView source_estimation_;
    FeynmanKacAnimation feynman_kac_anim_;
    std::vector<SourceEstimateResult> estimation_results_;
    int selected_estimation_method_ = 0;
    DataRecorder data_recorder_;
    std::string recording_status_;
    std::string sites_status_;
    std::string ny_sites_csv_path_ = "wildfire_pm25_dataset.csv";

    sf::RenderWindow window_;
    sf::Font font_;

    sf::FloatRect left_panel_;
    sf::FloatRect right_panel_;

    sf::Clock frame_clock_;
    bool menu_open_ = false;
    bool help_open_ = false;
    bool show_wind_ = true;
    bool show_adjoint_overlay_ = true;
    bool show_ecs_quick_panel_ = false;
    bool pde_auto_color_scale_ = true;
    float concentration_scale_ug_per_m2_ = 1.923e8f;
    float mixing_height_m_ = 800.0f;
    float pde_fixed_color_scale_ = 1.0e-4f;
    float pde_color_scale_runtime_ = 4.0e-4f;
    int menu_index_ = 0;
    int menu_rows_ = 0;
    bool has_last_source_click_ = false;
    Vec2 last_source_click_ = Vec2::Zero();
    int source_click_count_since_reset_ = 0;
    MassDisplayUnit mass_unit_ = MassDisplayUnit::MicrogramsPerSquareMeter;

    ui::Entity ecs_panel_title_ = ui::kInvalidEntity;
    ui::Entity ecs_pause_button_ = ui::kInvalidEntity;
    ui::Entity ecs_reset_button_ = ui::kInvalidEntity;
    ui::Entity ecs_estimate_button_ = ui::kInvalidEntity;
    ui::Entity ecs_pde_title_ = ui::kInvalidEntity;
    ui::Entity ecs_pde_nx_value_ = ui::kInvalidEntity;
    ui::Entity ecs_pde_ny_value_ = ui::kInvalidEntity;
    ui::Entity ecs_pde_dt_value_ = ui::kInvalidEntity;
    ui::Entity ecs_pde_mode_value_ = ui::kInvalidEntity;
    ui::Entity ecs_pde_dirty_value_ = ui::kInvalidEntity;
    ui::Entity ecs_opt_title_ = ui::kInvalidEntity;
    ui::Entity ecs_opt_speed_value_ = ui::kInvalidEntity;
    ui::Entity ecs_opt_particles_value_ = ui::kInvalidEntity;
    ui::Entity ecs_opt_deposition_value_ = ui::kInvalidEntity;
    ui::Entity ecs_opt_emission_value_ = ui::kInvalidEntity;
    ui::Entity ecs_opt_lifespan_value_ = ui::kInvalidEntity;
    ui::Entity ecs_opt_fixed_scale_value_ = ui::kInvalidEntity;
    ui::Entity ecs_opt_dirty_value_ = ui::kInvalidEntity;
};

} // namespace atm
