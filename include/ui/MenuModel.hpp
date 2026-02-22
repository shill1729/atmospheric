#pragma once

#include "core/RuntimeSettings.hpp"

namespace atm {

class MenuModel {
public:
    enum class TopMenu {
        None,
        File,
        Options,
        Pde
    };

    explicit MenuModel(const RuntimeSettings& initial);

    TopMenu active_top_menu() const;
    void toggle_top_menu(TopMenu menu);
    void close_all();

    const RuntimeSettings& pending_settings() const;
    void sync_from_current(const RuntimeSettings& current);
    bool dirty() const;

    void adjust_time_scale(float delta);
    void adjust_max_particles(int delta);
    void adjust_deposition_rate(float delta);
    void adjust_constant_scalar_diffusivity(float delta);
    void adjust_source_base_emission(float delta);
    void adjust_source_decay_rate(float delta);
    void adjust_source_lifespan(float delta);
    void adjust_source_sigma(float delta);
    void adjust_source_max_sources(int delta);
    void adjust_pde_fixed_color_scale(float delta);
    void adjust_sensor_sample_period_s(float delta);
    void adjust_sensor_noise_std(float delta);

    void adjust_pde_grid_nx(int delta);
    void adjust_pde_grid_ny(int delta);
    void adjust_dt(float delta);
    void cycle_pde_diffusion_mode(int direction);
    void discard_changes();
    void commit_pending_as_current();

private:
    RuntimeSettings current_;
    RuntimeSettings pending_;
    TopMenu active_top_menu_ = TopMenu::None;
};

} // namespace atm
