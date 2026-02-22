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
