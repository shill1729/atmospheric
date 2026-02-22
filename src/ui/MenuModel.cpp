#include "ui/MenuModel.hpp"

#include <algorithm>

namespace atm {

MenuModel::MenuModel(const RuntimeSettings& initial)
    : current_(initial)
    , pending_(initial) {
}

MenuModel::TopMenu MenuModel::active_top_menu() const {
    return active_top_menu_;
}

void MenuModel::toggle_top_menu(TopMenu menu) {
    active_top_menu_ = (active_top_menu_ == menu) ? TopMenu::None : menu;
}

void MenuModel::close_all() {
    active_top_menu_ = TopMenu::None;
}

const RuntimeSettings& MenuModel::pending_settings() const {
    return pending_;
}

void MenuModel::sync_from_current(const RuntimeSettings& current) {
    current_ = current;
    pending_ = current;
}

bool MenuModel::dirty() const {
    return !same_runtime_settings(current_, pending_);
}

void MenuModel::adjust_pde_grid_nx(int delta) {
    pending_.pde_grid_nx = std::clamp(pending_.pde_grid_nx + delta, 2, 1024);
}

void MenuModel::adjust_pde_grid_ny(int delta) {
    pending_.pde_grid_ny = std::clamp(pending_.pde_grid_ny + delta, 2, 1024);
}

void MenuModel::adjust_dt(float delta) {
    pending_.dt = std::clamp(pending_.dt + delta, 0.001f, 10.0f);
}

void MenuModel::cycle_pde_diffusion_mode(int direction) {
    int id = static_cast<int>(pending_.pde_diffusion_mode);
    id = (id + direction) % 2;
    if (id < 0) {
        id += 2;
    }
    pending_.pde_diffusion_mode = static_cast<AdvectionDiffusionSolver::DiffusionMode>(id);
}

void MenuModel::discard_changes() {
    pending_ = current_;
}

void MenuModel::commit_pending_as_current() {
    current_ = pending_;
}

} // namespace atm
