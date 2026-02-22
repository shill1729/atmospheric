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

void MenuModel::adjust_time_scale(float delta) {
    pending_.time_scale = std::clamp(pending_.time_scale + delta, 0.25f, 120.0f);
}

void MenuModel::adjust_max_particles(int delta) {
    pending_.max_particles = std::clamp(pending_.max_particles + delta, 1, 500000);
}

void MenuModel::adjust_deposition_rate(float delta) {
    pending_.deposition_rate = std::clamp(pending_.deposition_rate + delta, 0.0f, 5.0f);
}

void MenuModel::adjust_constant_scalar_diffusivity(float delta) {
    pending_.constant_scalar_diffusivity = std::clamp(pending_.constant_scalar_diffusivity + delta, 0.001f, 5000.0f);
}

void MenuModel::adjust_source_base_emission(float delta) {
    pending_.source_base_emission = std::clamp(pending_.source_base_emission + delta, 0.0f, 100.0f);
}

void MenuModel::adjust_source_decay_rate(float delta) {
    pending_.source_decay_rate = std::clamp(pending_.source_decay_rate + delta, 0.0f, 10.0f);
}

void MenuModel::adjust_source_lifespan(float delta) {
    pending_.source_lifespan = std::clamp(pending_.source_lifespan + delta, 0.1f, 36000.0f);
}

void MenuModel::adjust_source_sigma(float delta) {
    pending_.source_sigma = std::clamp(pending_.source_sigma + delta, 0.1f, 5000.0f);
}

void MenuModel::adjust_source_max_sources(int delta) {
    pending_.source_max_sources = std::clamp(pending_.source_max_sources + delta, 1, 256);
}

void MenuModel::adjust_pde_fixed_color_scale(float delta) {
    pending_.pde_fixed_color_scale = std::clamp(pending_.pde_fixed_color_scale + delta, 1.0e-8f, 1.0e3f);
}

void MenuModel::adjust_sensor_sample_period_s(float delta) {
    pending_.sensor_sample_period_s = std::clamp(pending_.sensor_sample_period_s + delta, 0.1f, 3600.0f);
}

void MenuModel::adjust_sensor_noise_std(float delta) {
    pending_.sensor_noise_std = std::clamp(pending_.sensor_noise_std + delta, 0.0f, 100.0f);
}

void MenuModel::adjust_sensor_history_capacity(int delta) {
    pending_.sensor_history_capacity = std::clamp(pending_.sensor_history_capacity + delta, 1, 100000);
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
