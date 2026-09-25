#include "ui/MenuModel.hpp"

#include <algorithm>
#include <cctype>

namespace atm {
namespace {
constexpr int kMaxParticles = 10000000;
constexpr float kLogStepFactor = 1.25f;
constexpr float kMaxWindScale = 100.0f;
constexpr float kMaxSourceLifespan = 1.0e7f;

// Multiplies a positive quantity by kLogStepFactor^direction, so +/- steps
// stay proportionate whether the value is 1e-6 or 1e3. A zero value steps
// up to `floor_when_zero`.
float log_step(float value, int direction, float floor_when_zero) {
    if (value <= 0.0f) {
        return direction > 0 ? floor_when_zero : 0.0f;
    }
    return direction > 0 ? value * kLogStepFactor : value / kLogStepFactor;
}
} // namespace

MenuModel::MenuModel(const RuntimeSettings& initial)
    : current_(initial)
    , pending_(initial) {
}

MenuModel::TopMenu MenuModel::active_top_menu() const {
    return active_top_menu_;
}

void MenuModel::toggle_top_menu(TopMenu menu) {
    stop_editing();
    active_top_menu_ = (active_top_menu_ == menu) ? TopMenu::None : menu;
}

void MenuModel::close_all() {
    stop_editing();
    active_top_menu_ = TopMenu::None;
}

const RuntimeSettings& MenuModel::pending_settings() const {
    return pending_;
}

void MenuModel::sync_from_current(const RuntimeSettings& current) {
    current_ = current;
    pending_ = current;
    stop_editing();
}

bool MenuModel::dirty() const {
    return !same_runtime_settings(current_, pending_);
}

void MenuModel::adjust_time_scale(float delta) {
    pending_.time_scale = std::clamp(pending_.time_scale + delta, kMinTimeScale, kMaxTimeScale);
}

void MenuModel::adjust_max_particles(int delta) {
    pending_.max_particles = std::clamp(pending_.max_particles + delta, 1, kMaxParticles);
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
    pending_.source_lifespan = std::clamp(pending_.source_lifespan + delta, 0.1f, kMaxSourceLifespan);
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

void MenuModel::step_pde_fixed_color_scale(int direction) {
    pending_.pde_fixed_color_scale = std::clamp(log_step(pending_.pde_fixed_color_scale, direction, 1.0e-8f), 1.0e-8f, 1.0e3f);
}

void MenuModel::step_time_scale(int direction) {
    pending_.time_scale = std::clamp(log_step(pending_.time_scale, direction, kMinTimeScale), kMinTimeScale, kMaxTimeScale);
}

void MenuModel::step_wind_scale(int direction) {
    pending_.wind_scale = std::clamp(log_step(pending_.wind_scale, direction, 0.05f), 0.0f, kMaxWindScale);
}

void MenuModel::step_sensor_noise_std(int direction) {
    pending_.sensor_noise_std = std::clamp(log_step(pending_.sensor_noise_std, direction, 1.0e-7f), 0.0f, 100.0f);
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
    stop_editing();
}

void MenuModel::commit_pending_as_current() {
    current_ = pending_;
}

void MenuModel::start_edit(EditableField field, const std::string& initial_text) {
    active_edit_field_ = field;
    edit_buffer_ = initial_text;
}

bool MenuModel::editing() const {
    return active_edit_field_ != EditableField::None;
}

MenuModel::EditableField MenuModel::active_edit_field() const {
    return active_edit_field_;
}

const std::string& MenuModel::edit_buffer() const {
    return edit_buffer_;
}

void MenuModel::backspace_edit_char() {
    if (!edit_buffer_.empty()) {
        edit_buffer_.pop_back();
    }
}

void MenuModel::append_edit_char(char32_t unicode) {
    if (!editing()) {
        return;
    }
    if (unicode > 127) {
        return;
    }
    const char ch = static_cast<char>(unicode);
    const bool ok = std::isdigit(static_cast<unsigned char>(ch)) || ch == '.' || ch == '-' || ch == '+' || ch == 'e'
        || ch == 'E';
    if (ok) {
        edit_buffer_.push_back(ch);
    }
}

bool MenuModel::commit_edit() {
    if (!editing()) {
        return false;
    }
    if (edit_buffer_.empty()) {
        stop_editing();
        return false;
    }

    try {
        switch (active_edit_field_) {
        case EditableField::TimeScale:
            pending_.time_scale = clampf(std::stof(edit_buffer_), kMinTimeScale, kMaxTimeScale);
            break;
        case EditableField::MaxParticles:
            pending_.max_particles = clampi(std::stoi(edit_buffer_), 1, kMaxParticles);
            break;
        case EditableField::DepositionRate:
            pending_.deposition_rate = clampf(std::stof(edit_buffer_), 0.0f, 5.0f);
            break;
        case EditableField::ConstantScalarDiffusivity:
            pending_.constant_scalar_diffusivity = clampf(std::stof(edit_buffer_), 0.001f, 5000.0f);
            break;
        case EditableField::WindScale:
            pending_.wind_scale = clampf(std::stof(edit_buffer_), 0.0f, kMaxWindScale);
            break;
        case EditableField::SourceBaseEmission:
            pending_.source_base_emission = clampf(std::stof(edit_buffer_), 0.0f, 100.0f);
            break;
        case EditableField::SourceDecayRate:
            pending_.source_decay_rate = clampf(std::stof(edit_buffer_), 0.0f, 10.0f);
            break;
        case EditableField::SourceLifespan:
            pending_.source_lifespan = clampf(std::stof(edit_buffer_), 0.1f, kMaxSourceLifespan);
            break;
        case EditableField::SourceSigma:
            pending_.source_sigma = clampf(std::stof(edit_buffer_), 0.1f, 5000.0f);
            break;
        case EditableField::SourceMaxSources:
            pending_.source_max_sources = clampi(std::stoi(edit_buffer_), 1, 256);
            break;
        case EditableField::PdeFixedColorScale:
            pending_.pde_fixed_color_scale = clampf(std::stof(edit_buffer_), 1.0e-8f, 1.0e3f);
            break;
        case EditableField::SensorSamplePeriod:
            pending_.sensor_sample_period_s = clampf(std::stof(edit_buffer_), 0.1f, 3600.0f);
            break;
        case EditableField::SensorNoiseStd:
            pending_.sensor_noise_std = clampf(std::stof(edit_buffer_), 0.0f, 100.0f);
            break;
        case EditableField::SensorHistoryCapacity:
            pending_.sensor_history_capacity = clampi(std::stoi(edit_buffer_), 1, 100000);
            break;
        case EditableField::PdeGridNx:
            pending_.pde_grid_nx = clampi(std::stoi(edit_buffer_), 2, 1024);
            break;
        case EditableField::PdeGridNy:
            pending_.pde_grid_ny = clampi(std::stoi(edit_buffer_), 2, 1024);
            break;
        case EditableField::Dt:
            pending_.dt = clampf(std::stof(edit_buffer_), 0.001f, 10.0f);
            break;
        case EditableField::PdeDiffusionMode: {
            int id = clampi(std::stoi(edit_buffer_), 0, 1);
            pending_.pde_diffusion_mode = static_cast<AdvectionDiffusionSolver::DiffusionMode>(id);
            break;
        }
        case EditableField::None:
            stop_editing();
            return false;
        }
    } catch (...) {
        stop_editing();
        return false;
    }

    stop_editing();
    return true;
}

void MenuModel::cancel_edit() {
    stop_editing();
}

void MenuModel::stop_editing() {
    active_edit_field_ = EditableField::None;
    edit_buffer_.clear();
}

float MenuModel::clampf(float v, float lo, float hi) {
    return std::clamp(v, lo, hi);
}

int MenuModel::clampi(int v, int lo, int hi) {
    return std::clamp(v, lo, hi);
}

} // namespace atm
