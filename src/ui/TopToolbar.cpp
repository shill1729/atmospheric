#include "ui/TopToolbar.hpp"

#include <array>
#include <iomanip>
#include <sstream>
#include <string>

namespace atm {
namespace {

constexpr float kToolbarHeight = 34.0f;
constexpr float kTopButtonWidth = 96.0f;
constexpr float kTopButtonHeight = 24.0f;
constexpr float kTopButtonGap = 8.0f;
constexpr float kTopButtonX0 = 10.0f;
constexpr float kTopButtonY = 5.0f;

constexpr float kPanelTopOffset = 8.0f;
constexpr float kRowY0 = 42.0f;
constexpr float kRowStep = 30.0f;
constexpr float kMinusW = 24.0f;
constexpr float kValueW = 118.0f;
constexpr float kPlusW = 24.0f;
constexpr float kControlH = 20.0f;
constexpr float kControlYInset = 1.0f;
constexpr float kRightInset = 12.0f;
constexpr float kControlsGap = 6.0f;

struct TopMenuEntry {
    MenuModel::TopMenu menu;
    const char* label;
};

constexpr std::array<TopMenuEntry, 6> kTopMenus{{
    {MenuModel::TopMenu::File, "File"},
    {MenuModel::TopMenu::Source, "Source"},
    {MenuModel::TopMenu::Numerics, "Numerics"},
    {MenuModel::TopMenu::Sensors, "Sensors"},
    {MenuModel::TopMenu::Display, "Display"},
    {MenuModel::TopMenu::Pde, "PDE"},
}};

sf::FloatRect top_button_rect(int index) {
    const float x = kTopButtonX0 + static_cast<float>(index) * (kTopButtonWidth + kTopButtonGap);
    return sf::FloatRect({x, kTopButtonY}, {kTopButtonWidth, kTopButtonHeight});
}

int top_menu_index(MenuModel::TopMenu menu) {
    for (int i = 0; i < static_cast<int>(kTopMenus.size()); ++i) {
        if (kTopMenus[static_cast<std::size_t>(i)].menu == menu) {
            return i;
        }
    }
    return 0;
}

sf::FloatRect menu_panel_rect(MenuModel::TopMenu menu) {
    const sf::FloatRect anchor = top_button_rect(top_menu_index(menu));
    const float x = anchor.position.x + 2.0f;
    const float y = kToolbarHeight + kPanelTopOffset;
    switch (menu) {
    case MenuModel::TopMenu::File:
        return sf::FloatRect({x, y}, {286.0f, 176.0f});
    case MenuModel::TopMenu::Source:
        return sf::FloatRect({x, y}, {430.0f, 214.0f});
    case MenuModel::TopMenu::Numerics:
        return sf::FloatRect({x, y}, {430.0f, 184.0f});
    case MenuModel::TopMenu::Sensors:
        return sf::FloatRect({x, y}, {430.0f, 154.0f});
    case MenuModel::TopMenu::Display:
        return sf::FloatRect({x, y}, {430.0f, 94.0f});
    case MenuModel::TopMenu::Pde:
        return sf::FloatRect({x, y}, {430.0f, 184.0f});
    case MenuModel::TopMenu::None:
    default:
        return sf::FloatRect({x, y}, {0.0f, 0.0f});
    }
}

sf::FloatRect file_action_rect(const sf::FloatRect& panel, int index) {
    const float x = panel.position.x + 12.0f;
    const float y = panel.position.y + 36.0f + static_cast<float>(index) * 32.0f;
    return sf::FloatRect({x, y}, {panel.size.x - 24.0f, 24.0f});
}

float row_y(const sf::FloatRect& panel, int row) {
    return panel.position.y + kRowY0 + static_cast<float>(row) * kRowStep;
}

sf::FloatRect minus_rect(const sf::FloatRect& panel, int row) {
    const float y = row_y(panel, row) + kControlYInset;
    const float x = panel.position.x + panel.size.x - kRightInset - kPlusW - kControlsGap - kValueW - kControlsGap - kMinusW;
    return sf::FloatRect({x, y}, {kMinusW, kControlH});
}

sf::FloatRect value_rect(const sf::FloatRect& panel, int row) {
    const float y = row_y(panel, row) + kControlYInset;
    const float x = panel.position.x + panel.size.x - kRightInset - kPlusW - kControlsGap - kValueW;
    return sf::FloatRect({x, y}, {kValueW, kControlH});
}

sf::FloatRect plus_rect(const sf::FloatRect& panel, int row) {
    const float y = row_y(panel, row) + kControlYInset;
    const float x = panel.position.x + panel.size.x - kRightInset - kPlusW;
    return sf::FloatRect({x, y}, {kPlusW, kControlH});
}

std::string format_fixed(float v, int precision) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << v;
    return ss.str();
}

std::string diffusion_mode_label(AdvectionDiffusionSolver::DiffusionMode mode) {
    return mode == AdvectionDiffusionSolver::DiffusionMode::ScalarizedTrace ? "Scalarized (1)" : "Full Tensor (0)";
}

std::string editable_field_string(const RuntimeSettings& pending, MenuModel::EditableField field) {
    switch (field) {
    case MenuModel::EditableField::TimeScale:
        return format_fixed(pending.time_scale, 2);
    case MenuModel::EditableField::MaxParticles:
        return std::to_string(pending.max_particles);
    case MenuModel::EditableField::DepositionRate:
        return format_fixed(pending.deposition_rate, 4);
    case MenuModel::EditableField::ConstantScalarDiffusivity:
        return format_fixed(pending.constant_scalar_diffusivity, 3);
    case MenuModel::EditableField::SourceBaseEmission:
        return format_fixed(pending.source_base_emission, 3);
    case MenuModel::EditableField::SourceDecayRate:
        return format_fixed(pending.source_decay_rate, 4);
    case MenuModel::EditableField::SourceLifespan:
        return format_fixed(pending.source_lifespan, 1);
    case MenuModel::EditableField::SourceSigma:
        return format_fixed(pending.source_sigma, 1);
    case MenuModel::EditableField::SourceMaxSources:
        return std::to_string(pending.source_max_sources);
    case MenuModel::EditableField::PdeFixedColorScale:
        return format_fixed(pending.pde_fixed_color_scale, 4);
    case MenuModel::EditableField::SensorSamplePeriod:
        return format_fixed(pending.sensor_sample_period_s, 2);
    case MenuModel::EditableField::SensorNoiseStd:
        return format_fixed(pending.sensor_noise_std, 3);
    case MenuModel::EditableField::SensorHistoryCapacity:
        return std::to_string(pending.sensor_history_capacity);
    case MenuModel::EditableField::PdeGridNx:
        return std::to_string(pending.pde_grid_nx);
    case MenuModel::EditableField::PdeGridNy:
        return std::to_string(pending.pde_grid_ny);
    case MenuModel::EditableField::Dt:
        return format_fixed(pending.dt, 3);
    case MenuModel::EditableField::PdeDiffusionMode:
        return std::to_string(static_cast<int>(pending.pde_diffusion_mode));
    case MenuModel::EditableField::None:
    default:
        return std::string();
    }
}

std::string display_field_string(const RuntimeSettings& pending, MenuModel::EditableField field) {
    if (field == MenuModel::EditableField::PdeDiffusionMode) {
        return diffusion_mode_label(pending.pde_diffusion_mode);
    }
    return editable_field_string(pending, field);
}

void draw_panel_frame(sf::RenderWindow& window, const sf::Font& font, const sf::FloatRect& panel, const char* title) {
    sf::RectangleShape bg({panel.size.x, panel.size.y});
    bg.setPosition({panel.position.x, panel.position.y});
    bg.setFillColor(sf::Color(18, 28, 42, 244));
    bg.setOutlineThickness(1.0f);
    bg.setOutlineColor(sf::Color(98, 132, 168, 230));
    window.draw(bg);

    sf::Text title_text(font, title, 15);
    title_text.setPosition({panel.position.x + 10.0f, panel.position.y + 8.0f});
    title_text.setFillColor(sf::Color(224, 236, 248));
    window.draw(title_text);
}

void draw_small_button(
    sf::RenderWindow& window, const sf::Font& font, const sf::FloatRect& rect, const std::string& label, bool active_color) {
    sf::RectangleShape b({rect.size.x, rect.size.y});
    b.setPosition({rect.position.x, rect.position.y});
    b.setFillColor(active_color ? sf::Color(53, 79, 106, 240) : sf::Color(46, 62, 80, 224));
    b.setOutlineThickness(1.0f);
    b.setOutlineColor(sf::Color(112, 148, 184, 235));
    window.draw(b);

    sf::Text t(font, label, 13);
    t.setPosition({rect.position.x + 8.0f, rect.position.y - 1.0f});
    t.setFillColor(sf::Color(230, 241, 252));
    window.draw(t);
}

void draw_action_button(
    sf::RenderWindow& window, const sf::Font& font, const sf::FloatRect& rect, const char* text, bool emphasized) {
    sf::RectangleShape b({rect.size.x, rect.size.y});
    b.setPosition({rect.position.x, rect.position.y});
    b.setFillColor(emphasized ? sf::Color(52, 116, 78, 245) : sf::Color(52, 76, 102, 240));
    b.setOutlineThickness(1.0f);
    b.setOutlineColor(emphasized ? sf::Color(108, 170, 126, 235) : sf::Color(112, 148, 184, 235));
    window.draw(b);

    sf::Text label(font, text, 13);
    label.setPosition({rect.position.x + 10.0f, rect.position.y + 3.0f});
    label.setFillColor(sf::Color(230, 241, 252));
    window.draw(label);
}

void draw_setting_row(
    sf::RenderWindow& window, const sf::Font& font, const sf::FloatRect& panel, int row, const char* label,
    const std::string& value, const std::string& minus_text, const std::string& plus_text, bool editing) {
    const float y = row_y(panel, row);
    sf::Text l(font, label, 14);
    l.setPosition({panel.position.x + 14.0f, y});
    l.setFillColor(sf::Color(196, 214, 232));
    window.draw(l);

    const sf::FloatRect v_rect = value_rect(panel, row);
    sf::RectangleShape value_bg({v_rect.size.x, v_rect.size.y});
    value_bg.setPosition({v_rect.position.x, v_rect.position.y});
    value_bg.setFillColor(editing ? sf::Color(38, 70, 102, 245) : sf::Color(34, 52, 74, 230));
    value_bg.setOutlineThickness(1.0f);
    value_bg.setOutlineColor(editing ? sf::Color(144, 188, 230, 240) : sf::Color(104, 140, 176, 228));
    window.draw(value_bg);

    sf::Text v(font, value, 13);
    v.setPosition({v_rect.position.x + 8.0f, v_rect.position.y + 1.0f});
    v.setFillColor(sf::Color(232, 243, 255));
    window.draw(v);

    draw_small_button(window, font, minus_rect(panel, row), minus_text, false);
    draw_small_button(window, font, plus_rect(panel, row), plus_text, false);
}

void adjust_field(MenuModel& menu_model, MenuModel::EditableField field, int direction) {
    switch (field) {
    case MenuModel::EditableField::TimeScale:
        menu_model.adjust_time_scale(direction * 0.25f);
        break;
    case MenuModel::EditableField::MaxParticles:
        menu_model.adjust_max_particles(direction * 1000);
        break;
    case MenuModel::EditableField::DepositionRate:
        menu_model.adjust_deposition_rate(direction * 0.0025f);
        break;
    case MenuModel::EditableField::ConstantScalarDiffusivity:
        menu_model.adjust_constant_scalar_diffusivity(direction * 1.0f);
        break;
    case MenuModel::EditableField::SourceBaseEmission:
        menu_model.adjust_source_base_emission(direction * 0.05f);
        break;
    case MenuModel::EditableField::SourceDecayRate:
        menu_model.adjust_source_decay_rate(direction * 0.005f);
        break;
    case MenuModel::EditableField::SourceLifespan:
        menu_model.adjust_source_lifespan(direction * 1.0f);
        break;
    case MenuModel::EditableField::SourceSigma:
        menu_model.adjust_source_sigma(direction * 2.0f);
        break;
    case MenuModel::EditableField::SourceMaxSources:
        menu_model.adjust_source_max_sources(direction);
        break;
    case MenuModel::EditableField::PdeFixedColorScale:
        menu_model.adjust_pde_fixed_color_scale(direction * 1.0e-4f);
        break;
    case MenuModel::EditableField::SensorSamplePeriod:
        menu_model.adjust_sensor_sample_period_s(direction * 0.5f);
        break;
    case MenuModel::EditableField::SensorNoiseStd:
        menu_model.adjust_sensor_noise_std(direction * 0.01f);
        break;
    case MenuModel::EditableField::SensorHistoryCapacity:
        menu_model.adjust_sensor_history_capacity(direction * 10);
        break;
    case MenuModel::EditableField::PdeGridNx:
        menu_model.adjust_pde_grid_nx(direction * 2);
        break;
    case MenuModel::EditableField::PdeGridNy:
        menu_model.adjust_pde_grid_ny(direction * 2);
        break;
    case MenuModel::EditableField::Dt:
        menu_model.adjust_dt(direction * 0.01f);
        break;
    case MenuModel::EditableField::PdeDiffusionMode:
        menu_model.cycle_pde_diffusion_mode(direction >= 0 ? 1 : -1);
        break;
    case MenuModel::EditableField::None:
    default:
        break;
    }
}

bool handle_setting_row_click(
    const sf::Vector2f& p, const sf::FloatRect& panel, int row, MenuModel::EditableField field, MenuModel& menu_model) {
    if (minus_rect(panel, row).contains(p)) {
        adjust_field(menu_model, field, -1);
        return true;
    }
    if (plus_rect(panel, row).contains(p)) {
        adjust_field(menu_model, field, 1);
        return true;
    }
    if (value_rect(panel, row).contains(p) && field != MenuModel::EditableField::None) {
        const RuntimeSettings& pending = menu_model.pending_settings();
        menu_model.start_edit(field, editable_field_string(pending, field));
        return true;
    }
    return false;
}

} // namespace

void TopToolbar::draw(sf::RenderWindow& window, const sf::Font& font, const MenuModel& menu_model) const {
    sf::RectangleShape bar({static_cast<float>(window.getSize().x), kToolbarHeight});
    bar.setPosition({0.0f, 0.0f});
    bar.setFillColor(sf::Color(14, 20, 30, 244));
    bar.setOutlineThickness(1.0f);
    bar.setOutlineColor(sf::Color(78, 106, 136, 225));
    window.draw(bar);

    for (int i = 0; i < static_cast<int>(kTopMenus.size()); ++i) {
        const sf::FloatRect r = top_button_rect(i);
        const bool active = (menu_model.active_top_menu() == kTopMenus[static_cast<std::size_t>(i)].menu);
        sf::RectangleShape button({r.size.x, r.size.y});
        button.setPosition({r.position.x, r.position.y});
        button.setFillColor(active ? sf::Color(58, 90, 124, 240) : sf::Color(28, 38, 52, 220));
        button.setOutlineThickness(1.0f);
        button.setOutlineColor(sf::Color(98, 132, 166, 220));
        window.draw(button);

        sf::Text text(font, kTopMenus[static_cast<std::size_t>(i)].label, 14);
        text.setPosition({r.position.x + 10.0f, r.position.y + 3.0f});
        text.setFillColor(sf::Color(218, 232, 245));
        window.draw(text);
    }

    if (menu_model.dirty()) {
        sf::Text dirty(font, "Queued changes pending", 13);
        dirty.setPosition({kTopButtonX0 + 6.0f * (kTopButtonWidth + kTopButtonGap) + 16.0f, 9.0f});
        dirty.setFillColor(sf::Color(255, 214, 140));
        window.draw(dirty);
    }
}

void TopToolbar::draw_active_menu(sf::RenderWindow& window, const sf::Font& font, const MenuModel& menu_model) const {
    const MenuModel::TopMenu top_menu = menu_model.active_top_menu();
    if (top_menu == MenuModel::TopMenu::None) {
        return;
    }

    const sf::FloatRect panel = menu_panel_rect(top_menu);
    const RuntimeSettings& pending = menu_model.pending_settings();
    const bool editing = menu_model.editing();
    const MenuModel::EditableField active_field = menu_model.active_edit_field();

    if (top_menu == MenuModel::TopMenu::File) {
        draw_panel_frame(window, font, panel, "File");

        draw_action_button(window, font, file_action_rect(panel, 0), "Estimate Source (Paused)", false);
        draw_action_button(window, font, file_action_rect(panel, 1), "Apply Queued Changes", menu_model.dirty());
        draw_action_button(window, font, file_action_rect(panel, 2), "Revert Queued Changes", menu_model.dirty());
        draw_action_button(window, font, file_action_rect(panel, 3), "Restore Launch Defaults", false);
        return;
    }

    if (top_menu == MenuModel::TopMenu::Source) {
        draw_panel_frame(window, font, panel, "Source");
        draw_setting_row(
            window, font, panel, 0, "Base Emission", editing && active_field == MenuModel::EditableField::SourceBaseEmission
                ? menu_model.edit_buffer() + "_"
                : display_field_string(pending, MenuModel::EditableField::SourceBaseEmission),
            "-", "+", editing && active_field == MenuModel::EditableField::SourceBaseEmission);
        draw_setting_row(
            window, font, panel, 1, "Decay Rate", editing && active_field == MenuModel::EditableField::SourceDecayRate
                ? menu_model.edit_buffer() + "_"
                : display_field_string(pending, MenuModel::EditableField::SourceDecayRate),
            "-", "+", editing && active_field == MenuModel::EditableField::SourceDecayRate);
        draw_setting_row(
            window, font, panel, 2, "Lifespan (s)", editing && active_field == MenuModel::EditableField::SourceLifespan
                ? menu_model.edit_buffer() + "_"
                : display_field_string(pending, MenuModel::EditableField::SourceLifespan),
            "-", "+", editing && active_field == MenuModel::EditableField::SourceLifespan);
        draw_setting_row(
            window, font, panel, 3, "Sigma", editing && active_field == MenuModel::EditableField::SourceSigma
                ? menu_model.edit_buffer() + "_"
                : display_field_string(pending, MenuModel::EditableField::SourceSigma),
            "-", "+", editing && active_field == MenuModel::EditableField::SourceSigma);
        draw_setting_row(
            window, font, panel, 4, "Max Sources", editing && active_field == MenuModel::EditableField::SourceMaxSources
                ? menu_model.edit_buffer() + "_"
                : display_field_string(pending, MenuModel::EditableField::SourceMaxSources),
            "-", "+", editing && active_field == MenuModel::EditableField::SourceMaxSources);
        return;
    }

    if (top_menu == MenuModel::TopMenu::Numerics) {
        draw_panel_frame(window, font, panel, "Numerics");
        draw_setting_row(
            window, font, panel, 0, "Time Scale", editing && active_field == MenuModel::EditableField::TimeScale
                ? menu_model.edit_buffer() + "_"
                : display_field_string(pending, MenuModel::EditableField::TimeScale),
            "-", "+", editing && active_field == MenuModel::EditableField::TimeScale);
        draw_setting_row(
            window, font, panel, 1, "Max Particles", editing && active_field == MenuModel::EditableField::MaxParticles
                ? menu_model.edit_buffer() + "_"
                : display_field_string(pending, MenuModel::EditableField::MaxParticles),
            "-", "+", editing && active_field == MenuModel::EditableField::MaxParticles);
        draw_setting_row(
            window, font, panel, 2, "Deposition", editing && active_field == MenuModel::EditableField::DepositionRate
                ? menu_model.edit_buffer() + "_"
                : display_field_string(pending, MenuModel::EditableField::DepositionRate),
            "-", "+", editing && active_field == MenuModel::EditableField::DepositionRate);
        draw_setting_row(
            window, font, panel, 3, "Const Diffusivity",
            editing && active_field == MenuModel::EditableField::ConstantScalarDiffusivity ? menu_model.edit_buffer() + "_"
                                                                                             : display_field_string(
                                                                                                   pending,
                                                                                                   MenuModel::EditableField::
                                                                                                       ConstantScalarDiffusivity),
            "-", "+", editing && active_field == MenuModel::EditableField::ConstantScalarDiffusivity);
        return;
    }

    if (top_menu == MenuModel::TopMenu::Sensors) {
        draw_panel_frame(window, font, panel, "Sensors");
        draw_setting_row(
            window, font, panel, 0, "Sample Period (s)",
            editing && active_field == MenuModel::EditableField::SensorSamplePeriod ? menu_model.edit_buffer() + "_"
                                                                                      : display_field_string(
                                                                                            pending,
                                                                                            MenuModel::EditableField::
                                                                                                SensorSamplePeriod),
            "-", "+", editing && active_field == MenuModel::EditableField::SensorSamplePeriod);
        draw_setting_row(
            window, font, panel, 1, "Noise Std", editing && active_field == MenuModel::EditableField::SensorNoiseStd
                ? menu_model.edit_buffer() + "_"
                : display_field_string(pending, MenuModel::EditableField::SensorNoiseStd),
            "-", "+", editing && active_field == MenuModel::EditableField::SensorNoiseStd);
        draw_setting_row(
            window, font, panel, 2, "History Capacity",
            editing && active_field == MenuModel::EditableField::SensorHistoryCapacity ? menu_model.edit_buffer() + "_"
                                                                                         : display_field_string(
                                                                                               pending,
                                                                                               MenuModel::EditableField::
                                                                                                   SensorHistoryCapacity),
            "-", "+", editing && active_field == MenuModel::EditableField::SensorHistoryCapacity);
        return;
    }

    if (top_menu == MenuModel::TopMenu::Display) {
        draw_panel_frame(window, font, panel, "Display");
        draw_setting_row(
            window, font, panel, 0, "PDE Fixed Color Scale",
            editing && active_field == MenuModel::EditableField::PdeFixedColorScale ? menu_model.edit_buffer() + "_"
                                                                                      : display_field_string(
                                                                                            pending,
                                                                                            MenuModel::EditableField::
                                                                                                PdeFixedColorScale),
            "-", "+", editing && active_field == MenuModel::EditableField::PdeFixedColorScale);
        return;
    }

    if (top_menu == MenuModel::TopMenu::Pde) {
        draw_panel_frame(window, font, panel, "PDE");
        draw_setting_row(
            window, font, panel, 0, "Grid Nx", editing && active_field == MenuModel::EditableField::PdeGridNx
                ? menu_model.edit_buffer() + "_"
                : display_field_string(pending, MenuModel::EditableField::PdeGridNx),
            "-", "+", editing && active_field == MenuModel::EditableField::PdeGridNx);
        draw_setting_row(
            window, font, panel, 1, "Grid Ny", editing && active_field == MenuModel::EditableField::PdeGridNy
                ? menu_model.edit_buffer() + "_"
                : display_field_string(pending, MenuModel::EditableField::PdeGridNy),
            "-", "+", editing && active_field == MenuModel::EditableField::PdeGridNy);
        draw_setting_row(
            window, font, panel, 2, "dt", editing && active_field == MenuModel::EditableField::Dt
                ? menu_model.edit_buffer() + "_"
                : display_field_string(pending, MenuModel::EditableField::Dt),
            "-", "+", editing && active_field == MenuModel::EditableField::Dt);
        draw_setting_row(
            window, font, panel, 3, "Diffusion Mode",
            editing && active_field == MenuModel::EditableField::PdeDiffusionMode ? menu_model.edit_buffer() + "_"
                                                                                    : display_field_string(
                                                                                          pending,
                                                                                          MenuModel::EditableField::
                                                                                              PdeDiffusionMode),
            "<", ">", editing && active_field == MenuModel::EditableField::PdeDiffusionMode);
        return;
    }
}

TopToolbarClickResult TopToolbar::handle_click(
    const sf::Vector2i& pixel, MenuModel& menu_model, SimulationController& controller) const {
    TopToolbarClickResult out;
    const sf::Vector2f p(static_cast<float>(pixel.x), static_cast<float>(pixel.y));
    const bool had_open_menu = menu_model.active_top_menu() != MenuModel::TopMenu::None;

    for (int i = 0; i < static_cast<int>(kTopMenus.size()); ++i) {
        if (!top_button_rect(i).contains(p)) {
            continue;
        }

        if (menu_model.active_top_menu() == MenuModel::TopMenu::None && !menu_model.dirty()) {
            menu_model.sync_from_current(controller.current_settings());
        }

        menu_model.toggle_top_menu(kTopMenus[static_cast<std::size_t>(i)].menu);
        out.consumed = true;
        return out;
    }

    const MenuModel::TopMenu active_menu = menu_model.active_top_menu();
    if (active_menu == MenuModel::TopMenu::None) {
        if (p.y <= kToolbarHeight) {
            out.consumed = true;
        }
        return out;
    }

    const sf::FloatRect panel = menu_panel_rect(active_menu);
    if (!panel.contains(p)) {
        menu_model.close_all();
        out.consumed = true;
        return out;
    }

    if (active_menu == MenuModel::TopMenu::File) {
        if (file_action_rect(panel, 0).contains(p)) {
            out.request_source_estimate = true;
            out.consumed = true;
            menu_model.close_all();
            return out;
        }
        if (file_action_rect(panel, 1).contains(p)) {
            out.request_apply_queued_settings = true;
            out.consumed = true;
            menu_model.close_all();
            return out;
        }
        if (file_action_rect(panel, 2).contains(p)) {
            out.request_revert_queued_settings = true;
            out.consumed = true;
            menu_model.close_all();
            return out;
        }
        if (file_action_rect(panel, 3).contains(p)) {
            out.request_restore_defaults = true;
            out.consumed = true;
            menu_model.close_all();
            return out;
        }
        out.consumed = true;
        return out;
    }

    if (active_menu == MenuModel::TopMenu::Source) {
        if (handle_setting_row_click(p, panel, 0, MenuModel::EditableField::SourceBaseEmission, menu_model)
            || handle_setting_row_click(p, panel, 1, MenuModel::EditableField::SourceDecayRate, menu_model)
            || handle_setting_row_click(p, panel, 2, MenuModel::EditableField::SourceLifespan, menu_model)
            || handle_setting_row_click(p, panel, 3, MenuModel::EditableField::SourceSigma, menu_model)
            || handle_setting_row_click(p, panel, 4, MenuModel::EditableField::SourceMaxSources, menu_model)) {
            out.consumed = true;
            return out;
        }
    } else if (active_menu == MenuModel::TopMenu::Numerics) {
        if (handle_setting_row_click(p, panel, 0, MenuModel::EditableField::TimeScale, menu_model)
            || handle_setting_row_click(p, panel, 1, MenuModel::EditableField::MaxParticles, menu_model)
            || handle_setting_row_click(p, panel, 2, MenuModel::EditableField::DepositionRate, menu_model)
            || handle_setting_row_click(p, panel, 3, MenuModel::EditableField::ConstantScalarDiffusivity, menu_model)) {
            out.consumed = true;
            return out;
        }
    } else if (active_menu == MenuModel::TopMenu::Sensors) {
        if (handle_setting_row_click(p, panel, 0, MenuModel::EditableField::SensorSamplePeriod, menu_model)
            || handle_setting_row_click(p, panel, 1, MenuModel::EditableField::SensorNoiseStd, menu_model)
            || handle_setting_row_click(p, panel, 2, MenuModel::EditableField::SensorHistoryCapacity, menu_model)) {
            out.consumed = true;
            return out;
        }
    } else if (active_menu == MenuModel::TopMenu::Display) {
        if (handle_setting_row_click(p, panel, 0, MenuModel::EditableField::PdeFixedColorScale, menu_model)) {
            out.consumed = true;
            return out;
        }
    } else if (active_menu == MenuModel::TopMenu::Pde) {
        if (handle_setting_row_click(p, panel, 0, MenuModel::EditableField::PdeGridNx, menu_model)
            || handle_setting_row_click(p, panel, 1, MenuModel::EditableField::PdeGridNy, menu_model)
            || handle_setting_row_click(p, panel, 2, MenuModel::EditableField::Dt, menu_model)
            || handle_setting_row_click(p, panel, 3, MenuModel::EditableField::PdeDiffusionMode, menu_model)) {
            out.consumed = true;
            return out;
        }
    }

    out.consumed = true;
    if (had_open_menu) {
        return out;
    }
    return out;
}

bool TopToolbar::handle_text_input(char32_t unicode, MenuModel& menu_model) const {
    if (!menu_model.editing()) {
        return false;
    }

    if (unicode >= 32) {
        menu_model.append_edit_char(unicode);
    }
    return true;
}

bool TopToolbar::handle_key_input(sf::Keyboard::Key key, MenuModel& menu_model) const {
    if (menu_model.editing()) {
        if (key == sf::Keyboard::Key::Enter) {
            menu_model.commit_edit();
            return true;
        }
        if (key == sf::Keyboard::Key::Backspace) {
            menu_model.backspace_edit_char();
            return true;
        }
        if (key == sf::Keyboard::Key::Escape) {
            menu_model.cancel_edit();
            return true;
        }
        return true;
    }

    if (key == sf::Keyboard::Key::Escape && menu_model.active_top_menu() != MenuModel::TopMenu::None) {
        menu_model.close_all();
        return true;
    }
    return false;
}

bool TopToolbar::has_open_menu(const MenuModel& menu_model) const {
    return menu_model.active_top_menu() != MenuModel::TopMenu::None;
}

} // namespace atm
