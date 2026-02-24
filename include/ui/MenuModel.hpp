#pragma once

#include "core/RuntimeSettings.hpp"

#include <string>

namespace atm {

class MenuModel {
public:
    enum class TopMenu {
        None,
        File,
        Source,
        Numerics,
        Sensors,
        Display,
        Pde
    };

    enum class EditableField {
        None,
        TimeScale,
        MaxParticles,
        DepositionRate,
        ConstantScalarDiffusivity,
        SourceBaseEmission,
        SourceDecayRate,
        SourceLifespan,
        SourceSigma,
        SourceMaxSources,
        PdeFixedColorScale,
        SensorSamplePeriod,
        SensorNoiseStd,
        SensorHistoryCapacity,
        PdeGridNx,
        PdeGridNy,
        Dt,
        PdeDiffusionMode
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
    void adjust_sensor_history_capacity(int delta);

    void adjust_pde_grid_nx(int delta);
    void adjust_pde_grid_ny(int delta);
    void adjust_dt(float delta);
    void cycle_pde_diffusion_mode(int direction);
    void discard_changes();
    void commit_pending_as_current();

    void start_edit(EditableField field, const std::string& initial_text);
    bool editing() const;
    EditableField active_edit_field() const;
    const std::string& edit_buffer() const;
    void backspace_edit_char();
    void append_edit_char(char32_t unicode);
    bool commit_edit();
    void cancel_edit();

private:
    void stop_editing();
    static float clampf(float v, float lo, float hi);
    static int clampi(int v, int lo, int hi);

    RuntimeSettings current_;
    RuntimeSettings pending_;
    TopMenu active_top_menu_ = TopMenu::None;
    EditableField active_edit_field_ = EditableField::None;
    std::string edit_buffer_;
};

} // namespace atm
