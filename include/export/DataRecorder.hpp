#pragma once

#include "adjoint/SourceEstimator.hpp"
#include "core/Types.hpp"

#include <ctime>
#include <string>
#include <vector>

namespace atm {

class DataRecorder {
public:
    struct Reading {
        float time_s;
        float concentration_ug_m3;
        float wind_u;
        float wind_v;
    };

    struct SensorRecord {
        Vec2 position;
        std::string label;
        std::vector<Reading> readings;
    };

    struct SourceEvent {
        Vec2 position;
        float birth_time_s;
        float lifespan_s;
        float death_time_s;
    };

    // Equirectangular projection from the NY site network (see SiteLoader),
    // used by export_csv() to back-project domain (x,y) to (lat, lon).
    struct GeoProjection {
        float lat0_deg  = 0.0f;
        float lon0_deg  = 0.0f;
        float cos_lat0  = 1.0f;
        float scale     = 1.0f;
        float cx_domain = 0.0f;
        float cy_domain = 0.0f;
        float cx_data   = 0.0f;
        float cy_data   = 0.0f;
    };

    bool is_recording() const;
    bool has_data() const;
    int total_readings() const;

    void start_recording();
    void stop_recording();
    void clear();

    void record_source_event(const Vec2& position, float birth_time_s, float lifespan_s);

    void add_reading(
        std::size_t sensor_index,
        const Vec2& position,
        const std::string& label,
        float time_s,
        float raw_concentration,
        float conc_scale_ug_per_m2,
        float mixing_height_m,
        const Vec2& wind_uv);

    std::string export_csv(
        const std::string& output_dir,
        const std::string& wind_model,
        const std::string& diffusion_model,
        const std::string& pde_mode,
        float dt,
        float time_scale,
        float sensor_sample_period_s,
        float wind_scale,
        const std::vector<SourceEstimateResult>& estimation_results = {}) const;

    void set_geo_projection(const GeoProjection& p);
    void clear_geo_projection();
    bool has_geo_projection() const;

    const std::vector<SensorRecord>& records() const;
    const std::vector<SourceEvent>& source_events() const;

private:
    bool recording_ = false;
    std::vector<SensorRecord> records_;
    std::vector<SourceEvent> source_events_;
    bool has_geo_projection_ = false;
    GeoProjection geo_projection_ {};
    // Wall-clock time recording started; the Datetime_UTC anchor when no geo
    // projection is set.
    std::time_t recording_start_wall_ = 0;
};

} // namespace atm
