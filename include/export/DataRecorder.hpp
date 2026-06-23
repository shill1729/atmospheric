#pragma once

#include "core/Types.hpp"

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
        std::vector<Reading> readings;
    };

    bool is_recording() const;
    bool has_data() const;
    int total_readings() const;

    void start_recording();
    void stop_recording();
    void clear();

    void add_reading(
        std::size_t sensor_index,
        const Vec2& position,
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
        float sensor_sample_period_s) const;

    const std::vector<SensorRecord>& records() const;

private:
    bool recording_ = false;
    std::vector<SensorRecord> records_;
};

} // namespace atm
