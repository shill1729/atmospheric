#include "export/DataRecorder.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace atm {

bool DataRecorder::is_recording() const {
    return recording_;
}

bool DataRecorder::has_data() const {
    for (const auto& rec : records_) {
        if (!rec.readings.empty()) {
            return true;
        }
    }
    return false;
}

int DataRecorder::total_readings() const {
    int n = 0;
    for (const auto& rec : records_) {
        n += static_cast<int>(rec.readings.size());
    }
    return n;
}

void DataRecorder::start_recording() {
    records_.clear();
    recording_ = true;
}

void DataRecorder::stop_recording() {
    recording_ = false;
}

void DataRecorder::clear() {
    records_.clear();
    recording_ = false;
}

void DataRecorder::add_reading(
    std::size_t sensor_index,
    const Vec2& position,
    float time_s,
    float raw_concentration,
    float conc_scale_ug_per_m2,
    float mixing_height_m,
    const Vec2& wind_uv)
{
    if (sensor_index >= records_.size()) {
        records_.resize(sensor_index + 1);
    }
    auto& rec = records_[sensor_index];
    rec.position = position;
    const float conc_ug_m3 = raw_concentration * conc_scale_ug_per_m2 / std::max(1.0e-6f, mixing_height_m);
    rec.readings.push_back({time_s, conc_ug_m3, wind_uv.x(), wind_uv.y()});
}

const std::vector<DataRecorder::SensorRecord>& DataRecorder::records() const {
    return records_;
}

namespace {

std::string sanitize_name(std::string s) {
    for (char& c : s) {
        if (c == ' ' || c == '/' || c == '\\' || c == ':' || c == '*' || c == '?') {
            c = '_';
        }
    }
    return s;
}

} // namespace

std::string DataRecorder::export_csv(
    const std::string& output_dir,
    const std::string& wind_model,
    const std::string& diffusion_model,
    const std::string& pde_mode,
    float dt,
    float time_scale,
    float sensor_sample_period_s) const
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
#ifdef _WIN32
    localtime_s(&tm_buf, &t_now);
#else
    localtime_r(&t_now, &tm_buf);
#endif

    std::ostringstream ts;
    ts << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");

    std::ostringstream fn;
    fn << "atmospheric"
       << "_" << sanitize_name(wind_model)
       << "_" << sanitize_name(diffusion_model)
       << "_" << sanitize_name(pde_mode)
       << "_dt" << std::fixed << std::setprecision(3) << dt
       << "_ts" << std::fixed << std::setprecision(1) << time_scale
       << "_sp" << std::fixed << std::setprecision(1) << sensor_sample_period_s << "s"
       << "_" << ts.str()
       << ".csv";

    const std::string path = output_dir + "/" + fn.str();

    std::ofstream out(path);
    if (!out.is_open()) {
        return {};
    }

    out << "# Atmospheric Tool - Forward Simulation Export\n"
        << "# Wind model: " << wind_model << "\n"
        << "# Diffusion model: " << diffusion_model << "\n"
        << "# PDE diffusion mode: " << pde_mode << "\n"
        << "# dt (s): " << dt << "\n"
        << "# Time scale: " << time_scale << "\n"
        << "# Sensor averaging window (s): " << sensor_sample_period_s << "\n"
        << "# Sensors recorded: " << records_.size() << "\n"
        << "# Concentration: sensor window-averaged reading converted to ug/m^3\n";

    out << "time_s,x_m,y_m,concentration_ug_m3,wind_u_m_s,wind_v_m_s\n";
    out << std::fixed << std::setprecision(6);

    struct Entry {
        float time_s;
        std::size_t sensor_idx;
        std::size_t reading_idx;
    };
    std::vector<Entry> entries;
    for (std::size_t si = 0; si < records_.size(); ++si) {
        for (std::size_t ri = 0; ri < records_[si].readings.size(); ++ri) {
            entries.push_back({records_[si].readings[ri].time_s, si, ri});
        }
    }
    std::sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b) {
        if (a.time_s != b.time_s) {
            return a.time_s < b.time_s;
        }
        return a.sensor_idx < b.sensor_idx;
    });

    for (const auto& e : entries) {
        const auto& rec = records_[e.sensor_idx];
        const auto& r = rec.readings[e.reading_idx];
        out << r.time_s << ","
            << rec.position.x() << ","
            << rec.position.y() << ","
            << r.concentration_ug_m3 << ","
            << r.wind_u << ","
            << r.wind_v << "\n";
    }

    return path;
}

} // namespace atm
