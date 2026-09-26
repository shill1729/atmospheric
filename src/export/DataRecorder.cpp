#include "export/DataRecorder.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
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
    source_events_.clear();
    recording_ = true;
    recording_start_wall_ = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
}

void DataRecorder::stop_recording() {
    recording_ = false;
}

void DataRecorder::clear() {
    records_.clear();
    source_events_.clear();
    recording_ = false;
}

void DataRecorder::record_source_event(const Vec2& position, float birth_time_s, float lifespan_s) {
    source_events_.push_back({position, birth_time_s, lifespan_s, birth_time_s + lifespan_s});
}

const std::vector<DataRecorder::SourceEvent>& DataRecorder::source_events() const {
    return source_events_;
}

void DataRecorder::add_reading(
    std::size_t sensor_index,
    const Vec2& position,
    const std::string& label,
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
    rec.label = label;
    const float conc_ug_m3 = raw_concentration * conc_scale_ug_per_m2 / std::max(1.0e-6f, mixing_height_m);
    rec.readings.push_back({time_s, conc_ug_m3, wind_uv.x(), wind_uv.y()});
}

const std::vector<DataRecorder::SensorRecord>& DataRecorder::records() const {
    return records_;
}

void DataRecorder::set_geo_projection(const GeoProjection& p) {
    geo_projection_ = p;
    has_geo_projection_ = true;
}

void DataRecorder::clear_geo_projection() {
    has_geo_projection_ = false;
    geo_projection_ = {};
}

bool DataRecorder::has_geo_projection() const {
    return has_geo_projection_;
}

float DataRecorder::time_stretch() const {
    return has_geo_projection_ && geo_projection_.scale > 0.0f ? 1.0f / geo_projection_.scale : 1.0f;
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
    float sensor_sample_period_s,
    float wind_scale,
    const std::vector<SourceEstimateResult>& estimation_results) const
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

    std::error_code dir_error;
    std::filesystem::create_directories(output_dir, dir_error);
    const std::string path = (std::filesystem::path(output_dir) / fn.str()).string();

    std::ofstream out(path);
    if (!out.is_open()) {
        return {};
    }

    constexpr float R_EARTH = 6371000.0f;
    constexpr float DEG2RAD = 3.14159265358979323846f / 180.0f;

    auto domain_to_latlon = [&](float x, float y, float& lat, float& lon) {
        const float xm = (x - geo_projection_.cx_domain) / geo_projection_.scale + geo_projection_.cx_data;
        const float ym = (y - geo_projection_.cy_domain) / geo_projection_.scale + geo_projection_.cy_data;
        lat = geo_projection_.lat0_deg - ym / (R_EARTH * DEG2RAD);
        lon = geo_projection_.lon0_deg + xm / (geo_projection_.cos_lat0 * R_EARTH * DEG2RAD);
    };

    // Datetime_UTC anchor: the real wildfire event's start when the domain is
    // georeferenced to the NY network, otherwise the recording start time.
    std::tm wildfire_start_tm{};
    wildfire_start_tm.tm_year = 2024 - 1900;
    wildfire_start_tm.tm_mon = 11 - 1;
    wildfire_start_tm.tm_mday = 8;
#ifdef _WIN32
    const std::time_t wildfire_start_utc = _mkgmtime(&wildfire_start_tm);
#else
    const std::time_t wildfire_start_utc = timegm(&wildfire_start_tm);
#endif
    const std::time_t datetime_anchor = has_geo_projection_ ? wildfire_start_utc : recording_start_wall_;

    const float stretch = time_stretch();
    auto format_datetime_utc = [&](float time_s_val) {
        const std::time_t t = datetime_anchor + static_cast<std::time_t>(std::lround(time_s_val * stretch));
        std::tm tm_utc{};
#ifdef _WIN32
        gmtime_s(&tm_utc, &t);
#else
        gmtime_r(&t, &tm_utc);
#endif
        std::ostringstream oss;
        oss << std::put_time(&tm_utc, "%Y-%m-%d %H:%M:%S") << "+00:00";
        return oss.str();
    };
    // ISO 8601 with a 'T' separator, for space-free key=value header fields.
    auto format_iso_utc = [&](float time_s_val) {
        std::string s = format_datetime_utc(time_s_val);
        s[10] = 'T';
        return s;
    };

    out << std::fixed << std::setprecision(6);
    out << "# Atmospheric Tool - Forward Simulation Export\n"
        << "# Wind model: " << wind_model << "\n"
        << "# Wind scale: " << wind_scale << "\n"
        << "# Diffusion model: " << diffusion_model << "\n"
        << "# PDE diffusion mode: " << pde_mode << "\n"
        << "# dt (s): " << dt << "\n"
        << "# Time scale: " << time_scale << "\n"
        << "# Sensor averaging window (s): " << sensor_sample_period_s << "\n"
        << "# Sensors recorded: " << records_.size() << "\n"
        << "# Concentration: sensor window-averaged reading converted to ug/m^3\n"
        << "# Coordinates: "
        << (has_geo_projection_
            ? "lat_deg, lon_deg (WGS84, equirectangular back-projection from domain)\n"
            : "x_m, y_m (simulation domain, metres)\n")
        << "# Wind: "
        << (has_geo_projection_
            ? "u eastward, v northward (m/s); wind_direction is where the wind blows from, degrees clockwise from north\n"
            : "u along +x, v along +y (m/s, domain axes); wind_direction = atan2(-u, -v) in degrees\n")
        << "# Time stretch: " << stretch
        << (has_geo_projection_
            ? " real s per simulation s (Datetime_UTC = anchor + time_s * stretch; the domain is a 1:"
                + std::to_string(static_cast<int>(std::lround(stretch))) + " scale model of the network)\n"
            : " (no geo projection)\n")
        << "# Sources: " << source_events_.size() << "\n";
    for (std::size_t i = 0; i < source_events_.size(); ++i) {
        const auto& se = source_events_[i];
        out << "# Source " << (i + 1) << ":";
        if (has_geo_projection_) {
            float slat, slon;
            domain_to_latlon(se.position.x(), se.position.y(), slat, slon);
            out << " lat=" << slat << " lon=" << slon;
        } else {
            out << " x=" << se.position.x() << " y=" << se.position.y();
        }
        out << " born=" << se.birth_time_s << "s"
            << " lifespan=" << se.lifespan_s << "s"
            << " died=" << se.death_time_s << "s"
            << " born_utc=" << format_iso_utc(se.birth_time_s)
            << " died_utc=" << format_iso_utc(se.death_time_s) << "\n";
    }

    out << "# Source term estimates (from the last 'E' run before export): " << estimation_results.size() << "\n";
    for (std::size_t i = 0; i < estimation_results.size(); ++i) {
        const auto& r = estimation_results[i];
        out << "# Estimate " << (i + 1) << " (" << source_estimation_method_name(r.method) << "): ";
        if (!r.success || r.insufficient_signal) {
            out << "no result - " << r.message << "\n";
            continue;
        }
        if (has_geo_projection_) {
            float slat, slon;
            domain_to_latlon(r.x_star.x(), r.x_star.y(), slat, slon);
            out << "lat*=" << slat << " lon*=" << slon;
        } else {
            out << "x*=" << r.x_star.x() << " y*=" << r.x_star.y();
        }
        out << " t*=" << r.t_star_s << "s t*_utc=" << format_iso_utc(r.t_star_s);
        // Distance to the nearest recorded source, in the exported frame
        // (real metres when georeferenced, domain metres otherwise).
        if (!source_events_.empty()) {
            std::size_t nearest = 0;
            float best = std::numeric_limits<float>::infinity();
            for (std::size_t k = 0; k < source_events_.size(); ++k) {
                const float dist = (r.x_star - source_events_[k].position).norm();
                if (dist < best) {
                    best = dist;
                    nearest = k;
                }
            }
            out << " err_m=" << best * stretch << " nearest_source=" << (nearest + 1);
        }
        if (r.method != SourceEstimationMethod::AdjointBacktracking) {
            out << " q0=" << r.q0 << " q0_std=" << r.q0_std << " x_std_m=" << r.x_std_m * stretch << " y_std_m=" << r.y_std_m * stretch
                << " t_std_s=" << r.t_std_s << " weighted_rmse=" << r.weighted_rmse
                << " weakly_identified=" << (r.weakly_identified ? "true" : "false")
                << " sensors_used=" << r.sensors_used << " observations_used=" << r.observations_used;
        }
        out << "\n";
    }

    // Column order matches wildfire_pm25_dataset.csv, with time_s appended.
    out << "Datetime_UTC,site_name," << (has_geo_projection_ ? "lat_deg,lon_deg" : "x_m,y_m")
        << ",pm25_ugm-3,wind_speed,wind_direction,wind_u_component,wind_v_component,time_s\n";

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
        const std::string label = rec.label.empty() ? ("Sensor_" + std::to_string(e.sensor_idx)) : rec.label;
        out << format_datetime_utc(r.time_s) << "," << label << ",";
        if (has_geo_projection_) {
            float lat, lon;
            domain_to_latlon(rec.position.x(), rec.position.y(), lat, lon);
            out << lat << "," << lon;
        } else {
            out << rec.position.x() << "," << rec.position.y();
        }
        // Domain +y points south (screen-down; see project_sites_to_domain),
        // so the northward component is -wind_v when exporting lat/lon.
        const float u = r.wind_u;
        const float v = has_geo_projection_ ? -r.wind_v : r.wind_v;
        const float speed = std::hypot(u, v);
        // Meteorological convention, as in the real datasets: direction the wind blows from.
        const float direction = std::fmod(std::atan2(-u, -v) * 180.0f / 3.14159265358979323846f + 360.0f, 360.0f);
        out << "," << r.concentration_ug_m3
            << "," << speed
            << "," << direction
            << "," << u
            << "," << v
            << "," << r.time_s << "\n";
    }

    return path;
}

} // namespace atm
