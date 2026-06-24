#include "io/SiteLoader.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <unordered_map>

namespace atm {

namespace {

std::string trim(const std::string& s) {
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return {};
    return s.substr(b, s.find_last_not_of(" \t\r\n") - b + 1);
}

int column_index(const std::vector<std::string>& headers, const std::string& name) {
    for (int i = 0; i < static_cast<int>(headers.size()); ++i) {
        if (trim(headers[static_cast<std::size_t>(i)]) == name) return i;
    }
    return -1;
}

std::vector<std::string> split_csv_row(const std::string& line) {
    std::vector<std::string> cols;
    std::string field;
    bool in_quotes = false;
    for (char c : line) {
        if (c == '"') { in_quotes = !in_quotes; }
        else if (c == ',' && !in_quotes) { cols.push_back(field); field.clear(); }
        else { field += c; }
    }
    cols.push_back(field);
    return cols;
}

} // namespace

std::vector<SiteRecord> load_unique_sites(const std::string& csv_path, std::string& error_msg) {
    std::ifstream f(csv_path);
    if (!f.is_open()) {
        error_msg = "Cannot open: " + csv_path;
        return {};
    }

    std::string header_line;
    if (!std::getline(f, header_line)) {
        error_msg = "Empty file: " + csv_path;
        return {};
    }

    const auto headers = split_csv_row(header_line);
    const int col_name = column_index(headers, "site_name");
    const int col_lat  = column_index(headers, "lat_deg");
    const int col_lon  = column_index(headers, "lon_deg");

    if (col_name < 0 || col_lat < 0 || col_lon < 0) {
        error_msg = "Missing columns (need site_name, lat_deg, lon_deg)";
        return {};
    }

    std::unordered_map<std::string, SiteRecord> seen;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        const auto cols = split_csv_row(line);
        const int ncols = static_cast<int>(cols.size());
        if (col_name >= ncols || col_lat >= ncols || col_lon >= ncols) continue;

        const std::string name = trim(cols[static_cast<std::size_t>(col_name)]);
        const std::string lat_s = trim(cols[static_cast<std::size_t>(col_lat)]);
        const std::string lon_s = trim(cols[static_cast<std::size_t>(col_lon)]);
        if (name.empty() || lat_s.empty() || lon_s.empty()) continue;
        if (seen.count(name)) continue;

        try {
            const float lat = std::stof(lat_s);
            const float lon = std::stof(lon_s);
            seen[name] = SiteRecord{name, lat, lon};
        } catch (...) {
            continue;
        }
    }

    std::vector<SiteRecord> out;
    out.reserve(seen.size());
    for (auto& [k, v] : seen) out.push_back(std::move(v));
    std::sort(out.begin(), out.end(), [](const SiteRecord& a, const SiteRecord& b) {
        return a.name < b.name;
    });

    if (out.empty()) {
        error_msg = "No valid sites found in " + csv_path;
    }
    return out;
}

std::vector<Vec2> project_sites_to_domain(
    const std::vector<SiteRecord>& sites,
    const DomainConfig& domain,
    float pad_fraction,
    ProjectionParams* out_params)
{
    if (sites.empty()) return {};

    // Centroid
    float lat0 = 0.0f, lon0 = 0.0f;
    for (const auto& s : sites) { lat0 += s.lat_deg; lon0 += s.lon_deg; }
    lat0 /= static_cast<float>(sites.size());
    lon0 /= static_cast<float>(sites.size());

    // Equirectangular projection (meters, centroid as origin)
    constexpr float R = 6371000.0f;
    constexpr float deg2rad = 3.14159265358979323846f / 180.0f;
    const float cos_lat0 = std::cos(lat0 * deg2rad);

    std::vector<Vec2> proj;
    proj.reserve(sites.size());
    for (const auto& s : sites) {
        const float xm = (s.lon_deg - lon0) * cos_lat0 * R * deg2rad;
        const float ym = -(s.lat_deg - lat0) * R * deg2rad;
        proj.push_back(Vec2(xm, ym));
    }

    // Bounding box of projected positions
    float xmin = proj[0].x(), xmax = proj[0].x();
    float ymin = proj[0].y(), ymax = proj[0].y();
    for (const auto& p : proj) {
        xmin = std::min(xmin, p.x()); xmax = std::max(xmax, p.x());
        ymin = std::min(ymin, p.y()); ymax = std::max(ymax, p.y());
    }

    // Uniform scale: fit larger axis within available domain minus padding
    const float domain_w = domain.x_max - domain.x_min;
    const float domain_h = domain.y_max - domain.y_min;
    const float avail_w = domain_w * (1.0f - 2.0f * pad_fraction);
    const float avail_h = domain_h * (1.0f - 2.0f * pad_fraction);
    const float data_w  = xmax - xmin;
    const float data_h  = ymax - ymin;

    float scale = 1.0f;
    if (data_w > 1.0e-3f && data_h > 1.0e-3f) {
        scale = std::min(avail_w / data_w, avail_h / data_h);
    } else if (data_w > 1.0e-3f) {
        scale = avail_w / data_w;
    } else if (data_h > 1.0e-3f) {
        scale = avail_h / data_h;
    }

    // Center of data bbox maps to center of domain
    const float cx_data   = 0.5f * (xmin + xmax);
    const float cy_data   = 0.5f * (ymin + ymax);
    const float cx_domain = 0.5f * (domain.x_min + domain.x_max);
    const float cy_domain = 0.5f * (domain.y_min + domain.y_max);

    if (out_params) {
        out_params->lat0_deg  = lat0;
        out_params->lon0_deg  = lon0;
        out_params->cos_lat0  = cos_lat0;
        out_params->scale     = scale;
        out_params->cx_domain = cx_domain;
        out_params->cy_domain = cy_domain;
        out_params->cx_data   = cx_data;
        out_params->cy_data   = cy_data;
    }

    std::vector<Vec2> out;
    out.reserve(proj.size());
    for (const auto& p : proj) {
        out.push_back(Vec2(
            cx_domain + scale * (p.x() - cx_data),
            cy_domain + scale * (p.y() - cy_data)));
    }
    return out;
}

} // namespace atm
