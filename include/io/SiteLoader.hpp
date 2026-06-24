#pragma once

#include "core/Config.hpp"
#include "core/Types.hpp"

#include <string>
#include <vector>

namespace atm {

struct SiteRecord {
    std::string name;
    float lat_deg;
    float lon_deg;
};

// Parameters of the equirectangular projection used by project_sites_to_domain.
// Retained so the CSV exporter can reverse-project domain (x,y) back to lat/lon.
struct ProjectionParams {
    float lat0_deg  = 0.0f;
    float lon0_deg  = 0.0f;
    float cos_lat0  = 1.0f;
    float scale     = 1.0f;   // domain units per projected metre
    float cx_domain = 0.0f;   // domain-centre x (m)
    float cy_domain = 0.0f;   // domain-centre y (m)
    float cx_data   = 0.0f;   // bbox-centre of projected network, pre-scale (m from centroid)
    float cy_data   = 0.0f;
};

// Parse unique site records from the wildfire PM2.5 CSV (one entry per unique site_name).
// Returns an empty vector and sets error_msg on failure.
std::vector<SiteRecord> load_unique_sites(const std::string& csv_path, std::string& error_msg);

// Equirectangular projection of sites to simulation domain coordinates.
// Computes centroid, projects to meters, then uniformly scales + centers so the
// network fits the domain with pad_fraction margin on each side.
// If out_params is non-null it is filled with the parameters needed for the inverse projection.
std::vector<Vec2> project_sites_to_domain(
    const std::vector<SiteRecord>& sites,
    const DomainConfig& domain,
    float pad_fraction = 0.08f,
    ProjectionParams* out_params = nullptr);

} // namespace atm
