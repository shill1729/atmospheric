#include "core/RuntimeSettings.hpp"

namespace atm {

RuntimeSettings make_runtime_settings(const Config& config, AdvectionDiffusionSolver::DiffusionMode mode) {
    RuntimeSettings out;
    out.pde_grid_nx = config.domain.nx;
    out.pde_grid_ny = config.domain.ny;
    out.dt = config.numerics.dt;
    out.pde_diffusion_mode = mode;
    return out;
}

bool same_runtime_settings(const RuntimeSettings& a, const RuntimeSettings& b) {
    return a.pde_grid_nx == b.pde_grid_nx && a.pde_grid_ny == b.pde_grid_ny && a.dt == b.dt
        && a.pde_diffusion_mode == b.pde_diffusion_mode;
}

} // namespace atm
