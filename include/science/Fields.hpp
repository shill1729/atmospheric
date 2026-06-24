#pragma once

#include "core/Config.hpp"
#include "core/Types.hpp"

#include <string_view>

namespace atm {

class Fields {
public:
    enum class WindPreset {
        JetShear = 0,
        VortexPair = 1,
        ShearVortexBlend = 2,
        Cellular = 3,
        Zero = 4,
        Uniform = 5,
        SolidBodyRotation = 6
    };
    enum class DiffusivityPreset {
        ConstantScalar = 0,
        SpatialScalar = 1,
        ConstantTensor = 2,
        DiagonalTensor = 3,
        FullAnisotropicTensor = 4,
        BrownianHalf = 5
    };

    Fields(const DomainConfig& domain, const PhysicsConfig& physics);

    Vec2 wind(float time_s, const Vec2& x) const;
    Mat2 diffusivity(float time_s, const Vec2& x) const;
    Vec2 div_diffusivity(float time_s, const Vec2& x) const;
    float scalar_diffusivity(float time_s, const Vec2& x) const;
    Vec2 grad_scalar_diffusivity(float time_s, const Vec2& x) const;
    void cycle_wind_preset(int direction);
    WindPreset wind_preset() const;
    void set_wind_preset(WindPreset preset);
    std::string_view wind_preset_name() const;
    void cycle_diffusivity_preset(int direction);
    DiffusivityPreset diffusivity_preset() const;
    void set_diffusivity_preset(DiffusivityPreset preset);
    std::string_view diffusivity_preset_name() const;

private:
    Vec2 wind_jet_shear(float time_s, const Vec2& x) const;
    Vec2 wind_vortex_pair(float time_s, const Vec2& x) const;
    Vec2 wind_shear_vortex_blend(float time_s, const Vec2& x) const;
    Vec2 wind_cellular(float time_s, const Vec2& x) const;
    Vec2 wind_uniform() const;
    Vec2 wind_solid_body_rotation(const Vec2& x) const;

    const DomainConfig& domain_;
    float constant_scalar_diffusivity_ = 22.0f;
    WindPreset wind_preset_ = WindPreset::JetShear;
    DiffusivityPreset diffusivity_preset_ = DiffusivityPreset::ConstantScalar;
};

} // namespace atm
