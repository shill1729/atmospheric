#pragma once

#include "core/Config.hpp"
#include "core/Types.hpp"

#include <string>
#include <string_view>
#include <vector>

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
        SolidBodyRotation = 6,
        VeeringUniform = 7,
        JenningsReplay = 8
    };

    // One sample of the measured network-mean wind (real m/s, v northward).
    struct ObservedWind {
        float real_time_s = 0.0f;
        float u_east = 0.0f;
        float v_north = 0.0f;
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
    void set_wind_scale(float scale);
    float wind_scale() const;
    // Real seconds per simulation second, for presets defined in real time
    // (Veering Uniform, Jennings Replay); see DataRecorder::time_stretch.
    void set_time_stretch(float stretch);
    // Enables the Jennings Replay preset; an empty series disables it.
    void set_observed_wind(std::vector<ObservedWind> series);
    bool has_observed_wind() const;
    void cycle_wind_preset(int direction);
    WindPreset wind_preset() const;
    void set_wind_preset(WindPreset preset);
    std::string_view wind_preset_name() const;
    // Preset name plus its parameters, for export headers.
    std::string wind_preset_details() const;
    // Accepts names like "jet-shear", "veering-uniform", "jennings-replay".
    static bool wind_preset_from_name(std::string name, WindPreset& out);
    void cycle_diffusivity_preset(int direction);
    DiffusivityPreset diffusivity_preset() const;
    void set_diffusivity_preset(DiffusivityPreset preset);
    std::string_view diffusivity_preset_name() const;

private:
    Vec2 preset_wind(float time_s, const Vec2& x) const;
    Vec2 wind_jet_shear(float time_s, const Vec2& x) const;
    Vec2 wind_vortex_pair(float time_s, const Vec2& x) const;
    Vec2 wind_shear_vortex_blend(float time_s, const Vec2& x) const;
    Vec2 wind_cellular(float time_s, const Vec2& x) const;
    Vec2 wind_uniform() const;
    Vec2 wind_solid_body_rotation(const Vec2& x) const;
    Vec2 wind_veering_uniform(float time_s, const Vec2& x) const;
    Vec2 wind_jennings_replay(float time_s) const;

    const DomainConfig& domain_;
    float constant_scalar_diffusivity_ = 22.0f;
    float wind_scale_ = 1.0f;
    float time_stretch_ = 1.0f;
    PhysicsConfig::VeeringWind veering_;
    std::vector<ObservedWind> observed_wind_;
    WindPreset wind_preset_ = WindPreset::JetShear;
    DiffusivityPreset diffusivity_preset_ = DiffusivityPreset::ConstantScalar;
};

} // namespace atm
