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
        Cellular = 2
    };

    explicit Fields(const DomainConfig& domain);

    Vec2 wind(float time_s, const Vec2& x) const;
    Mat2 diffusivity(float time_s, const Vec2& x) const;
    Vec2 div_diffusivity(float time_s, const Vec2& x) const;
    void cycle_wind_preset(int direction);
    WindPreset wind_preset() const;
    std::string_view wind_preset_name() const;

private:
    Vec2 wind_jet_shear(float time_s, const Vec2& x) const;
    Vec2 wind_vortex_pair(float time_s, const Vec2& x) const;
    Vec2 wind_cellular(float time_s, const Vec2& x) const;

    const DomainConfig& domain_;
    WindPreset wind_preset_ = WindPreset::JetShear;
};

} // namespace atm
