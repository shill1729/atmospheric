#pragma once

#include "core/Config.hpp"
#include "core/Types.hpp"

namespace atm {

class Fields {
public:
    explicit Fields(const DomainConfig& domain);

    Vec2 wind(float time_s, const Vec2& x) const;
    Mat2 diffusivity(float time_s, const Vec2& x) const;
    Vec2 div_diffusivity(float time_s, const Vec2& x) const;

private:
    const DomainConfig& domain_;
};

} // namespace atm
