#pragma once

#include "core/Config.hpp"
#include "core/Types.hpp"

namespace atm {

class SourceModel {
public:
    SourceModel(const DomainConfig& domain, const SourceConfig& source);

    void activate(const Vec2& position);
    void deactivate();
    void step(float dt);

    bool is_active() const;
    float age_s() const;
    float lifespan_s() const;
    float emission_rate() const;
    float source_density(const Vec2& x) const;
    const Vec2& position() const;

private:
    const DomainConfig& domain_;
    const SourceConfig& source_;
    Vec2 position_;
    float age_ = 0.0f;
    bool active_ = false;
};

} // namespace atm
