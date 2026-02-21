#pragma once

#include "core/Config.hpp"
#include "core/Types.hpp"

#include <vector>

namespace atm {

class SourceModel {
public:
    struct ActiveSource {
        Vec2 position;
        float age_s = 0.0f;
    };

    SourceModel(const DomainConfig& domain, const SourceConfig& source);

    void activate(const Vec2& position);
    void deactivate();
    void step(float dt);

    bool is_active() const;
    float newest_age_s() const;
    std::size_t active_count() const;
    std::size_t max_sources() const;
    float lifespan_s() const;
    float emission_rate() const;
    float emission_rate(const ActiveSource& source) const;
    float source_density(const Vec2& x) const;
    const std::vector<ActiveSource>& active_sources() const;

private:
    const DomainConfig& domain_;
    const SourceConfig& source_;
    std::vector<ActiveSource> active_sources_;
};

} // namespace atm
