#pragma once

#include "core/Config.hpp"

#include <string>
#include <vector>

namespace atm {

std::vector<std::string> validate_config(const Config& config);

} // namespace atm
