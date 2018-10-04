#pragma once

#ifdef LOCAL_RUN
#include <json.hpp>
#else
#include "../../../nlohmann/json.hpp"
#endif

using json = nlohmann::json;
