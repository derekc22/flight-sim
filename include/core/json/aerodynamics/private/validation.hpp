#pragma once
#include <nlohmann/json.hpp>

namespace json {

    void validate_surfaces_json(const nlohmann::json& surfaces_json);

}
