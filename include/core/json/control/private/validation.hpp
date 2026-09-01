#pragma once
#include <nlohmann/json.hpp>

namespace json {

    void validate_controllers(const nlohmann::json& controllers_json, bool trim_flag);

}
