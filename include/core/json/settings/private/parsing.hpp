#pragma once
#include "simulation/actuators/public/manager.hpp"
#include "simulation/settings/public/data/types.hpp"

#include <nlohmann/json.hpp>

namespace json
{

	settings::Settings parse_settings(const nlohmann::json& config, const actuators::ActuatorManager& actuator_manager);

}
