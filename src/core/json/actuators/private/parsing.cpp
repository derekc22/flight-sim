#include "core/json/actuators/private/parsing.hpp"

#include "core/json/actuators/private/validation.hpp"
#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/actuators/public/manager.hpp"

#include <nlohmann/json.hpp>

namespace json
{

	actuators::ActuatorManager parse_actuator_manager(
		const nlohmann::json& config)
	{
		const auto& surface_actuator_json = config.at("surfaces");
		const auto& propulsor_actuator_json = config.at("propulsors");

		actuators::SurfaceActuators surface_actuators = {
			.elevator = parse_surface_actuator<actuators::ElevatorActuator>(surface_actuator_json, "elevator"),
			.aileron = parse_surface_actuator<actuators::AileronActuator>(surface_actuator_json, "aileron"),
			.rudder = parse_surface_actuator<actuators::RudderActuator>(surface_actuator_json, "rudder"),
			.flap = parse_surface_actuator<actuators::FlapActuator>(surface_actuator_json, "flap"),
			.spoiler = parse_surface_actuator<actuators::SpoilerActuator>(surface_actuator_json, "spoiler")
		};

		actuators::PropulsorActuators propulsor_actuators = {
			.front_propulsor =
				parse_propulsor_actuator<actuators::FrontPropulsorActuator>(propulsor_actuator_json, "front_propulsor"),
			.left_propulsor =
				parse_propulsor_actuator<actuators::LeftPropulsorActuator>(propulsor_actuator_json, "left_propulsor"),
			.right_propulsor =
				parse_propulsor_actuator<actuators::RightPropulsorActuator>(propulsor_actuator_json, "right_propulsor"),
		};

		return {.surface_actuators = surface_actuators, .propulsor_actuators = propulsor_actuators};
	}

} // namespace json
