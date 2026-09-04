#include "simulation/actuators/public/data/helpers.hpp"

#include "core/json/actuators/public/parser.hpp"
#include "core/json/aerodynamics/public/parser.hpp"
#include "core/json/allocator/public/parser.hpp"
#include "core/json/avionics/public/parser.hpp"
#include "core/json/control/public/parser.hpp"
#include "core/json/estimation/public/parser.hpp"
#include "core/json/guidance/public/parser.hpp"
#include "core/json/initialization/public/parser.hpp"
#include "core/json/sensors/public/parser.hpp"
#include "core/json/settings/public/parser.hpp"
#include "core/json/structural/public/parser.hpp"
#include "simulation/runner/private/data/helpers.hpp"
#include "simulation/settings/public/detail/application.hpp"

namespace runner
{

	vehicles::Aircraft load_vehicle(
		const std::string& aircraft_id,
		const JSONFlags& json_flags)
	{
		structural::StructuralManager structural_manager = json::parse_structural_config();
		actuators::ActuatorManager actuator_manager = json::parse_actuator_config(structural_manager);
		control::ControlManager control_manager = json::parse_control_config(json_flags.trim_flag);

		// create vehicle from config
		vehicles::Aircraft aircraft{
			aircraft_id,
			structural_manager,
			json::parse_aerodynamics_config(),
			actuator_manager,
			control_manager,
			json::parse_sensors_config(),
			json::parse_avionics_config(),
			json::parse_guidance_config(control_manager),
			json::parse_estimation_config(json_flags.trim_flag),
			json::parse_allocator_config()
		};

		// set initial conditions from config
		vehicles::StepOptions InitStepOpts = json::parse_initialization_config(json_flags.trim_flag);
		structural::StructuralState struc_t = aircraft.structural_manager.step({}).struc_t;
		InitStepOpts.CGFrameFRDStepOpts =
			vehicles::CGFrameFRDStepOptions{.pB_GB = dynamics::Position{struc_t.pB_GB.data}};
		aircraft.step(InitStepOpts);

		// parse settings
		settings::Settings simulation_settings = json::parse_settings_config(actuator_manager);

		// apply settings
		aircraft = settings::apply_settings(aircraft, simulation_settings);

		return aircraft;
	}

} // namespace runner
