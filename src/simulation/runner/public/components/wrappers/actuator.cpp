#include "simulation/runner/public/components/wrappers/actuator.hpp"

#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/actuators/public/manager.hpp"
#include "simulation/constants/public/scalars.hpp"
#include "simulation/vehicles/public/aircraft.hpp"

namespace runner
{

	ActuatorWrapper::ActuatorWrapper(
		const actuators::SurfaceActuators& surface_actuators,
		const actuators::PropulsorActuators& propulsor_actuators)
	{
		// set u_actual_t_1 to match actuators' neutral initialization
		u_actual_t_1 = actuators::get_neutral_actuator_inputs(surface_actuators, propulsor_actuators);
	}

	ActuatorWrapperOutput ActuatorWrapper::step(
		const ActuatorWrapperInput& input)
	{
		actuators::ActuatorManager& actuator_manager = input.aircraft.actuator_manager;

		// apply fixed actuator inputs
		// apply surface actuator dynamics
		// apply propulsor actuator dynamics
		actuators::ActuatorManagerOutput output = actuator_manager.step({.u_cmd = input.u_cmd, .dt = constants::dt});

		// update prior-step control command
		u_cmd_t_1 = output.u_cmd;

		// update prior-step actual control
		u_actual_t_1 = output.u_actual;

		return {.u_cmd = output.u_cmd, .u_actual = output.u_actual};
	}

} // namespace runner
