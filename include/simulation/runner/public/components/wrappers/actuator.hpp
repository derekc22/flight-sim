#pragma once
#include "simulation/actuators/public/components/collection.hpp"
#include "simulation/runner/public/data/types.hpp"

namespace runner
{

	struct ActuatorWrapper {
		actuators::ActuatorInputs_T<double> u_cmd_t_1;
		actuators::ActuatorInputs_T<double> u_actual_t_1;

		ActuatorWrapper(const actuators::SurfaceActuators& surface_actuators,
			const actuators::PropulsorActuators& propulsor_actuators);

		/**
		 * @brief Advances actuator dynamics for one simulation step.
		 *
		 * @param[in] input Aircraft and commanded actuator inputs for the step.
		 * @return Applied commands and actual actuator inputs.
		 */
		ActuatorWrapperOutput step(const ActuatorWrapperInput& input);
	};

} // namespace runner
