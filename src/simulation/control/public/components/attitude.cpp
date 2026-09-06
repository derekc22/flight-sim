#include "simulation/control/public/components/attitude.hpp"

#include "simulation/control/private/components/controllers/attitude/damper.hpp"
#include "simulation/control/private/components/controllers/attitude/pid.hpp"
#include "simulation/control/private/detail/stateful_controller.hpp"
#include "simulation/util/public/math.hpp"

namespace control
{

	AttitudeControl::AttitudeControl(
		const AttitudePIDParameters& params)
		: implementation(
			  make_stateful_controller<AttitudePID, AttitudeControlImplementation, AttitudeControlInput>(params))
	{
	}

	AttitudeControl::AttitudeControl(
		const DamperPIDParameters& params)
		: implementation(
			  make_stateful_controller<DamperPID, AttitudeControlImplementation, AttitudeControlInput>(params))
	{
	}

	ControlComponentOutput AttitudeControl::step(
		const AttitudeControlInput& input,
		double dt)
	{
		ControlComponentOutput output{.mu = implementation(input, dt)};
		util::fill_arr(output.active_mask, 3, 6, true);
		util::fill_arr(output.actuator_mask, 0, 3, true);
		return output;
	}

} // namespace control
