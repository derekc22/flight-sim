#include "simulation/control/public/components/velocity.hpp"

#include "simulation/control/private/components/controllers/velocity/pid.hpp"
#include "simulation/control/private/detail/stateful_controller.hpp"
#include "simulation/util/public/math.hpp"

namespace control
{

	VelocityControl::VelocityControl(
		const VelocityPIDParameters& params)
		: implementation(
			  make_stateful_controller<VelocityPID, VelocityControlImplementation, VelocityControlInput>(params))
	{
	}

	ControlComponentOutput VelocityControl::step(
		const VelocityControlInput& input,
		double dt)
	{
		ControlComponentOutput output{.mu = implementation(input, dt)};
		util::fill_arr(output.active_mask, 0, 1, true);
		util::fill_arr(output.actuator_mask, 3, 4, true);
		return output;
	}

} // namespace control
