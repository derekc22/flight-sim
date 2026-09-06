#include "simulation/control/public/components/linear_quadratic.hpp"

#include "simulation/control/private/components/controllers/linear_quadratic/lqi.hpp"
#include "simulation/control/private/components/controllers/linear_quadratic/lqr.hpp"
#include "simulation/control/private/detail/stateful_controller.hpp"
#include "simulation/util/public/math.hpp"

namespace control
{

	LinearQuadraticControl::LinearQuadraticControl(
		const LinearQuadraticRegulatorParameters& params)
		: implementation(make_stateful_controller<LinearQuadraticRegulator,
			  LinearQuadraticControlImplementation,
			  LinearQuadraticControlInput>(params))
	{
	}

	LinearQuadraticControl::LinearQuadraticControl(
		const LinearQuadraticIntegratorParameters& params)
		: implementation(make_stateful_controller<LinearQuadraticIntegrator,
			  LinearQuadraticControlImplementation,
			  LinearQuadraticControlInput>(params))
	{
	}

	ControlComponentOutput LinearQuadraticControl::step(
		const LinearQuadraticControlInput& input,
		double dt)
	{
		ControlComponentOutput output{.mu = implementation(input, dt)};
		util::fill_arr(output.active_mask, 0, 6, true);
		util::fill_arr(output.actuator_mask, 0, 6, true);
		return output;
	}

} // namespace control
