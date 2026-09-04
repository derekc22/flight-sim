#pragma once
#include "simulation/control/private/detail/stateful_controller.hpp"

namespace control
{

	template <typename Controller, typename Implementation, typename Input, typename Parameters>
	Implementation make_stateful_controller(
		const Parameters& params)
	{
		// Creates a stateful lambda that owns a controller initialized with params
		// Each call forwards input and dt to the stored controller's step() method and returns the result
		// mutable is required because captured values are const by default, but step() may modify the stored controller
		return [controller = Controller{params}](const Input& input, double dt) mutable {
			return controller.step(input, dt);
		};
	}

} // namespace control
