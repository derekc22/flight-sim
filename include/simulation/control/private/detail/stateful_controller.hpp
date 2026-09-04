#pragma once
#include "simulation/control/public/data/types.hpp"

namespace control
{

	template <typename Controller, typename Implementation, typename Input, typename Parameters>
	Implementation make_stateful_controller(const Parameters& params);

} // namespace control

#include "simulation/control/private/detail/stateful_controller.tpp"
