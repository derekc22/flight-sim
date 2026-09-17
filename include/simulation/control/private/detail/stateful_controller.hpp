#pragma once
#include "simulation/control/public/data/types.hpp"

namespace control
{

	/**
	 * @brief Creates a stateful controller callable that owns its controller instance.
	 *
	 * @tparam Controller Controller type constructed from @p params.
	 * @tparam Implementation Callable wrapper type returned by the function.
	 * @tparam Input Controller input type.
	 * @tparam Parameters Controller parameter type.
	 * @param[in] params Parameters used to construct the owned controller.
	 * @return Callable that forwards an input and time step to the owned controller.
	 */
	template <typename Controller, typename Implementation, typename Input, typename Parameters>
	Implementation make_stateful_controller(const Parameters& params);

} // namespace control

#include "simulation/control/private/detail/stateful_controller.tpp"
