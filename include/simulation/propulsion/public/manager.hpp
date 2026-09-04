#pragma once
#include "simulation/propulsion/public/data/types.hpp"

namespace propulsion
{

	struct PropulsionManager {
		PropulsionState propulsion_state_t_1;

		PropulsionManagerOutput step(const PropulsionManagerInput& input);
		void commit(const PropulsionState& propulsion_state_t);

		template <typename T> PropulsionManagerOutput_T<T> step_T(const PropulsionManagerInput_T<T>& input);
	};

} // namespace propulsion

#include "simulation/propulsion/public/manager.tpp"
