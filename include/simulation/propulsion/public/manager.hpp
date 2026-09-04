#pragma once
#include "simulation/propulsion/public/data/types.hpp"

namespace propulsion
{

	struct PropulsionManager {
		PropulsionState propulsion_state_t_1;

		template <typename T> PropulsionManagerOutput_T<T> step(const PropulsionManagerInput_T<T>& input);
		void commit(const PropulsionState& propulsion_state_t);
	};

} // namespace propulsion

#include "simulation/propulsion/public/manager.tpp"
