#pragma once
#include "simulation/propulsion/public/components/collection.hpp"
#include "simulation/propulsion/public/data/types.hpp"

namespace propulsion
{

	struct PropulsionManager {
		PropulsorEffectors propulsor_effectors;

		template <typename T> PropulsionManagerOutput_T<T> step(const PropulsionManagerInput_T<T>& input);
		void commit(const PropulsionState& propulsion_state_t);
	};

} // namespace propulsion

#include "simulation/propulsion/public/manager.tpp"
