#include "simulation/propulsion/public/manager.hpp"

namespace propulsion
{

	void PropulsionManager::commit(
		const PropulsionState& propulsion_state_t)
	{
		propulsor_effectors.front_propulsor.commit(propulsion_state_t.front_propulsor_omega);
		propulsor_effectors.left_propulsor.commit(propulsion_state_t.left_propulsor_omega);
		propulsor_effectors.right_propulsor.commit(propulsion_state_t.right_propulsor_omega);
	}

} // namespace propulsion
