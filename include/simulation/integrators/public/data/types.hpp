#pragma once
#include "simulation/actuators/public/components/collection.hpp"
#include "simulation/aerodynamics/public/manager.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/propulsion/public/manager.hpp"
#include "simulation/structural/public/data/types.hpp"

namespace integrators
{

	struct RK4Model {
		const structural::StructuralState& struc_t;
		aerodynamics::AerodynamicsManager& aerodynamic;
		actuators::PropulsorActuators& propulsor_actuators;
		propulsion::PropulsionManager& propulsion;
	};

	struct RK4Output {
		dynamics::RigidBodyState Xt1;
		dynamics::WrenchSet WB_set;
	};

	template <typename T> struct WrenchEvaluation_T {
		dynamics::WrenchSet_T<T> WB_set;
		propulsion::PropulsionState_T<T> propulsion_state_t;
	};

	struct WrenchEvaluation {
		dynamics::WrenchSet WB_set;
		propulsion::PropulsionState propulsion_state_t;
	};

} // namespace integrators
