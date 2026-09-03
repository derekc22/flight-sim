#pragma once
#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/integrators/public/data/types.hpp"

namespace integrators {

	template <typename T, typename Model>
	WrenchEvaluation_T<T> compute_wrench_set_T(const Model& model,
	    const dynamics::Twist_T<T>& twist,
	    const atmospheric::StaticAtmosphericState& atm,
	    const actuators::ActuatorInputs_T<T>& u,
	    const atmospheric::Wind& windB,
	    const constants::Vector3_T<T>& gB,
	    T dt,
	    bool steady_state);

}

#include "simulation/integrators/public/detail/wrench.tpp"
