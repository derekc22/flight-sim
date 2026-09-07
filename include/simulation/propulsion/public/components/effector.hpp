#pragma once
#include "simulation/propulsion/public/data/types.hpp"

#include <Eigen/Dense>
#include <optional>

namespace propulsion
{

	struct PropulsorEffector {
		Eigen::Vector3d pB_propB = constants::Zero3;
		Eigen::Vector3d n_prop = constants::ei;
		std::optional<PropellerAssembly> propellers;
		std::optional<double> propeller_omega_t_1;

		template <typename T> PropulsorEffectorOutput_T<T> step(const PropulsorEffectorInput_T<T>& input);
		void commit(const std::optional<double>& propeller_omega_t);
	};

} // namespace propulsion

#include "simulation/propulsion/public/components/effector.tpp"
