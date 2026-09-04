#pragma once
#include "simulation/actuators/public/data/types.hpp"

#include <Eigen/Dense>
#include <optional>

namespace actuators
{

	struct Actuator {
		double limit_max;
		double limit_min;
		double tau;
		std::optional<double> lag_state;

		double step(double cmd, double dt);

		Actuator(double limit_max, double limit_min, double tau);
		Actuator();
	};

	struct SurfaceActuator : Actuator {
		using Actuator::Actuator;
	};

	struct PropulsorActuator : Actuator {
		double inclination_angle;
		double toe_angle;
		Eigen::Vector3d pB_propB;
		Eigen::Vector3d n_prop;
		std::optional<PropellerAssembly> propellers;

		PropulsorActuator(double limit_max,
			double limit_min,
			double tau,
			double inclination_angle,
			double toe_angle,
			const Eigen::Vector3d& pB_propB,
			std::optional<PropellerAssembly> propellers = std::nullopt);
		PropulsorActuator();
	};

} // namespace actuators
