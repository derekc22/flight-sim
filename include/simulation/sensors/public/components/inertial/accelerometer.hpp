#pragma once
#include "simulation/sensors/public/components/base.hpp"
#include "simulation/sensors/public/data/types.hpp"

namespace sensors
{

	struct Accelerometer : Sensor {
		std::optional<Eigen::Vector3d> prev_fB_lag;

		/**
		 * @brief Measures body-expressed specific force through the configured sensor model.
		 *
		 * @param[in] fB Ground-truth body-expressed specific force [m/s^2].
		 * @param[in] dt Measurement interval [s].
		 * @return Lagged, biased, and noised specific-force measurement [m/s^2].
		 */
		TranslationalAccelerationMeasurement measure(const dynamics::TranslationalAcceleration& fB, double dt);
		// Note: fB = aB_BI - gB
		//          = FB_net/m - gB
		// Where
		// fB is specific force
		// aB_BI is inertial acceleration (the inerital derivative of velocity - as opposed to the body derivative of velocity, vB_BI_dot)
		// That is, an accelerometer measures specific force - which is inertial acceleration minus the contribution from gravity and expressed in the body frame
	};

} // namespace sensors
