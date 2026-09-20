#pragma once
#include "simulation/sensors/public/components/base.hpp"
#include "simulation/sensors/public/data/types.hpp"

namespace sensors
{

	struct Gyroscope : Sensor {
		std::optional<Eigen::Vector3d> prev_wB_BI_lag;

		/**
		 * @brief Measures body angular velocity through the configured sensor model.
		 *
		 * @param[in] wB_BI Ground-truth body angular velocity [rad/s].
		 * @param[in] dt Measurement interval [s].
		 * @return Lagged, biased, and noised body-angular-velocity measurement [rad/s].
		 */
		AngularVelocityMeasurement measure(const dynamics::AngularVelocity& wB_BI, double dt);
	};

} // namespace sensors
