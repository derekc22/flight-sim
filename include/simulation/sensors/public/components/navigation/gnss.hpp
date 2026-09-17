#pragma once
#include "simulation/sensors/public/components/base.hpp"
#include "simulation/sensors/public/data/types.hpp"

namespace sensors
{

	struct GNSSReceiver : Sensor {
		std::optional<Eigen::Vector3d> prev_pI_BI_lag;
		std::optional<Eigen::Vector3d> prev_vB_BI_lag;

		/**
		 * @brief Measures inertial position through the configured GNSS sensor model.
		 *
		 * @param[in] pI_BI Ground-truth inertial position [m].
		 * @param[in] dt Measurement interval [s].
		 * @return Lagged, biased, and noised position measurement [m].
		 */
		PositionMeasurement measure(const dynamics::Position& pI_BI, double dt);

		/**
		 * @brief Measures body-expressed translational velocity through the configured GNSS sensor model.
		 *
		 * @param[in] vB_BI Ground-truth body-expressed translational velocity [m/s].
		 * @param[in] dt Measurement interval [s].
		 * @return Lagged, biased, and noised translational-velocity measurement [m/s].
		 */
		TranslationalVelocityMeasurement measure(const dynamics::TranslationalVelocity& vB_BI, double dt);
	};

} // namespace sensors
