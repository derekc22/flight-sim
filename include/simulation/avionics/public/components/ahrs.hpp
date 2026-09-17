#pragma once
#include "simulation/avionics/public/data/types.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/constants/public/scalars.hpp"

#include <Eigen/Dense>

namespace avionics
{

	struct AttitudeHeadingReferenceSystem {
		/**
		 * @brief Advances the orientation estimate using a Mahony-style attitude filter.
		 *
		 * Propagates the inertial-to-body quaternion with gyroscope data. When the
		 * measured specific-force magnitude is within the configured tolerance of
		 * gravity, an accelerometer-derived proportional-integral correction is added.
		 * Otherwise, the uncorrected gyroscope prediction is returned.
		 *
		 * @param[in] prev_qIB Previous inertial-to-body orientation estimate [-].
		 * @param[in] wB_BI Measured body angular velocity [rad/s].
		 * @param[in] fB Measured body-expressed specific force [m/s^2].
		 * @param[in] dt Estimation interval [s].
		 * @return Normalized and canonicalized inertial-to-body orientation estimate [-].
		 */
		OrientationMeasurement compute(const OrientationMeasurement& prev_qIB,
			const sensors::AngularVelocityMeasurement& wB_BI,
			const sensors::TranslationalAccelerationMeasurement& fB,
			double dt);
		double Kp;
		double Ki;
		double fB_tol_scale;

		Eigen::Vector3d integral = constants::Zero3;
		double fB_tol = fB_tol_scale * constants::g_earth;
	};

} // namespace avionics
