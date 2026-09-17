#pragma once
#include "simulation/avionics/public/data/types.hpp"

namespace avionics
{

	struct InertialNavigationSystem {
		/**
		 * @brief Advances inertial position using body velocity and specific force.
		 *
		 * Rotates the previous body-expressed velocity and the gravity-restored
		 * acceleration into the inertial frame, then applies constant-acceleration
		 * translational kinematics.
		 *
		 * @param[in] prev_pI_BI Previous inertial-expressed position [m].
		 * @param[in] prev_vB_BI Previous body-expressed translational velocity [m/s].
		 * @param[in] fB Measured body-expressed specific force [m/s^2].
		 * @param[in] gB Body-expressed gravitational acceleration [m/s^2].
		 * @param[in] prev_qIB Previous inertial-to-body orientation estimate [-].
		 * @param[in] dt Integration interval [s].
		 * @return Updated inertial-expressed position [m].
		 */
		sensors::PositionMeasurement compute(const sensors::PositionMeasurement& prev_pI_BI,
			const sensors::TranslationalVelocityMeasurement& prev_vB_BI,
			const sensors::TranslationalAccelerationMeasurement& fB,
			const dynamics::Gravity& gB,
			const OrientationMeasurement& prev_qIB,
			double dt);

		/**
		 * @brief Advances body-expressed translational velocity using inertial measurements.
		 *
		 * Restores gravity to the measured specific force and includes the rotating-body
		 * transport term before applying a forward-Euler update.
		 *
		 * @param[in] prev_vB_BI Previous body-expressed translational velocity [m/s].
		 * @param[in] fB Measured body-expressed specific force [m/s^2].
		 * @param[in] gB Body-expressed gravitational acceleration [m/s^2].
		 * @param[in] wB_BI Measured body angular velocity [rad/s].
		 * @param[in] dt Integration interval [s].
		 * @return Updated body-expressed translational velocity [m/s].
		 */
		sensors::TranslationalVelocityMeasurement compute(const sensors::TranslationalVelocityMeasurement& prev_vB_BI,
			const sensors::TranslationalAccelerationMeasurement& fB,
			const dynamics::Gravity& gB,
			const sensors::AngularVelocityMeasurement& wB_BI,
			double dt);
	};

} // namespace avionics
