#pragma once
#include <Eigen/Dense>
#include "simulation/avionics/public/data/types.hpp"
#include "simulation/constants/public/scalars.hpp"
#include "simulation/constants/public/linalg.hpp"

namespace avionics {

	struct AttitudeHeadingReferenceSystem {
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
