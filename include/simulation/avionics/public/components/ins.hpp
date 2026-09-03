#pragma once
#include "simulation/avionics/public/data/types.hpp"

namespace avionics {

	struct InertialNavigationSystem {
		sensors::PositionMeasurement compute(const sensors::PositionMeasurement& prev_pI_BI,
		    const sensors::TranslationalVelocityMeasurement& prev_vB_BI,
		    const sensors::TranslationalAccelerationMeasurement& fB,
		    const dynamics::Gravity& gB,
		    const OrientationMeasurement& prev_qIB,
		    double dt);
		sensors::TranslationalVelocityMeasurement compute(const sensors::TranslationalVelocityMeasurement& prev_vB_BI,
		    const sensors::TranslationalAccelerationMeasurement& fB,
		    const dynamics::Gravity& gB,
		    const sensors::AngularVelocityMeasurement& wB_BI,
		    double dt);
	};

} // namespace avionics
