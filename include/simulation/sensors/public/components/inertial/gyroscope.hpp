#pragma once
#include "simulation/sensors/public/components/base.hpp"
#include "simulation/sensors/public/data/types.hpp"

namespace sensors {

	struct Gyroscope : Sensor {
		std::optional<Eigen::Vector3d> prev_wB_BI_lag;
		AngularVelocityMeasurement measure(const dynamics::AngularVelocity& wB_BI, double dt);
	};

} // namespace sensors
