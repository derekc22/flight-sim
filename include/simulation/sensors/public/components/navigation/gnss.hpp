#pragma once
#include "simulation/sensors/public/components/base.hpp"
#include "simulation/sensors/public/data/types.hpp"

namespace sensors {

	struct GNSSReceiver : Sensor {
		std::optional<Eigen::Vector3d> prev_pI_BI_lag;
		std::optional<Eigen::Vector3d> prev_vB_BI_lag;
		PositionMeasurement measure(const dynamics::Position& pI_BI, double dt);
		TranslationalVelocityMeasurement measure(const dynamics::TranslationalVelocity& vB_BI, double dt);
	};

} // namespace sensors
