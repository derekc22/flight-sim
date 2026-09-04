#pragma once
#include "simulation/runner/public/data/types.hpp"
#include "simulation/sensors/public/data/types.hpp"

namespace runner
{

	struct MeasurementsWrapper {
		sensors::SensorMeasurements sensor_meas_t_1;
		dynamics::RigidBodyState Yt_1;

		MeasurementsWrapperOutput step(const MeasurementsWrapperInput& input);
	};

} // namespace runner
