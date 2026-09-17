#pragma once
#include "simulation/runner/public/data/types.hpp"
#include "simulation/sensors/public/data/types.hpp"

namespace runner
{

	struct MeasurementsWrapper {
		sensors::SensorMeasurements sensor_meas_t_1;
		dynamics::RigidBodyState Yt_1;

		/**
		 * @brief Produces the measured rigid-body state for one simulation step.
		 *
		 * Ground truth is returned when avionics are disabled. Otherwise, sensors and
		 * avionics run at their scheduled rates and their previous outputs are held
		 * between updates.
		 *
		 * @param[in,out] input Aircraft sensors, scheduler, and simulation data.
		 * @return Measured rigid-body state.
		 */
		MeasurementsWrapperOutput step(const MeasurementsWrapperInput& input);
	};

} // namespace runner
