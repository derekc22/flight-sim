#include "simulation/sensors/public/components/inertial/accelerometer.hpp"

namespace sensors {

	TranslationalAccelerationMeasurement Accelerometer::measure(
	    const dynamics::TranslationalAcceleration& fB,
	    double dt)
	{
		return {step(fB.data, prev_fB_lag, dt)};
	}

} // namespace sensors
