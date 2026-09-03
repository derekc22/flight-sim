#include "simulation/sensors/public/components/navigation/magnetometer.hpp"

namespace sensors {

	HeadingMeasurement Magnetometer::measure(
	    const geography::Heading& heading,
	    double dt)
	{
		return {step(heading.data, prev_heading_lag, dt)};
	}

} // namespace sensors
