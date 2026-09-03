#include "simulation/sensors/public/components/air_data/tat_probe.hpp"

namespace sensors {

	StagnationAirTemperatureMeasurement TotalAirTemperatureProbe::measure(
	    const atmospheric::StagnationAirTemperature& T0,
	    double dt)
	{
		return {step(T0.data, prev_T0_lag, dt)};
	}

} // namespace sensors
