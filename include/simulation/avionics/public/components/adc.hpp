#pragma once
#include "simulation/avionics/public/data/types.hpp"

namespace avionics
{

	struct AirDataComputer {
		FreeStreamVelocityMeasurement compute(const MachNumberMeasurement& Mach,
			const StaticAirTemperatureMeasurement& T);
		PressureAltitudeMeasurement compute(const sensors::StaticAirPressureMeasurement& P);
		VerticalSpeedMeasurement compute(const sensors::StaticAirPressureMeasurement& P,
			const sensors::StaticAirPressureMeasurement& prev_P,
			const StaticAirTemperatureMeasurement& T,
			double dt);
		AirDensityMeasurement compute(const sensors::StaticAirPressureMeasurement& P,
			const StaticAirTemperatureMeasurement& T);
	};

} // namespace avionics
