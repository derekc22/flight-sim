#include <cmath>
#include "simulation/avionics/public/components/adc.hpp"
#include "simulation/constants/public/scalars.hpp"
#include "simulation/util/public/math.hpp"

namespace avionics {

	FreeStreamVelocityMeasurement AirDataComputer::compute(
	    const MachNumberMeasurement& Mach,
	    const StaticAirTemperatureMeasurement& T)
	{
		double Vinf = Mach.data * util::sqrt(constants::gamma_air * constants::R_air * T.data);
		return {Vinf};
	}

	PressureAltitudeMeasurement AirDataComputer::compute(
	    const sensors::StaticAirPressureMeasurement& P)
	{
		double pressure_alt = (constants::T_SL / std::abs(constants::lapse_rate)) *
		    (1.0 -
		        std::pow(P.data / constants::P_SL,
		            (constants::R_air * std::abs(constants::lapse_rate)) / constants::g_earth));
		return {pressure_alt};
	}

	VerticalSpeedMeasurement AirDataComputer::compute(
	    const sensors::StaticAirPressureMeasurement& P,
	    const sensors::StaticAirPressureMeasurement& prev_P,
	    const StaticAirTemperatureMeasurement& T,
	    double dt)
	{
		double P_dot_meas = (P.data - prev_P.data) / dt;
		double alt_BE_dot = -(constants::R_air * T.data / (constants::g_earth * P.data)) * P_dot_meas;
		return {alt_BE_dot};
	}

	AirDensityMeasurement AirDataComputer::compute(
	    const sensors::StaticAirPressureMeasurement& P,
	    const StaticAirTemperatureMeasurement& T)
	{
		double rho = P.data / (constants::R_air * T.data);
		return {rho};
	}

} // namespace avionics
