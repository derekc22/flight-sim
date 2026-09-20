#pragma once
#include "simulation/avionics/public/data/types.hpp"

namespace avionics
{

	struct AirDataComputer {
		/**
		 * @brief Computes free-stream speed from Mach number and static air temperature.
		 *
		 * @param[in] Mach Measured Mach number [-].
		 * @param[in] T Measured static air temperature [K].
		 * @return Measured free-stream speed [m/s].
		 */
		FreeStreamVelocityMeasurement compute(const MachNumberMeasurement& Mach,
			const StaticAirTemperatureMeasurement& T);

		/**
		 * @brief Computes pressure altitude from static air pressure.
		 *
		 * @param[in] P Measured static air pressure [Pa].
		 * @return Pressure altitude [m].
		 */
		PressureAltitudeMeasurement compute(const sensors::StaticAirPressureMeasurement& P);

		/**
		 * @brief Computes positive-up vertical speed from the static-pressure rate.
		 *
		 * @param[in] P Current measured static air pressure [Pa].
		 * @param[in] prev_P Previous measured static air pressure [Pa].
		 * @param[in] T Measured static air temperature [K].
		 * @param[in] dt Measurement interval [s].
		 * @return Pressure-derived vertical speed [m/s].
		 */
		VerticalSpeedMeasurement compute(const sensors::StaticAirPressureMeasurement& P,
			const sensors::StaticAirPressureMeasurement& prev_P,
			const StaticAirTemperatureMeasurement& T,
			double dt);

		/**
		 * @brief Computes air density from static pressure and temperature.
		 *
		 * @param[in] P Measured static air pressure [Pa].
		 * @param[in] T Measured static air temperature [K].
		 * @return Measured air density [kg/m^3].
		 */
		AirDensityMeasurement compute(const sensors::StaticAirPressureMeasurement& P,
			const StaticAirTemperatureMeasurement& T);
	};

} // namespace avionics
