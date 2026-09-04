#pragma once

namespace constants
{

	// Constants
	inline constexpr double r_earth = 6.371e6; // [m]
	inline constexpr double g_earth = 9.80665; // [ms^-2]
	inline constexpr double pi = 3.14159265358979;
	inline constexpr double eps = 1e-9;
	inline constexpr double eps_strict = 1e-12;
	inline constexpr double hz = 500;			   // [hz]
	inline constexpr double dt = 1.0 / hz;		   // [s]
	inline constexpr double gamma_air = 1.4;	   // [-]
	inline constexpr double R_air = 287.052874247; // [J·kg^-1·K^-1]
	inline constexpr double lapse_rate = -0.0065;  // [K/m]
	inline constexpr double P_SL = 101325;		   // [Pa]
	inline constexpr double rho_SL = 1.225;		   // [kg/m^3]
	inline constexpr double T_SL = 288.0;		   // [K]

	// Runtime safety thresholds
	inline constexpr double collision_threshold = 5.0;		// [m]
	inline constexpr double fuel_starvation_fraction = 0.1; // [-]

} // namespace constants
