#pragma once
#include <Eigen/Dense>

namespace atmospheric
{

	struct Wind {
		Eigen::Vector3d data;
	};

	struct StaticAirTemperature {
		double data;
	};

	struct StagnationAirTemperature {
		double data;
	};

	struct AirDensity {
		double data;
	};

	struct StaticAirPressure {
		double data;
	};

	struct StagnationAirPressure {
		double data;
	};

	struct DynamicViscosity {
		double data;
	};

	struct MachNumber {

		double data;
	};

	struct StaticAtmosphericState {
		StaticAirTemperature T;
		StaticAirPressure P;
		AirDensity rho;
		DynamicViscosity mu;
	};

	struct StagnationAtmosphericState {
		StagnationAirTemperature T0;
		StagnationAirPressure P0;
		AirDensity rho;
		DynamicViscosity mu;
	};

} // namespace atmospheric
