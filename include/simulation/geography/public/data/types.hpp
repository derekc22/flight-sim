#pragma once

namespace geography
{

	struct Latitude {
		double data;
	};

	struct Longitude {
		double data;
	};

	struct GeometricAltitude {
		// current: radial height from Earth center minus spherical Earth radius
		// planned: geodetic/ellipsoidal height above WGS84
		double data;
	};

	struct LocalHeight {
		// positive-up height relative to local NED origin, i.e. -pN_BN.z()
		double data;
	};

	struct PressureAltitude {
		// altitude inferred from static pressure
		double data;
	};

	struct GroundElevation {
		// terrain/ground height relative to the Earth surface
		double data;
	};

	struct HeightAGL {
		// height above the terrain/ground
		double data;
	};

	struct Heading {
		double data;
	};

	struct GeographicState {
		Latitude lat;
		Longitude lon;
		GeometricAltitude alt;
	};

} // namespace geography
