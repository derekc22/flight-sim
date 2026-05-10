#include <Eigen/Dense>
#include "simulation/constants/constants.hpp"
#include "simulation/geography/geography.hpp"

namespace geography {

    geography::GeographicState lat_lon_alt_from_pE(const dynamics::Position & pE) {
        double x = pE.data(0);
        double y = pE.data(1);
        double z = pE.data(2);

        double p = std::sqrt(x * x + y * y);
        double r = std::sqrt(x * x + y * y + z * z);

        // Longitude (radians)
        Longitude lon{ std::atan2(y, x) };

        // Geocentric latitude (radians)
        Latitude lat{ std::atan2(z, p) };

        // Altitude above the spherical Earth (meters)
        Altitude alt{ r - constants::r_earth };

        return { lat, lon, alt };
    }

    dynamics::Gravity gE(const dynamics::Position& pE){ 
        return { -constants::g_earth * pE.data.normalized() }; 
    }

}
