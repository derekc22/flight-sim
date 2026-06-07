#include <cmath>
#include <Eigen/Dense>
#include "simulation/constants/public.hpp"
#include "simulation/geography/private.hpp"
#include "simulation/util/public.hpp"

namespace geography {

    geography::GeographicState lat_lon_alt_from_pE(const dynamics::Position & pE) {
        double x = pE.data(0);
        double y = pE.data(1);
        double z = pE.data(2);

        double p = util::sqrt(x * x + y * y);
        double r = util::sqrt(x * x + y * y + z * z);

        // Longitude (radians)
        Longitude lon{ util::atan2(y, x) };

        // Geocentric latitude (radians)
        Latitude lat{ util::atan2(z, p) };

        // Altitude above the spherical Earth (meters)
        Altitude alt{ r - constants::r_earth };

        return { lat, lon, alt };
    }

    dynamics::Gravity gE(const dynamics::Position& pE) { 
        return { -constants::g_earth * pE.data.normalized() }; 
    }

}
