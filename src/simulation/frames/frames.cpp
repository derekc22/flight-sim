#include <Eigen/Dense>
#include <cmath>
#include <tuple>
#include "simulation/frames/frames.hpp"
#include "simulation/transforms/transforms.hpp"
#include "simulation/global/global.hpp"

namespace frames {

    Eigen::Matrix3d CEN_from_lat_lon(double lat, double lon) {
        Eigen::Matrix3d CEN;
        CEN     <<   -std::sin(lat) * std::cos(lon),  -std::sin(lat) * std::sin(lon),   std::cos(lat),
                                     -std::sin(lon),                   std::cos(lon),               0,
                     -std::cos(lat) * std::cos(lon),  -std::cos(lat) * std::sin(lon),  -std::sin(lat);
        return CEN;
    };

    std::array<double, 3> lat_lon_alt_from_xECEF(const Eigen::Vector3d& xECEF) {
        double x = xECEF(0);
        double y = xECEF(1);
        double z = xECEF(2);

        double p = std::sqrt(x * x + y * y);
        double r = std::sqrt(x * x + y * y + z * z);

        // Longitude (radians)
        double lon = std::atan2(y, x);

        // Geocentric latitude (radians)
        double lat = std::atan2(z, p);

        // Altitude above the spherical Earth (meters)
        double alt = r - global::r_earth;

        return {lat, lon, alt};
    }


    namespace common {
        // Get local ECEF gravity vector expressed WRT the ECEF frame based on your current position WRT the ECEF frame expressed WRT the ECEF frame
        Eigen::Vector3d gECEF(Eigen::Vector3d pE_BE){ return -global::gravity * pE_BE.normalized(); }
    }

}
