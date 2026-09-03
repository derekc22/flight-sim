#include <cmath>
#include <stdexcept>
#include <Eigen/Dense>
#include "simulation/constants/public/scalars.hpp"
#include "simulation/frames/public/detail/kinematics.hpp"
#include "simulation/geography/private/detail/geodesy.hpp"
#include "simulation/geography/public/detail/geodesy.hpp"
#include "simulation/util/public/trig.hpp"

namespace geography {

    GeographicState compute_geographic_state(const frames::Frame& F, const frames::Frame& E) {
        if (E.id != frames::FrameID::ECEFFrame) {
            throw std::invalid_argument("geography::compute_geographic_state: E must be ECEFFrame");
        }
        dynamics::HomogeneousTransformationMatrix HEF = frames::H_from_R(F, E);
        return lat_lon_alt_from_pE(HEF.p());
    }

    dynamics::OrientationMatrix CEN_from_lat_lon(const Latitude& latitude, const Longitude& longitude) {
        Eigen::Matrix3d CEN;
        double lat = latitude.data;
        double lon = longitude.data;
        CEN     <<   -util::sin(lat) * util::cos(lon),  -util::sin(lat) * util::sin(lon),   util::cos(lat),
                                      -util::sin(lon),                    util::cos(lon),                0,
                     -util::cos(lat) * util::cos(lon),  -util::cos(lat) * util::sin(lon),  -util::sin(lat);
        return { CEN };
    };

    dynamics::Position pE_from_lat_lon_alt(const GeographicState& geo) {
        double lat = geo.lat.data;
        double lon = geo.lon.data;
        double alt = geo.alt.data;

        double r = constants::r_earth + alt;

        double x = r * util::cos(lat) * util::cos(lon);
        double y = r * util::cos(lat) * util::sin(lon);
        double z = r * util::sin(lat);
        Eigen::Vector3d p(x, y, z);

        return { p };
    }

}
