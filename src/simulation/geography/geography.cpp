#include <Eigen/Dense>
#include "simulation/frames/frames.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/geography/geography.hpp"
#include "simulation/transforms/transforms.hpp"

namespace geography {

    dynamics::OrientationMatrix CEN_from_lat_lon(const geography::Latitude& latitude, const geography::Longitude& longitude) {
        Eigen::Matrix3d CEN;
        double lat = latitude.data;
        double lon = longitude.data;
        CEN     <<   -std::sin(lat) * std::cos(lon),  -std::sin(lat) * std::sin(lon),   std::cos(lat),
                                     -std::sin(lon),                   std::cos(lon),               0,
                     -std::cos(lat) * std::cos(lon),  -std::cos(lat) * std::sin(lon),  -std::sin(lat);
        return dynamics::OrientationMatrix{ CEN };
    };

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
        Altitude alt{ r - global::r_earth };

        return geography::GeographicState{ lat, lon, alt };
    }

    dynamics::Position pE_from_lat_lon_alt(const geography::GeographicState& geographicState) {
        double lat = geographicState.lat.data;   // [rad]
        double lon = geographicState.lon.data;   // [rad]
        double alt = geographicState.alt.data;   // [m]

        double r = global::r_earth + alt;

        double cos_lat = std::cos(lat);
        double sin_lat = std::sin(lat);
        double cos_lon = std::cos(lon);
        double sin_lon = std::sin(lon);

        double x = r * cos_lat * cos_lon;
        double y = r * cos_lat * sin_lon;
        double z = r * sin_lat;
        Eigen::Vector3d p(x, y, z);

        return dynamics::Position{ p };
    }

    dynamics::Gravity gE(const dynamics::Position& pE){ return dynamics::Gravity{ -global::gravity * pE.data.normalized() }; }
    dynamics::Gravity gN() { return dynamics::Gravity{ Eigen::Vector3d(0, 0, global::gravity) }; };

    dynamics::Gravity gB(const dynamics::Position& pE, const dynamics::HomogenousFrameTransformationMatrix& HEB) { return dynamics::Gravity{ HEB.C().data * geography::gE(HEB.p()).data }; };
    dynamics::Gravity gB(const dynamics::Position& pE, const dynamics::OrientationMatrix& CEB) { return dynamics::Gravity{ CEB.data * geography::gE(pE).data }; };
    dynamics::Gravity gB(const dynamics::Position& pE, const dynamics::OrientationQuaternion& qEB) { return dynamics::Gravity{ transforms::quat2rot(qEB.data) * geography::gE(pE).data }; };
    dynamics::Gravity gB(const dynamics::Position& pE, const dynamics::EulerAngles& eulEB) { return dynamics::Gravity{ transforms::eul2C(eulEB.psi(), eulEB.theta(), eulEB.phi(), "ZYX", "intr") * geography::gE(pE).data }; };

    dynamics::Gravity gB(const dynamics::HomogenousFrameTransformationMatrix& HNB) { return dynamics::Gravity{ HNB.C().data * geography::gN().data }; };
    dynamics::Gravity gB(const dynamics::OrientationMatrix& CNB) { return dynamics::Gravity{ CNB.data * geography::gN().data }; };
    dynamics::Gravity gB(const dynamics::OrientationQuaternion& qNB) { return dynamics::Gravity{ transforms::quat2rot(qNB.data) * geography::gN().data }; };
    dynamics::Gravity gB(const dynamics::EulerAngles& eulNB) { return dynamics::Gravity{ transforms::eul2C(eulNB.psi(), eulNB.theta(), eulNB.phi(), "ZYX", "intr") * geography::gN().data }; };

    dynamics::Gravity gS(const dynamics::Gravity& gB, const dynamics::OrientationMatrix& CBS) { return dynamics::Gravity{ CBS.data * gB.data }; };
    dynamics::Gravity gW(const dynamics::Gravity& gS, const dynamics::OrientationMatrix& CSW) { return dynamics::Gravity{ CSW.data * gS.data }; };




}

