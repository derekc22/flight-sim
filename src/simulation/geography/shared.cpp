#include <cmath>
#include <format>
#include <stdexcept>
#include <Eigen/Dense>
#include "simulation/constants/constants.hpp"
#include "simulation/frames/frames.hpp"
#include "simulation/geography/geography.hpp"
#include "simulation/geography/shared.hpp"
#include "simulation/transforms/transforms.hpp"

namespace geography {

    GeographicState compute_geographic_state(const frames::Frame& F) {
        if (F.parent != nullptr) {
            throw std::invalid_argument(std::format("geography::geographic_state: Invalid frame input, the parent of {} must be ECEFFrame", F.name));
        }
        const frames::FrameView fv = F.view();
        return geography::lat_lon_alt_from_pE(fv.H->p());
    }

    dynamics::OrientationMatrix CEN_from_lat_lon(const geography::Latitude& latitude, const geography::Longitude& longitude) {
        Eigen::Matrix3d CEN;
        double lat = latitude.data;
        double lon = longitude.data;
        CEN     <<   -std::sin(lat) * std::cos(lon),  -std::sin(lat) * std::sin(lon),   std::cos(lat),
                                     -std::sin(lon),                   std::cos(lon),               0,
                     -std::cos(lat) * std::cos(lon),  -std::cos(lat) * std::sin(lon),  -std::sin(lat);
        return { CEN };
    };

    dynamics::Position pE_from_lat_lon_alt(const geography::GeographicState& geographic_state) {
        double lat = geographic_state.lat.data;
        double lon = geographic_state.lon.data;
        double alt = geographic_state.alt.data;

        double r = constants::r_earth + alt;

        double cos_lat = std::cos(lat);
        double sin_lat = std::sin(lat);
        double cos_lon = std::cos(lon);
        double sin_lon = std::sin(lon);

        double x = r * cos_lat * cos_lon;
        double y = r * cos_lat * sin_lon;
        double z = r * sin_lat;
        Eigen::Vector3d p(x, y, z);

        return { p };
    }

    dynamics::Gravity gN() { 
        return { Eigen::Vector3d(0, 0, constants::g_earth) }; 
    };

    dynamics::Gravity gB(const dynamics::Position& pE, const dynamics::homogeneousFrameTransformationMatrix& HEB) { 
        return { HEB.C().data * geography::gE(HEB.p()).data }; 
    };
    dynamics::Gravity gB(const dynamics::Position& pE, const dynamics::OrientationMatrix& CEB) { 
        return { CEB.data * geography::gE(pE).data }; 
    };
    dynamics::Gravity gB(const dynamics::Position& pE, const dynamics::OrientationQuaternion& qEB) { 
        return { transforms::quat_to_rot(qEB.data) * geography::gE(pE).data }; 
    };
    dynamics::Gravity gB(const dynamics::Position& pE, const dynamics::EulerAngles& eulEB) { 
        return { transforms::eul_to_C(eulEB.psi(), eulEB.theta(), eulEB.phi(), "ZYX", "intr") * geography::gE(pE).data }; 
    };

    dynamics::Gravity gB(const dynamics::homogeneousFrameTransformationMatrix& HNB) { 
        return { HNB.C().data * geography::gN().data }; 
    };
    dynamics::Gravity gB(const dynamics::OrientationMatrix& CNB) {
        return { CNB.data * geography::gN().data };
    };
    dynamics::Gravity gB(const dynamics::OrientationQuaternion& qNB) { 
        return { transforms::quat_to_rot(qNB.data) * geography::gN().data }; 
    };
    dynamics::Gravity gB(const dynamics::EulerAngles& eulNB) { 
        return { transforms::eul_to_C(eulNB.psi(), eulNB.theta(), eulNB.phi(), "ZYX", "intr") * geography::gN().data }; 
    };

    dynamics::Gravity gS(const dynamics::Gravity& gB, const dynamics::OrientationMatrix& CBS) { 
        return { CBS.data * gB.data }; 
    };
    dynamics::Gravity gW(const dynamics::Gravity& gS, const dynamics::OrientationMatrix& CSW) { 
        return { CSW.data * gS.data }; 
    };

}
