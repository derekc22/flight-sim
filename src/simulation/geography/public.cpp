#include <cmath>
#include <format>
#include <stdexcept>
#include <Eigen/Dense>
#include "simulation/constants/public.hpp"
#include "simulation/frames/public.hpp"
#include "simulation/geography/private.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/transforms/public.hpp"
#include "simulation/util/public.hpp"

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
        CEN     <<   -util::sin(lat) * util::cos(lon),  -util::sin(lat) * util::sin(lon),   util::cos(lat),
                                      -util::sin(lon),                    util::cos(lon),                0,
                     -util::cos(lat) * util::cos(lon),  -util::cos(lat) * util::sin(lon),  -util::sin(lat);
        return { CEN };
    };

    dynamics::Position pE_from_lat_lon_alt(const geography::GeographicState& geographic_state) {
        double lat = geographic_state.lat.data;
        double lon = geographic_state.lon.data;
        double alt = geographic_state.alt.data;

        double r = constants::r_earth + alt;

        double x = r * util::cos(lat) * util::cos(lon);
        double y = r * util::cos(lat) * util::sin(lon);
        double z = r * util::sin(lat);
        Eigen::Vector3d p(x, y, z);

        return { p };
    }

    dynamics::Gravity gN() { 
        return { Eigen::Vector3d(0, 0, constants::g_earth) }; 
    };

    dynamics::Gravity gB(const dynamics::Position& pE, const dynamics::HomogeneousFrameTransformationMatrix& HEB) { 
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

    dynamics::Gravity gB(const dynamics::HomogeneousFrameTransformationMatrix& HNB) { 
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
