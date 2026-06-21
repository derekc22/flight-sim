#include <iostream>
#include <Eigen/Dense>
#include "simulation/runner/private.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/util/trig/public.hpp"

namespace runner {

    void print_state(
        int t,
        const dynamics::RigidBodyState& Xt,
        const geography::GeographicState& geo,
        const aerodynamics::AerodynamicState& aero,
        const atmospheric::Wind& windB
    ) {
        const Eigen::Vector3d& p = Xt.p.data;
        dynamics::EulerAngles eul;
        eul.set(Xt.q);
        const Eigen::Vector3d& v = Xt.v.data;
        const Eigen::Vector3d& w = Xt.w.data;
        const Eigen::Vector3d& g = geography::gB(Xt.q).data;
        const Eigen::Vector3d& wind = windB.data;

        std::cout
            << "t: " << t * constants::dt << " [s]" << "\n\n"
            << "p: " << p.x() << ", " << p.y() << ", " << p.z() << " [m]" << "\n\n"
            << "eul: " << util::rad_to_deg(eul.psi()) << ", " << util::rad_to_deg(eul.theta()) << ", " << util::rad_to_deg(eul.phi()) << " [deg]" << "\n\n"
            << "v: " << v.x() << ", " << v.y() << ", " << v.z() << " [ms^-1]" << "\n\n"
            << "w: " << util::rad_to_deg(w.x()) << ", " << util::rad_to_deg(w.y()) << ", " << util::rad_to_deg(w.z()) << " [deg/s]" << "\n\n"
            << "g: " << g.x() << ", " << g.y() << ", " << g.z() << " [ms^-2]" << "\n\n"
            << "lat: " << util::rad_to_deg(geo.lat.data) << ", lon: " << util::rad_to_deg(geo.lon.data) << " [deg]" << ", alt: " << geo.alt.data << " [m]" << "\n\n"
            << "alpha: " <<  util::rad_to_deg(aero.alpha.data) << ", beta: " <<  util::rad_to_deg(aero.beta.data) << " [deg]" << "\n\n"
            << "wind: " << wind.x() << ", " << wind.y() << ", " << wind.z() << " [m/s]" << "\n\n"
            << "-------------------------------------------------------------------------------" << "\n\n";
    }

}