#include <iomanip>
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

    void print_vec(const char* name, const Eigen::Vector3d& x, const char* unit) {
        std::cout
            << std::left << std::setw(8) << name
            << "[ "
            << std::right << std::setw(10) << x.x() << ", "
            << std::right << std::setw(10) << x.y() << ", "
            << std::right << std::setw(10) << x.z()
            << " ] " << unit << '\n';
    }

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

        std::cout << std::fixed << std::setprecision(3);

        std::cout
            << "\n"
            << "t       " << t * constants::dt << " [s]\n"
            << "---------------------------------------------------------------------------------\n";

        print_vec("p", p, "[m]");

        std::cout
            << std::left << std::setw(8) << "eul"
            << "[ "
            << std::right << std::setw(10) << util::rad_to_deg(eul.psi()) << ", "
            << std::right << std::setw(10) << util::rad_to_deg(eul.theta()) << ", "
            << std::right << std::setw(10) << util::rad_to_deg(eul.phi())
            << " ] [deg]\n";

        print_vec("v", v, "[m/s]");

        std::cout
            << std::left << std::setw(8) << "w"
            << "[ "
            << std::right << std::setw(10) << util::rad_to_deg(w.x()) << ", "
            << std::right << std::setw(10) << util::rad_to_deg(w.y()) << ", "
            << std::right << std::setw(10) << util::rad_to_deg(w.z())
            << " ] [deg/s]\n";

        print_vec("g", g, "[m/s^2]");
        print_vec("wind", wind, "[m/s]");

        std::cout
            << std::left << std::setw(8) << "geo"
            << "lat: " << std::right << std::setw(10) << util::rad_to_deg(geo.lat.data) << " [deg], "
            << "lon: " << std::right << std::setw(10) << util::rad_to_deg(geo.lon.data) << " [deg], "
            << "alt: " << std::right << std::setw(10) << geo.alt.data << " [m]\n";

        std::cout
            << std::left << std::setw(8) << "aero"
            << "alpha: " << std::right << std::setw(10) << util::rad_to_deg(aero.alpha.data) << " [deg], "
            << "beta: "  << std::right << std::setw(10) << util::rad_to_deg(aero.beta.data)  << " [deg]\n";

        std::cout
            << "---------------------------------------------------------------------------------\n";
    }

}