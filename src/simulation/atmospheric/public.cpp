#include <cmath>
#include <stdexcept>
#include "simulation/atmospheric/private.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/frames/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/util/public.hpp"
#include "simulation/util/trig/public.hpp"
#include "simulation/util/units/public.hpp"

namespace atmospheric {

    StaticAtmosphericState compute_static_atmospheric_state(const frames::Frame& F, const frames::Frame& E) {
        if (E.id != frames::FrameID::ECEFFrame) {
            throw std::invalid_argument("atmospheric::compute_static_atmospheric_state: E must be ECEFFrame");
        }
        return std_atmosphere(geography::compute_geographic_state(F, E).alt);
    }

    StaticAirTemperature T_from_T0(const StagnationAirTemperature& T0, const MachNumber& M) {
        double T = T0.data / ( 1 + ((constants::gamma_air - 1) / 2) * M.data * M.data );
        return { T };
    };

    MachNumber mps_to_mach(const dynamics::TranslationalVelocity& v, const StaticAirTemperature& T) {
        double a = util::sqrt(constants::gamma_air * constants::R_air * T.data);
        double M = v.data.norm() / a;
        return { M };
    }

    StagnationAtmosphericState static_to_stagnation(const StaticAtmosphericState& atm, const MachNumber& M) {
        StagnationAirTemperature T0 = T0_from_T(atm.T, M);
        StagnationAirPressure P0 = P0_from_P(atm.P, M);

        return { T0, P0, atm.rho, atm.mu };
    };

    MachNumber compute_mach(const StagnationAirPressure& P0, const StaticAirPressure& P) {
        double M = util::sqrt( (2.0 / (constants::gamma_air - 1.0)) * (std::pow( P0.data / P.data, (constants::gamma_air - 1.0) / constants::gamma_air ) - 1.0) ); 
        return { M };
    }

    Wind build_wind(double heading_deg, double spd_kts) {
        double psi_wind = util::deg_to_rad(heading_deg);
        double v_wind = util::kts_to_mps(spd_kts);

        double wind_n = -v_wind * util::cos(psi_wind);
        double wind_e = -v_wind * util::sin(psi_wind);
        double wind_d = 0.0;

        return { Eigen::Vector3d(wind_n, wind_e, wind_d) };
    }

}
