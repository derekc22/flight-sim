#include <cmath>
#include <format>
#include <stdexcept>
#include "simulation/atmospheric/atmospheric.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/frames/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/util/public.hpp"

namespace atmospheric {

    StaticAtmosphericState compute_static_atmospheric_state(const frames::Frame& F) {
        if (F.parent != nullptr) {
            throw std::invalid_argument(std::format("atmospheric::static_atm_state: Invalid frame input, the parent of {} must be ECEFFrame", F.name));
        }
        return atmospheric::std_atmosphere(geography::compute_geographic_state(F).alt);
    }

    StaticAirTemperature T_from_T0(const StagnationAirTemperature& T0, const MachNumber& M) {
        double T = T0.data / ( 1 + ((constants::gamma_air - 1) / 2) * M.data * M.data );
        return { T };
    };

    MachNumber mps_to_mach(const dynamics::TranslationalVelocity& v, const StaticAirTemperature& T){
        double a = std::sqrt(constants::gamma_air * constants::R_air * T.data);
        double M = util::vector_norm(v.data)/a;
        return { M };
    }

    StagnationAtmosphericState static_to_stagnation(const StaticAtmosphericState& static_atm_state, const MachNumber& M) {
        AirDensity rho = static_atm_state.rho;
        DynamicViscosity mu = static_atm_state.mu;

        StagnationAirTemperature T0 = T0_from_T(static_atm_state.T, M);
        StagnationAirPressure P0 = P0_from_P(static_atm_state.P, M);

        return { T0, P0, rho, mu };
    };

    MachNumber compute_mach(const StagnationAirPressure& P0, const StaticAirPressure& P){
        double M = std::sqrt( (2.0 / (constants::gamma_air - 1.0)) * (std::pow( P0.data / P.data, (constants::gamma_air - 1.0) / constants::gamma_air ) - 1.0) ); 
        return { M };
    }

    Wind build_wind(double heading_deg, double spd_kts){
        double psi_wind = util::deg_to_rad(heading_deg);
        double V_wind = util::kts_to_mps(spd_kts);

        double wind_N = -V_wind * util::cos(psi_wind);
        double wind_E = -V_wind * util::sin(psi_wind);
        double wind_D = 0.0;

        return { Eigen::Vector3d(wind_N, wind_E, wind_D) };
    }

}
