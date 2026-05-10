#include <cmath>
#include <format>
#include <stdexcept>
#include "simulation/atmospheric/atmospheric.hpp"
#include "simulation/atmospheric/shared.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/frames/frames.hpp"
#include "simulation/geography/shared.hpp"
#include "simulation/util/util.hpp"

namespace atmospheric {

    StaticAtmosphericState compute_static_atmospheric_state(const frames::Frame& F) {
        if (F.parent != nullptr) {
            throw std::invalid_argument(std::format("atmospheric::static_atm_state: Invalid frame input, the parent of {} must be ECEFFrame", F.name));
        }
        return atmospheric::std_atmosphere(geography::compute_geographic_state(F).alt);
    }

    StagnationAirTemperature T0_from_T(const StaticAirTemperature& T, const MachNumber& M) {
        double T0 = T.data * ( 1 + ((constants::gamma_air - 1) / 2) * M.data * M.data );
        return { T0 };
    };

    StagnationAirPressure P0_from_P(const StaticAirPressure& P, const MachNumber& M) {
        double P0 = P.data * std::pow( 1 + ((constants::gamma_air - 1) / 2) * M.data * M.data, constants::gamma_air/(constants::gamma_air - 1) );
        return { P0 };
    };

    StaticAirTemperature T_from_T0(const StagnationAirTemperature& T0, const MachNumber& M) {
        double T = T0.data / ( 1 + ((constants::gamma_air - 1) / 2) * M.data * M.data );
        return { T };
    };

    StaticAirPressure P_from_P0(const StagnationAirPressure& P0, const MachNumber& M) {
        double P = P0.data / std::pow( 1 + ((constants::gamma_air - 1) / 2) * M.data * M.data, constants::gamma_air / (constants::gamma_air - 1) );
        return { P };
    };

    MachNumber ms_to_mach(const dynamics::LinearVelocity& v, const StaticAirTemperature& T){
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

    StaticAtmosphericState stagnation_to_static(const StagnationAtmosphericState& stagnation_atmospheric_state, const MachNumber& M) {
        AirDensity rho = stagnation_atmospheric_state.rho;
        DynamicViscosity mu = stagnation_atmospheric_state.mu;

        StaticAirTemperature T = T_from_T0(stagnation_atmospheric_state.T0, M);
        StaticAirPressure P = P_from_P0(stagnation_atmospheric_state.P0, M);

        return { T, P, rho, mu };
    };

    MachNumber compute_mach(const StagnationAirPressure& P0, const StaticAirPressure& P){
        double M = std::sqrt( (2.0 / (constants::gamma_air - 1.0)) * (std::pow( P0.data / P.data, (constants::gamma_air - 1.0) / constants::gamma_air ) - 1.0) ); 
        return { M };
    }

}
