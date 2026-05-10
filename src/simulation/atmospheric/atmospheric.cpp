#include <array>
#include <format>
#include <stdexcept>
#include "simulation/atmospheric/atmospheric.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/frames/frames.hpp"
#include "simulation/geography/geography.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/util/util.hpp"

namespace atmospheric {

    StaticAtmosphericState compute_static_atmospheric_state(const frames::Frame& F) {
        if (F.parent != nullptr) {
            throw std::invalid_argument(std::format("atmospheric::static_atm_state: Invalid frame input, the parent of {} must be ECEFFrame", F.name));
        }
        return atmospheric::std_atmosphere(geography::compute_geographic_state(F).alt);
    }

    StaticAtmosphericState std_atmosphere(const geography::Altitude& altitude){

        double h = altitude.data;

        // Values at 11 km (tropopause in this simplified model)
        const double temperature_11 = constants::T_SL + constants::lapse_rate * 11000.0;
        const double density_11 = constants::rho_SL * std::pow(temperature_11 / constants::T_SL, -((constants::g_earth / (constants::lapse_rate * constants::R_air)) + 1.0));

        double T = 0.0;     // Static temperature, T [K]
        double P = 0.0;     // Static pressure, P [Pa]
        double rho = 0.0;   // Air density, ρ [kg/m^3]

        // Gradient region
        if (h <= 11000.0) {
            T = constants::T_SL + constants::lapse_rate * h;
            rho = constants::rho_SL * std::pow(T / constants::T_SL, -((constants::g_earth / (constants::lapse_rate * constants::R_air)) + 1.0));
        }
        // Isothermal region
        else {
            T = temperature_11;
            rho = density_11 * std::exp(-constants::g_earth * ((h - 11000.0) / (constants::R_air * temperature_11)));
        }

        // Viscosity
        const double mu = 1.54 * (1.0 + 0.0039 * (T - 250.0)) * 1e-5;  // Viscosity, μ [kg/m·s]

        P = rho * constants::R_air * T;
 
        return StaticAtmosphericState{ T, P, rho, mu };
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


    Wind build_wind(double heading_deg, double spd_kts){
        double psi_wind = util::deg_to_rad(heading_deg);
        double V_wind = util::kts_to_mps(spd_kts);

        double wind_N = -V_wind * util::cos(psi_wind);
        double wind_E = -V_wind * util::sin(psi_wind);
        double wind_D = 0.0;

        return { Eigen::Vector3d(wind_N, wind_E, wind_D) };
    }

}
