#include <array>
#include "simulation/atmospheric/atmospheric.hpp"
#include "simulation/geography/geography.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/util/util.hpp"

namespace atmospheric {



    StaticAtmosphericState std_atmosphere(const geography::Altitude& altitude){

        double h = altitude.data;

        // Sea-level conditions
        constexpr double rho_SL = 1.225;     // kg/m^3
        constexpr double T_SL = 288.0; // K

        // Constants
        constexpr double lapse_rate = -0.0065;  // K/m

        // Values at 11 km (tropopause in this simplified model)
        const double temperature_11 = T_SL + lapse_rate * 11000.0;
        const double density_11 = rho_SL * std::pow(temperature_11 / T_SL, -((constants::g_earth / (lapse_rate * constants::R_air)) + 1.0));

        double T = 0.0;     // Static temperature, T [K]
        double P = 0.0;     // Static pressure, P [Pa]
        double rho = 0.0;   // AirDensity, ρ [kg/m^3]

        // Gradient region
        if (h <= 11000.0) {
            T = T_SL + lapse_rate * h;
            rho = rho_SL * std::pow(T / T_SL, -((constants::g_earth / (lapse_rate * constants::R_air)) + 1.0));
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

    MachNumber ms_to_mach(const dynamics::LinearVelocity& v, const StaticAirTemperature& T){
        double a = std::sqrt(constants::gamma_air * constants::R_air * T.data);
        double M = util::vector_norm(v.data)/a;
        return { M };
    }

    StagnationAtmosphericState static_to_stagnation(const StaticAtmosphericState& static_atmospheric_state, const MachNumber& M) {
        double rho = static_atmospheric_state.rho.data;
        double mu = static_atmospheric_state.mu.data;

        double T0 = T0_from_T(static_atmospheric_state.T, M).data;
        double P0 = P0_from_P(static_atmospheric_state.P, M).data;

        return { T0, P0, rho, mu };
    };



}