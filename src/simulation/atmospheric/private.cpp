#include <array>
#include <cmath>
#include "simulation/atmospheric/private.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/util/public.hpp"

namespace atmospheric {

    StaticAtmosphericState std_atmosphere(const geography::Altitude& altitude) {

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

    StaticAirPressure P_from_P0(const StagnationAirPressure& P0, const MachNumber& M) {
        double P = P0.data / std::pow( 1 + ((constants::gamma_air - 1) / 2) * M.data * M.data, constants::gamma_air / (constants::gamma_air - 1) );
        return { P };
    };

    StaticAtmosphericState stagnation_to_static(const StagnationAtmosphericState& stag_atm, const MachNumber& M) {
        AirDensity rho = stag_atm.rho;
        DynamicViscosity mu = stag_atm.mu;

        StaticAirTemperature T = T_from_T0(stag_atm.T0, M);
        StaticAirPressure P = P_from_P0(stag_atm.P0, M);

        return { T, P, rho, mu };
    };
}
