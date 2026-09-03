#include <array>
#include <cmath>
#include "simulation/atmospheric/private/detail/state.hpp"
#include "simulation/constants/public/scalars.hpp"

namespace atmospheric {

    StaticAtmosphericState std_atmosphere(const geography::GeometricAltitude& altitude) {

        double alt = altitude.data;

        // Values at 11 km (tropopause in this simplified model)
        const double temperature_11 = constants::T_SL + constants::lapse_rate * 11000.0;
        const double density_11 = constants::rho_SL * std::pow(temperature_11 / constants::T_SL, -((constants::g_earth / (constants::lapse_rate * constants::R_air)) + 1.0));

        double T = 0.0;     // Static temperature, T [K]
        double P = 0.0;     // Static pressure, P [Pa]
        double rho = 0.0;   // Air density, ρ [kg/m^3]

        // Gradient region
        if (alt <= 11000.0) {
            T = constants::T_SL + constants::lapse_rate * alt;
            rho = constants::rho_SL * std::pow(T / constants::T_SL, -((constants::g_earth / (constants::lapse_rate * constants::R_air)) + 1.0));
        }
        // Isothermal region
        else {
            T = temperature_11;
            rho = density_11 * std::exp(-constants::g_earth * ((alt - 11000.0) / (constants::R_air * temperature_11)));
        }

        // Viscosity
        const double mu = 1.54 * (1.0 + 0.0039 * (T - 250.0)) * 1e-5;  // Viscosity, μ [kg/m·s]

        P = rho * constants::R_air * T;
 
        return StaticAtmosphericState{ T, P, rho, mu };
    }

}
