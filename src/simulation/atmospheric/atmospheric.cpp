#include <array>
#include "simulation/atmospheric/atmospheric.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/util/util.hpp"

namespace atmospheric {

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

    Wind build_wind(double heading_deg, double spd_kts){
        double psi_wind = util::deg_to_rad(heading_deg);
        double V_wind = util::kts_to_mps(spd_kts);

        double wind_N = -V_wind * util::cos(psi_wind);
        double wind_E = -V_wind * util::sin(psi_wind);
        double wind_D = 0.0;

        return { Eigen::Vector3d(wind_N, wind_E, wind_D) };
    }

}
