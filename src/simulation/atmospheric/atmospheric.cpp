#include <array>
#include "simulation/atmospheric/atmospheric.hpp"

namespace atmospheric {



    std::array<double, 3> StandardAtmosphere::measure(double height){

        // Sea-level conditions
        constexpr double density_SL = 1.225;     // kg/m^3
        constexpr double temperature_SL = 288.0; // K

        // Constants
        constexpr double g = 9.81;              // m/s^2
        constexpr double lapse_rate = -0.0065;  // K/m
        constexpr double gas_constant = 287.0;  // J/(kg·K)

        // Values at 11 km (tropopause in this simplified model)
        const double temperature_11 = temperature_SL + lapse_rate * 11000.0;
        const double density_11 = density_SL * std::pow(temperature_11 / temperature_SL, -((g / (lapse_rate * gas_constant)) + 1.0));

        double temperature = 0.0;   // T [K]
        double density = 0.0;       // ρ [kg/m^3]

        // Gradient region
        if (height <= 11000.0) {
            temperature = temperature_SL + lapse_rate * height;
            density = density_SL * std::pow(temperature / temperature_SL, -((g / (lapse_rate * gas_constant)) + 1.0));
        }
        // Isothermal region
        else {
            temperature = temperature_11;
            density = density_11 * std::exp(-g * ((height - 11000.0) / (gas_constant * temperature_11)));
        }

        // Viscosity
        const double viscosity = 1.54 * (1.0 + 0.0039 * (temperature - 250.0)) * 1e-5;  // μ [kg/m·s]

 
        return { temperature, density, viscosity };
    }






}