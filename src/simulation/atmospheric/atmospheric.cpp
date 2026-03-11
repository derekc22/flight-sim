#include <array>
#include "simulation/atmospheric/atmospheric.hpp"
#include "simulation/geography/geography.hpp"

namespace atmospheric {



    AtmosphericState std_atmosphere(const geography::Altitude& height){

        double h = height.data;

        // Sea-level conditions
        constexpr double density_SL = 1.225;     // kg/m^3
        constexpr double temperature_SL = 288.0; // K

        // Constants
        constexpr double g = 9.81;              // m/s^2    // DO NOT SET TO ZERO WHEN RUNNING GRAVITY-FREE TESTS! WHILE RIGID BODY DYNAMICS CAN BE SIMULATED WITHOUT GRAVITY, THIS ATMOSPHERE MODEL CANNOT - THUS, THIS VALUE MUST BE MAINTAINED INDEPENDENTLY OF global::gravity
        constexpr double lapse_rate = -0.0065;  // K/m
        constexpr double gas_constant = 287.0;  // J/(kg·K)

        // Values at 11 km (tropopause in this simplified model)
        const double temperature_11 = temperature_SL + lapse_rate * 11000.0;
        const double density_11 = density_SL * std::pow(temperature_11 / temperature_SL, -((g / (lapse_rate * gas_constant)) + 1.0));

        double temperature = 0.0;   // T [K]
        double density = 0.0;       // ρ [kg/m^3]

        // Gradient region
        if (h <= 11000.0) {
            temperature = temperature_SL + lapse_rate * h;
            density = density_SL * std::pow(temperature / temperature_SL, -((g / (lapse_rate * gas_constant)) + 1.0));
        }
        // Isothermal region
        else {
            temperature = temperature_11;
            density = density_11 * std::exp(-g * ((h - 11000.0) / (gas_constant * temperature_11)));
        }

        // Viscosity
        const double viscosity = 1.54 * (1.0 + 0.0039 * (temperature - 250.0)) * 1e-5;  // μ [kg/m·s]

 
        return AtmosphericState{ temperature, density, viscosity };
    }






}