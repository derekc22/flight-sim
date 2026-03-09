#pragma once
#include <array>
#include "simulation/geography/geography.hpp"

namespace atmospheric {


    struct Wind {
        
    };


    struct Temperature {
        double data;   // [K]
    };

    struct Density {
        double data;   // ρ [kg/m^3]
    };

    struct DynamicViscosity {
        double data;   // μ [kg/m·s]
    };

    struct AtmosphericState {
        Temperature T;
        Density rho;
        DynamicViscosity mu;
    };

    AtmosphericState std_atmosphere(const geography::Altitude& height);









}