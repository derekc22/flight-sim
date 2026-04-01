#pragma once
#include <array>
#include "simulation/geography/geography.hpp"

namespace atmospheric {


    struct Wind {
        Eigen::Vector3d data;
    };


    struct Temperature {
        double data;   // [K]
    };

    struct Density {
        double data;   // ρ [kg/m^3]
    };

    struct StagnationPressure {
        double data;   // P_total [Pa]
    };

    struct StaticPressure {
        double data;   // P_static [Pa]
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