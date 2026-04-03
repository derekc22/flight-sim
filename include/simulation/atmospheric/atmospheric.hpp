#pragma once
#include "simulation/geography/geography.hpp"

namespace atmospheric {


    struct Wind {
        Eigen::Vector3d data;
    };

    struct StaticAirTemperature {
        double data;   // T_static [K]
    };

    struct StagnationAirTemperature {
        double data;   // T_total [K]
    };

    struct AirDensity {
        double data;   // ρ [kg/m^3]
    };

    struct StaticAirPressure {
        double data;   // P_static [Pa]
    };

    struct StagnationAirPressure {
        double data;   // P_total [Pa]
    };

    struct DynamicViscosity {
        double data;   // μ [kg/m·s]
    };

    struct MachNumber {
        double data;   // M [kg/m·s]
    };

    struct StaticAtmosphericState {
        StaticAirTemperature T;
        StaticAirPressure P;
        AirDensity rho;
        DynamicViscosity mu;
    };

    struct StagnationAtmosphericState {
        StagnationAirPressure T0;
        StagnationAirPressure P0;
        AirDensity rho;
        DynamicViscosity mu;
    };


    StaticAtmosphericState std_atmosphere(const geography::Altitude& altitude);









}