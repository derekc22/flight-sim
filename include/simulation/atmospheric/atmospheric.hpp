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
        StagnationAirTemperature T0;
        StagnationAirPressure P0;
        AirDensity rho;
        DynamicViscosity mu;
    };


    StaticAtmosphericState std_atmosphere(const geography::Altitude& altitude);

    StagnationAirTemperature T0_from_T(const StaticAirTemperature& T, const MachNumber& M);

    StagnationAirPressure P0_from_P(const StaticAirPressure& P, const MachNumber& M);

    StaticAirTemperature T_from_T0(const StagnationAirTemperature& T0, const MachNumber& M);

    StaticAirPressure P_from_P0(const StagnationAirPressure& P0, const MachNumber& M);

    MachNumber ms_to_mach(const dynamics::LinearVelocity& v, const StaticAirTemperature& T);

    StagnationAtmosphericState static_to_stagnation(const StaticAtmosphericState& static_atmospheric_state, const MachNumber& M);

    StaticAtmosphericState stagnation_to_static(const StagnationAtmosphericState& stagnation_atmospheric_state, const MachNumber& M);

    MachNumber compute_mach(const StagnationAirPressure& P0, const StaticAirPressure& P);

}
