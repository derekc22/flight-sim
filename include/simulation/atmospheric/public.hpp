#pragma once
#include <Eigen/Dense>
#include "simulation/dynamics/public.hpp"
#include "simulation/frames/public.hpp"
#include "simulation/geography/public.hpp"

namespace atmospheric {

    struct Wind {
        Eigen::Vector3d data;
    };

    struct StaticAirTemperature {
        double data;
    };

    struct StagnationAirTemperature {
        double data;
    };

    struct AirDensity {
        double data;
    };

    struct StaticAirPressure {
        double data;
    };

    struct StagnationAirPressure {
        double data;
    };

    struct DynamicViscosity {
        double data;
    };

    struct MachNumber {

        double data;
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

    /** @warning The parent of F must the ECEFFrame */
    StaticAtmosphericState compute_static_atmospheric_state(const frames::Frame& F);

    StaticAirTemperature T_from_T0(const StagnationAirTemperature& T0, const MachNumber& M);
    MachNumber mps_to_mach(const dynamics::TranslationalVelocity& v, const StaticAirTemperature& T);
    StagnationAtmosphericState static_to_stagnation(const StaticAtmosphericState& atm, const MachNumber& M);
    MachNumber compute_mach(const StagnationAirPressure& P0, const StaticAirPressure& P);
    Wind build_wind(double heading_deg, double spd_kts);

}
