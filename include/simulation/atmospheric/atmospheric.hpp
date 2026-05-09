#pragma once
#include "simulation/atmospheric/shared.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/geography/shared.hpp"

namespace frames { struct Frame; } // forward declare

namespace atmospheric {

    /** @warning The parent of F must the ECEFFrame */
    StaticAtmosphericState compute_static_atmospheric_state(const frames::Frame& F);

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
