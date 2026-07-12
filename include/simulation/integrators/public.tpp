#pragma once
#include "simulation/integrators/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/propulsion/public.hpp"

namespace integrators {

    template <typename T>
    WrenchSet_T<T> compute_wrench_set_T(const aerodynamics::AerodynamicProperties& aerodynamic_properties, const actuators::PropulsorActuators& propulsor_actuators, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& atm, const actuators::ActuatorInputs_T<T>& u, const propulsion::PropulsorOmegaDot_T<T>& omega_dot, const atmospheric::Wind& windB, double mass, const constants::Vector3_T<T>& gB) {
        const dynamics::Wrench_T<T> aerodynamic = aerodynamics::step_aero_forces_moments_T<T>(aerodynamic_properties, twist, atm, u.surface_inputs, windB);
        const dynamics::Wrench_T<T> propulsive = propulsion::step_propulsive_forces_moments_T<T>(propulsor_actuators, twist, atm, u.propulsor_inputs, omega_dot);

        return {
            .aerodynamic = aerodynamic,
            .propulsive = propulsive,
            .net = {
                .F = aerodynamic.F + propulsive.F + T(mass) * gB,
                .M = aerodynamic.M + propulsive.M
            }
        };
    }

}
