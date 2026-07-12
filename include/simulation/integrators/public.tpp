#pragma once
#include "simulation/integrators/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/propulsion/public.hpp"

namespace integrators {

    template <typename T, typename Model>
    WrenchSet_T<T> compute_wrench_set_T(const Model& model, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& atm, const actuators::ActuatorInputs_T<T>& u, const propulsion::PropellerOmegaDotSet_T<T>& propeller_omega_dot_set, const atmospheric::Wind& windB, const constants::Vector3_T<T>& gB) {
        const dynamics::Wrench_T<T> aerodynamic = aerodynamics::step_aero_forces_moments_T<T>(model.aerodynamic, twist, atm, u.surface_inputs, windB);
        const dynamics::Wrench_T<T> propulsive = propulsion::step_propulsive_forces_moments_T<T>(model.propulsor_actuators, twist, atm, u.propulsor_inputs, propeller_omega_dot_set);

        return {
            .aerodynamic = aerodynamic,
            .propulsive = propulsive,
            .net = {
                .F = aerodynamic.F + propulsive.F + T(model.structural.mass.data) * gB,
                .M = aerodynamic.M + propulsive.M
            }
        };
    }

}
