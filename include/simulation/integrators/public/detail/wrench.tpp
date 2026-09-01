#pragma once
#include <type_traits>
#include "simulation/integrators/public/detail/wrench.hpp"
#include "simulation/aerodynamics/public/detail/loads.hpp"
#include "simulation/propulsion/public/manager.hpp"

namespace integrators {

    template <typename T, typename Model>
    WrenchEvaluation_T<T> compute_wrench_set_T(const Model& model, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& atm, const actuators::ActuatorInputs_T<T>& u, const atmospheric::Wind& windB, const constants::Vector3_T<T>& gB, T dt, bool steady_state) {
        const constants::Vector3_T<T> pB_GB = model.struc_t.pB_GB.data.template cast<T>();
        const dynamics::Wrench_T<T> aerodynamic = aerodynamics::step_aero_forces_moments_T<T>(model.aerodynamic.surfaces, pB_GB, twist, atm, u.surface_inputs, windB);

        const propulsion::PropulsionManagerInput_T<T> propulsion_input{
            .propulsors = model.propulsor_actuators,
            .center_of_gravity = pB_GB,
            .vehicle_twist = twist,
            .atmosphere = atm,
            .actuator_inputs = u.propulsor_inputs,
            .dt = dt,
            .steady_state = steady_state
        };

        const propulsion::PropulsionManagerOutput_T<T> propulsion_output = [&]() {
            if constexpr (std::is_same_v<T, double>) {
                return model.propulsion.step(propulsion_input);
            }
            else {
                return model.propulsion.template step_T<T>(propulsion_input);
            }
        }();

        const dynamics::Wrench_T<T>& propulsive = propulsion_output.propulsive_wrench;

        return {
            .wrench = {
                .aerodynamic = aerodynamic,
                .propulsive = propulsive,
                .net = {
                    .F = aerodynamic.F + propulsive.F + T(model.struc_t.mass.data) * gB,
                    .M = aerodynamic.M + propulsive.M
                }
            },
            .next_propulsion_state = propulsion_output.next_state
        };
    }

}
