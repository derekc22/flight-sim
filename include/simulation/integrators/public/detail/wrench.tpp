#pragma once
#include <type_traits>
#include "simulation/constants/public/linalg.hpp"
#include "simulation/integrators/public/detail/wrench.hpp"
#include "simulation/aerodynamics/public/detail/loads.hpp"
#include "simulation/propulsion/public/manager.hpp"

namespace integrators {

    template <typename T, typename Model>
    WrenchEvaluation_T<T> compute_wrench_set_T(const Model& model, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& atm, const actuators::ActuatorInputs_T<T>& u, const atmospheric::Wind& windB, const constants::Vector3_T<T>& gB, T dt, bool steady_state) {
        const constants::Vector3_T<T> pB_GB = model.struc_t.pB_GB.data.template cast<T>();
        const dynamics::Wrench_T<T> WB_aerodynamic = aerodynamics::step_aero_forces_moments_T<T>(model.aerodynamic.surfaces, pB_GB, twist, atm, u.surface_inputs, windB);

        const propulsion::PropulsionManagerInput_T<T> propulsion_input{
            .propulsors = model.propulsor_actuators,
            .pB_GB = pB_GB,
            .twist = twist,
            .atm = atm,
            .u = u.propulsor_inputs,
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

        const dynamics::Wrench_T<T>& WB_propulsive = propulsion_output.WB_propulsive;

        return {
            .WB_set = {
                .aerodynamic = WB_aerodynamic,
                .propulsive = WB_propulsive,
                .net = {
                    .F = WB_aerodynamic.F + WB_propulsive.F + T(model.struc_t.mass.data) * gB,
                    .M = WB_aerodynamic.M + WB_propulsive.M
                }
            },
            .propulsion_state_t = propulsion_output.state_t
        };
    }

}
