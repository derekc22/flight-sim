#pragma once
#include "simulation/autodiff/public/detail/evaluation.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public/data/helpers.hpp"
#include "simulation/dynamics/public/detail/derivatives.hpp"
#include "simulation/geography/public/detail/gravity.hpp"
#include "simulation/integrators/public/detail/wrench.hpp"

namespace autodiff {

    template <typename T>
    dynamics::Wrench_T<T> compute_net_wrench_T(const operating::OperatingPoint_T<T>& operating_point, const AutoDiffModel& model, const operating::OperatingConditions& conditions, T dt) {
        const dynamics::State_T<T> x = operating_point.state;
        actuators::ActuatorInputs_T<T> u = operating_point.input;

        const dynamics::Twist_T<T> twist = dynamics::build_twist_from_state_T(x);

        u.surface_inputs.flap_cmd = T(model.fixed_actuator_inputs.flap);
        u.surface_inputs.spoiler_cmd = T(model.fixed_actuator_inputs.spoiler);

        // recall, the definition of steady trim is ẋ = f(x, u) = 0
        // that is, for trim, d/dt(·) = 0 must be enforced for all modelled state variables
        // however, despite this repo not including omega as part of the modelled aircraft state, omega_dot is also set to 0
        // this is done because nonzero omega_dot represents a non-steady propeller transient that induces a moment on the aircraft
        // this moment, if unbalanced, will result in ẋ != 0, which violates the condition of steady equilibrium
        // thus, setting omega_dot = 0 allows trim to be achieved
        // note: nonzero omega_dot only violates trim for the reduced-order model if the moment results in any of the modelled aircraft states changing (ẋ != 0)
        // however, for a higher-order model, omega is necessarily included as a modelled aircraft state. as such, omega_dot = 0 is required in this case - regardless of whether it affects other states

        const constants::Vector3_T<T> gB = geography::gB_T(x.phi, x.theta);

        const integrators::WrenchEvaluation_T<T> evaluation = integrators::compute_wrench_set_T<T>(
            model,
            twist,
            conditions.atm,
            u,
            conditions.windB,
            gB,
            dt,
            conditions.steady_state // set omega_dot = 0 if computing gradients for trim (steady state)
        );

        return evaluation.WB_set.net;
    }

    template <typename T>
    dynamics::StateDot_T<T> compute_state_dot_T(const operating::OperatingPoint_T<T>& operating_point, const AutoDiffModel& model, const operating::OperatingConditions& conditions, T dt) {
        const dynamics::Wrench_T<T> net_wrench = compute_net_wrench_T<T>(operating_point, model, conditions, dt);
        return compute_state_dot_from_net_wrench_T(operating_point.state, model, net_wrench);
    }

    template <typename T>
    dynamics::StateDot_T<T> compute_state_dot_T(const operating::VirtualOperatingPoint_T<T>& operating_point, const AutoDiffModel& model) {
        return compute_state_dot_from_net_wrench_T(operating_point.state, model, operating_point.input);
    }

    template <typename T>
    dynamics::StateDot_T<T> compute_state_dot_from_net_wrench_T(const dynamics::State_T<T>& x, const AutoDiffModel& model, const dynamics::Wrench_T<T>& net_wrench) {
        const dynamics::Twist_T<T> twist = dynamics::build_twist_from_state_T(x);
        const constants::Vector3_T<T> v_dot = dynamics::ddtB_vB_BI_T<T>(twist.v, twist.w, model.struc_t.mass.data, net_wrench.F);
        const constants::Vector3_T<T> w_dot = dynamics::ddtB_wB_BI_T<T>(twist.w, model.struc_t.JB.data, net_wrench.M);
        const constants::Vector3_T<T> eul_dot = dynamics::wB_BI_to_eul_dot_T<T>(twist.w, x.theta, x.phi);

        return {
            .vx_dot = v_dot.x(),
            .vy_dot = v_dot.y(),
            .vz_dot = v_dot.z(),
            .p_dot = w_dot.x(),
            .q_dot = w_dot.y(),
            .r_dot = w_dot.z(),
            .phi_dot = eul_dot.x(),
            .theta_dot = eul_dot.y(),
        };
    }

}
