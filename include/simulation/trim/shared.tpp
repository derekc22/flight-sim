#pragma once
#include "simulation/actuators/propulsor/shared.hpp"
#include "simulation/actuators/surface/shared.hpp"
#include "simulation/actuators/shared.hpp"
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/propulsion/shared.hpp"
#include "simulation/trim/shared.hpp"
#include "simulation/util/util.hpp"

namespace trim {

    template <typename T>
    dynamics::Twist_T<T> build_twist_from_trim_state_T(const dynamics::State_T<T>& x) {
        dynamics::Twist_T<T> twist;
        twist.v << x.vx, x.vy, x.vz;
        twist.w << x.p, x.q, x.r;
        return twist;
    }

    template <typename T>
    actuators::SurfaceActuatorInputs_T<T> build_surface_actuator_inputs_from_trim_T(const actuators::ActuatorInputs_T<T>& u, const actuators::FixedActuatorInputs_T& fixed_actuator_inputs) {
        return {
            .elevator_cmd = u.elevator_cmd,
            .aileron_cmd = u.aileron_cmd,
            .rudder_cmd = u.rudder_cmd,
            .flap_cmd = T(fixed_actuator_inputs.flap),
            .spoiler_cmd = T(fixed_actuator_inputs.spoiler),
        };
    }

    template <typename T>
    actuators::PropulsorActuatorInputs_T<T> build_propulsor_actuator_inputs_from_trim_T(const actuators::ActuatorInputs_T<T>& u) {
        return {
            .front_propulsor_cmd = u.front_propulsor_cmd,
            .left_propulsor_cmd = u.left_propulsor_cmd,
            .right_propulsor_cmd = u.right_propulsor_cmd
        };
    }

    template <typename T>
    constants::Vector3_T<T> gB_T(const T& phi, const T& theta) {
        constants::Vector3_T<T> gB;
        gB << -T(constants::g_earth) * util::sin(theta),
               T(constants::g_earth) * util::sin(phi) * util::cos(theta),
               T(constants::g_earth) * util::cos(phi) * util::cos(theta);
        return gB;
    }

    template <typename T>
    dynamics::Wrench_T<T> compute_trim_net_wrench_T(const dynamics::State_T<T>& x, const dynamics::Twist_T<T>& twist, const actuators::ActuatorInputs_T<T>& u, const TrimModel& model, const TrimConditions& conditions) {
        const actuators::SurfaceActuatorInputs_T<T> surface_actuator_inputs = build_surface_actuator_inputs_from_trim_T(u, model.fixed_actuator_inputs);
        const dynamics::Wrench_T<T> aero_wrench = aerodynamics::step_aero_forces_moments_T<T>(model.aerodynamic, model.structural, twist, conditions.static_atm_state, surface_actuator_inputs, conditions.windB);

        const actuators::PropulsorActuatorInputs_T<T> propulsor_actuator_inputs = build_propulsor_actuator_inputs_from_trim_T(u);
        const dynamics::Wrench_T<T> prop_wrench = propulsion::step_propulsive_forces_moments_T<T>(model.propulsor_actuators, twist, conditions.static_atm_state, propulsor_actuator_inputs, propulsion::PropulsorOmegaDot_T<T>{});

        return {
            .F = aero_wrench.F + prop_wrench.F + T(model.structural.Mass.data) * gB_T(x.phi, x.theta),
            .M = aero_wrench.M + prop_wrench.M,
        };
    }

    template <typename T>
    dynamics::StateDot_T<T> compute_trim_state_dot_T(const dynamics::State_T<T>& x, const actuators::ActuatorInputs_T<T>& u, const TrimModel& model, const TrimConditions& conditions) {
        const dynamics::Twist_T<T> twist = build_twist_from_trim_state_T(x);
        const dynamics::Wrench_T<T> net_wrench = compute_trim_net_wrench_T<T>(x, twist, u, model, conditions);
        const constants::Vector3_T<T> v_dot = dynamics::_ddtB_vB_BI_T<T>(twist.v, twist.w, model.structural.Mass.data, net_wrench.F);
        const constants::Vector3_T<T> w_dot = dynamics::_ddtB_wB_BI_T<T>(twist.w, model.structural.J.data, net_wrench.M);
        const constants::Vector3_T<T> eul_dot = dynamics::_wB_BI_to_eul_dot_T<T>(twist.w, x.theta, x.phi);

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

    template <typename T>
    TrimVariablesVector_T<T> unpack_trim_variables_T(const dynamics::State_T<T>& x, const actuators::ActuatorInputs_T<T>& u) {
        TrimVariablesVector_T<T> out;
        out << x.vx, x.vy, x.vz,
               x.p, x.q, x.r,
               x.phi, x.theta,
               u.elevator_cmd, u.aileron_cmd, u.rudder_cmd,
               u.front_propulsor_cmd, u.left_propulsor_cmd, u.right_propulsor_cmd;
        return out;
    }

    template <typename T>
    dynamics::State_T<T> pack_trim_state_T(const TrimVariablesVector_T<T>& z) {
        return {
            .vx = z(0),
            .vy = z(1),
            .vz = z(2),
            .p = z(3),
            .q = z(4),
            .r = z(5),
            .phi = z(6),
            .theta = z(7),
        };
    }

    template <typename T>
    actuators::ActuatorInputs_T<T> pack_trim_actuator_inputs_T(const TrimVariablesVector_T<T>& z) {
        return {
            .elevator_cmd = z(8),
            .aileron_cmd = z(9),
            .rudder_cmd = z(10),

            .front_propulsor_cmd = z(11),
            .left_propulsor_cmd = z(12),
            .right_propulsor_cmd = z(13),
        };
    }

}
