#pragma once
#include "simulation/actuators/propulsor/shared.hpp"
#include "simulation/actuators/surface/shared.hpp"
#include "simulation/actuators/shared.hpp"
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/util/util.hpp"

namespace trim {
    template <typename T>
    T get_control_from_solver_space_T(const T& u_solver, double limit_min, double limit_max) {
        const T mid = T(0.5 * (limit_max + limit_min));
        const T half_range = T(0.5 * (limit_max - limit_min));
        return mid + half_range * (u_solver / util::sqrt(T(1) + u_solver * u_solver));
    }

    template <typename T>
    actuators::ActuatorInputs_T<T> pack_trim_actuator_inputs_T(const TrimVariablesVector_T<T>& z, const actuators::ActuatorLimits_T<double>& actuator_limits) {
        return {
            .elevator_cmd = get_control_from_solver_space_T<T>(z(constants::state_dim + 0), actuator_limits.limit_min.elevator_cmd, actuator_limits.limit_max.elevator_cmd),
            .aileron_cmd = get_control_from_solver_space_T<T>(z(constants::state_dim + 1), actuator_limits.limit_min.aileron_cmd, actuator_limits.limit_max.aileron_cmd),
            .rudder_cmd = get_control_from_solver_space_T<T>(z(constants::state_dim + 2), actuator_limits.limit_min.rudder_cmd, actuator_limits.limit_max.rudder_cmd),
            .front_propulsor_cmd = get_control_from_solver_space_T<T>(z(constants::state_dim + 3), actuator_limits.limit_min.front_propulsor_cmd, actuator_limits.limit_max.front_propulsor_cmd),
            .left_propulsor_cmd = get_control_from_solver_space_T<T>(z(constants::state_dim + 4), actuator_limits.limit_min.left_propulsor_cmd, actuator_limits.limit_max.left_propulsor_cmd),
            .right_propulsor_cmd = get_control_from_solver_space_T<T>(z(constants::state_dim + 5), actuator_limits.limit_min.right_propulsor_cmd, actuator_limits.limit_max.right_propulsor_cmd),
        };
    }

    template <typename T>
    TrimResidual<T> compute_trim_residual(const dynamics::State_T<T>& x, const actuators::ActuatorInputs_T<T>& u, const TrimModel& model, const TrimTarget& target, const TrimConditions& conditions) {
        const dynamics::StateDot_T<T> trim_state_dot = compute_trim_state_dot_T<T>(x, u, model, conditions);
        const dynamics::Twist_T<T> twist = build_twist_from_trim_state_T(x);
        const aerodynamics::AerodynamicState_T<T> ads = aerodynamics::compute_aerodynamic_state_T<T>(twist, conditions.windB);
        const constants::Vector3_T<T> eul_dot = dynamics::_wB_BI_to_eul_dot_T<T>(twist.w, x.theta, x.phi);

        return {
            .vx_dot = trim_state_dot.vx_dot,
            .vy_dot = trim_state_dot.vy_dot,
            .vz_dot = trim_state_dot.vz_dot,
            .p_dot = trim_state_dot.p_dot,
            .q_dot = trim_state_dot.q_dot,
            .r_dot = trim_state_dot.r_dot,
            .phi_dot = trim_state_dot.phi_dot,
            .theta_dot = trim_state_dot.theta_dot,
            .beta_err = ads.beta - T(target.beta),
            .phi_err = x.phi - T(target.phi),
            .theta_err = x.theta - T(target.theta),
            .vx_err = x.vx - T(target.vx),
            .vz_err = x.vz - T(target.vz),
            .psi_dot_err = eul_dot.z() - T(target.psi_dot)
        };
    }

    template <typename T>
    TrimResidualVector_T<T> unpack_trim_residual_T(const TrimResidual<T>& residual) {
        TrimResidualVector_T<T> out;
        out << residual.vx_dot,
               residual.vy_dot,
               residual.vz_dot,
               residual.p_dot,
               residual.q_dot,
               residual.r_dot,
               residual.phi_dot,
               residual.theta_dot,
               residual.beta_err,
               residual.phi_err,
               residual.theta_err,
               residual.vx_err,
               residual.vz_err,
               residual.psi_dot_err;
        return out;
    }

    template <typename T>
    TrimResidualVector_T<T> compute_trim_residual_vector_T(const TrimVariablesVector_T<T>& z, const TrimModel& model, const TrimTarget& target, const TrimConditions& conditions, bool use_physical_controls) {
        const dynamics::State_T<T> x = pack_trim_state_T<T>(z);

        actuators::ActuatorInputs_T<T> u;
        if (use_physical_controls) u = pack_trim_actuator_inputs_T<T>(z);
        else u = pack_trim_actuator_inputs_T<T>(z, model.actuator_limits);

        const TrimResidual<T> residual = compute_trim_residual<T>(x, u, model, target, conditions);
        return unpack_trim_residual_T(residual);
    }
}
