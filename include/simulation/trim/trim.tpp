#pragma once

namespace trim {
    template <typename T>
    dynamics::Twist_T<T> build_twist_from_trim_state_T(const TrimState<T>& x) {
        dynamics::Twist_T<T> twist;
        twist.v << x.vx, x.vy, x.vz;
        twist.w << x.p, x.q, x.r;
        return twist;
    }

    template <typename T>
    aerodynamics::SurfaceActuatorInputs_T<T> build_surface_actuator_inputs_from_trim_T(const TrimActuatorInputs<T>& u, const TrimFixedActuatorInputs& fixed_controls) {
        return {
            .elevator_cmd = u.elevator_cmd,
            .aileron_cmd = u.aileron_cmd,
            .rudder_cmd = u.rudder_cmd,
            .flap_cmd = T(fixed_controls.flap),
            .spoiler_cmd = T(fixed_controls.spoiler),
        };
    }

    template <typename T>
    propulsion::PropulsorActuatorInputs_T<T> build_propulsor_actuator_inputs_from_trim_T(const TrimActuatorInputs<T>& u) {
        return {
            .front_propulsor_cmd = u.front_propulsor_cmd,
            .left_propulsor_cmd = u.left_propulsor_cmd,
            .right_propulsor_cmd = u.right_propulsor_cmd
        };
    }

    template <typename T>
    T get_control_from_solver_space_T(const T& u_solver, const actuators::Actuator& actuator) {
        const double mid = 0.5 * (actuator.limit_max + actuator.limit_min);
        const double half_range = 0.5 * (actuator.limit_max - actuator.limit_min);

        if (half_range <= 0.0) {
            return T(mid);
        }

        return T(mid) + T(half_range) * u_solver / util::sqrt(T(1) + u_solver * u_solver);
    }

    template <typename T>
    TrimActuatorInputs<T> unpack_trim_solver_input_T(const TrimVariableVector_T<T>& z, const actuators::SurfaceActuators& surface_actuators, const actuators::PropulsorActuators& propulsor_actuators) {
        return {
            .elevator_cmd = get_control_from_solver_space_T<T>(z(8), surface_actuators.elevator),
            .aileron_cmd = get_control_from_solver_space_T<T>(z(9), surface_actuators.aileron),
            .rudder_cmd = get_control_from_solver_space_T<T>(z(10), surface_actuators.rudder),

            .front_propulsor_cmd = get_control_from_solver_space_T<T>(z(11), propulsor_actuators.front_propulsor),
            .left_propulsor_cmd = get_control_from_solver_space_T<T>(z(12), propulsor_actuators.left_propulsor),
            .right_propulsor_cmd = get_control_from_solver_space_T<T>(z(13), propulsor_actuators.right_propulsor)
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
    struct TrimNetWrench_T {
        constants::Vector3_T<T> F = constants::Zero3_T<T>;
        constants::Vector3_T<T> M = constants::Zero3_T<T>;
    };

    template <typename T>
    TrimNetWrench_T<T> compute_trim_net_wrench_T(const TrimState<T>& x, const dynamics::Twist_T<T>& twist, const TrimActuatorInputs<T>& u, const TrimModel& model, const TrimConditions& conditions) {
        const aerodynamics::SurfaceActuatorInputs_T<T> surface_actuator_inputs = build_surface_actuator_inputs_from_trim_T(u, model.fixed_controls);
        const aerodynamics::AerodynamicWrench_T<T> aero_wrench = aerodynamics::step_aero_forces_moments_T<T>(
            model.aerodynamic,
            model.structural,
            twist,
            conditions.static_atmospheric_state,
            surface_actuator_inputs,
            conditions.windB
        );

        const propulsion::PropulsorActuatorInputs_T<T> propulsor_actuator_inputs = build_propulsor_actuator_inputs_from_trim_T(u);
        const propulsion::PropulsiveWrench_T<T> prop_wrench = propulsion::step_propulsive_forces_moments_T<T>(
            model.propulsor_actuators,
            twist,
            conditions.static_atmospheric_state,
            propulsor_actuator_inputs,
            propulsion::PropulsorOmegaDot_T<T>{}
        );

        return {
            .F = aero_wrench.F + prop_wrench.F + T(model.structural.Mass.data) * gB_T(x.phi, x.theta),
            .M = aero_wrench.M + prop_wrench.M,
        };
    }

    template <typename T>
    TrimStateDot<T> compute_trim_state_dot_T(const TrimState<T>& x, const TrimActuatorInputs<T>& u, const TrimModel& model, const TrimConditions& conditions) {
        const dynamics::Twist_T<T> twist = build_twist_from_trim_state_T(x);
        const TrimNetWrench_T<T> net_wrench = compute_trim_net_wrench_T<T>(x, twist, u, model, conditions);
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
    TrimResidual<T> compute_trim_residual(const TrimState<T>& x, const TrimActuatorInputs<T>& u, const TrimModel& model, const TrimTarget& target, const TrimConditions& conditions) {
        const TrimStateDot<T> trim_state_dot = compute_trim_state_dot_T<T>(x, u, model, conditions);
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
    TrimVariableVector_T<T> pack_trim_variables_T(const TrimState<T>& x, const TrimActuatorInputs<T>& u) {
        TrimVariableVector_T<T> out;
        out << x.vx, x.vy, x.vz,
               x.p, x.q, x.r,
               x.phi, x.theta,
               u.elevator_cmd, u.aileron_cmd, u.rudder_cmd,
               u.front_propulsor_cmd, u.left_propulsor_cmd, u.right_propulsor_cmd;
        return out;
    }

    template <typename T>
    TrimActuatorInputsVector_T<T> pack_trim_control_surface_inputs_T(const TrimActuatorInputs<T>& u) {
        TrimActuatorInputsVector_T<T> out;
        out << u.elevator_cmd, u.aileron_cmd, u.rudder_cmd,
               u.front_propulsor_cmd, u.left_propulsor_cmd, u.right_propulsor_cmd;
        return out;
    }

    template <typename T>
    TrimStateVector_T<T> pack_trim_state_T(const TrimState<T>& x) {
        TrimStateVector_T<T> out;
        out << x.vx, x.vy, x.vz,
               x.p, x.q, x.r,
               x.phi, x.theta;
        return out;
    }

    template <typename T>
    TrimStateDotVector_T<T> pack_trim_state_dot_T(const TrimStateDot<T>& x_dot) {
        TrimStateDotVector_T<T> out;
        out << x_dot.vx_dot, x_dot.vy_dot, x_dot.vz_dot,
               x_dot.p_dot, x_dot.q_dot, x_dot.r_dot,
               x_dot.phi_dot, x_dot.theta_dot;
        return out;
    }

    template <typename T>
    TrimState<T> unpack_trim_state_T(const TrimVariableVector_T<T>& z) {
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
    TrimActuatorInputs<T> unpack_trim_input_T(const TrimVariableVector_T<T>& z) {
        return {
            .elevator_cmd = z(8),
            .aileron_cmd = z(9),
            .rudder_cmd = z(10),

            .front_propulsor_cmd = z(11),
            .left_propulsor_cmd = z(12),
            .right_propulsor_cmd = z(13),
        };
    }

    template <typename T>
    TrimResidualVector_T<T> pack_trim_residual_T(const TrimResidual<T>& residual) {
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
    TrimResidualVector_T<T> compute_trim_residual_vector_T(const TrimVariableVector_T<T>& z, const TrimModel& model, const TrimTarget& target, const TrimConditions& conditions, bool use_physical_controls) {
        const TrimState<T> x = unpack_trim_state_T<T>(z);

        TrimActuatorInputs<T> u;
        if (use_physical_controls) u = unpack_trim_input_T<T>(z);
        else u = unpack_trim_solver_input_T<T>(z, model.surface_actuators, model.propulsor_actuators);

        const TrimResidual<T> residual = compute_trim_residual<T>(x, u, model, target, conditions);
        return pack_trim_residual_T(residual);
    }
}
