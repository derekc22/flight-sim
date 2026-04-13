#pragma once

namespace autopilot {
    template <typename T>
    dynamics::Twist_T<T> _build_twist_from_trim_state_T(const TrimState<T>& x) {
        dynamics::Twist_T<T> twist;
        twist.v << x.vx, x.vy, x.vz;
        twist.w << x.p, x.q, x.r;
        return twist;
    }

    template <typename T>
    aerodynamics::ControlSurfaceInputs_T<T> _build_control_surface_inputs_from_trim_T(const TrimControlSurfaceInputs<T>& u, const TrimFixedControls& fixed_controls) {
        return aerodynamics::ControlSurfaceInputs_T<T>{
            .elevator = u.elevator,
            .aileron = u.aileron,
            .rudder = u.rudder,
            .flaps = T(fixed_controls.flaps),
            .spoilers = T(fixed_controls.spoilers),
        };
    }

    template <typename T>
    T _get_control_from_solver_space_T(const T& u_solver, const actuators::Actuator& actuator) {
        const double mid = 0.5 * (actuator.limit_max + actuator.limit_min);
        const double half_range = 0.5 * (actuator.limit_max - actuator.limit_min);

        if (half_range <= 0.0) {
            return T(mid);
        }

        return T(mid) + T(half_range) * u_solver / util::sqrt(T(1) + u_solver * u_solver);
    }

    template <typename T>
    TrimControlSurfaceInputs<T> _unpack_trim_solver_input_T(const TrimVariableVector_T<T>& z, const actuators::Actuators& actuators) {
        return {
            .elevator = _get_control_from_solver_space_T<T>(z(8), actuators.elevator),
            .aileron = _get_control_from_solver_space_T<T>(z(9), actuators.aileron),
            .rudder = _get_control_from_solver_space_T<T>(z(10), actuators.rudder),
        };
    }

    template <typename T>
    dynamics::Vector3_T<T> _gB_T(const T& phi, const T& theta) {
        dynamics::Vector3_T<T> gB;
        gB << -T(constants::g_earth) * util::sin(theta),
               T(constants::g_earth) * util::sin(phi) * util::cos(theta),
               T(constants::g_earth) * util::cos(phi) * util::cos(theta);
        return gB;
    }

    template <typename T>
    TrimStateDot<T> compute_trim_state_dot_T(const TrimState<T>& x, const TrimControlSurfaceInputs<T>& u, const TrimModel& model, const TrimConditions& conditions) {
        const dynamics::Twist_T<T> twist = _build_twist_from_trim_state_T(x);
        const aerodynamics::ControlSurfaceInputs_T<T> controls = _build_control_surface_inputs_from_trim_T(u, model.fixed_controls);
        const aerodynamics::AerodynamicLoad_T<T> aero = aerodynamics::step_aero_forces_moments_T<T>(
            model.aerodynamic,
            model.structural,
            twist,
            conditions.static_atmospheric_state,
            controls,
            // model.actuator.actuators,
            conditions.windB
            // false
        );

        const dynamics::Vector3_T<T> FB_net = aero.F + T(model.structural.Mass.data) * _gB_T(x.phi, x.theta);
        const dynamics::Vector3_T<T> MB_net = aero.M;
        const dynamics::Vector3_T<T> v_dot = dynamics::_ddtB_vB_BI_T<T>(twist.v, twist.w, model.structural.Mass.data, FB_net);
        const dynamics::Vector3_T<T> w_dot = dynamics::_ddtB_wB_BI_T<T>(twist.w, model.structural.J.data, MB_net);
        const dynamics::Vector3_T<T> eul_dot = dynamics::_wB_BI_to_eul_dot_T<T>(twist.w, x.theta, x.phi);

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
    TrimResidual<T> compute_trim_residual(const TrimState<T>& x, const TrimControlSurfaceInputs<T>& u, const TrimModel& model, const TrimTarget& target, const TrimConditions& conditions) {
        const TrimStateDot<T> trim_state_dot = compute_trim_state_dot_T<T>(x, u, model, conditions);
        const aerodynamics::AerodynamicState_T<T> ads = aerodynamics::compute_aerodynamic_state_T<T>(_build_twist_from_trim_state_T(x), conditions.windB);

        return {
            .vx_dot = trim_state_dot.vx_dot,
            .vy_dot = trim_state_dot.vy_dot,
            .vz_dot = trim_state_dot.vz_dot,
            .p_dot = trim_state_dot.p_dot,
            .q_dot = trim_state_dot.q_dot,
            .r_dot = trim_state_dot.r_dot,
            .phi_dot = trim_state_dot.phi_dot,
            .theta_dot = trim_state_dot.theta_dot,
            .beta_error = ads.beta - T(target.beta),
            .phi_error = x.phi - T(target.phi),
            .theta_error = x.theta - T(target.theta),
        };
    }

    template <typename T>
    TrimVariableVector_T<T> pack_trim_variables_T(const TrimState<T>& x, const TrimControlSurfaceInputs<T>& u) {
        TrimVariableVector_T<T> out;
        out << x.vx, x.vy, x.vz,
               x.p, x.q, x.r,
               x.phi, x.theta,
               u.elevator, u.aileron, u.rudder;
        return out;
    }

    template <typename T>
    TrimControlSurfaceInputsVector_T<T> pack_trim_control_surface_inputs_T(const TrimControlSurfaceInputs<T>& u) {
        TrimControlSurfaceInputsVector_T<T> out;
        out << u.elevator, u.aileron, u.rudder;
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
    TrimControlSurfaceInputs<T> unpack_trim_input_T(const TrimVariableVector_T<T>& z) {
        return {
            .elevator = z(8),
            .aileron = z(9),
            .rudder = z(10),
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
               residual.beta_error,
               residual.phi_error,
               residual.theta_error;
        return out;
    }

    template <typename T>
    TrimResidualVector_T<T> compute_trim_residual_vector_T(const TrimVariableVector_T<T>& z, const TrimModel& model, const TrimTarget& target, const TrimConditions& conditions, bool use_physical_controls) {
        const TrimState<T> x = unpack_trim_state_T<T>(z);

        TrimControlSurfaceInputs<T> u;
        if (use_physical_controls) u = unpack_trim_input_T<T>(z);
        else u = _unpack_trim_solver_input_T<T>(z, model.actuator.actuators);

        const TrimResidual<T> residual = compute_trim_residual<T>(x, u, model, target, conditions);
        return pack_trim_residual_T(residual);
    }
}
