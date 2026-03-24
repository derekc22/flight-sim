#pragma once

namespace autopilot {
    inline void _check_trim_variables_size(std::size_t size) {
        if (size != trim_variable_dofs) {
            throw std::invalid_argument("autopilot::_check_trim_variables_size: trim variable vector has incorrect size");
        }
    }

    template <typename T>
    dynamics::Twist_T<T> _trim_twist_T(const TrimState<T>& x) {
        dynamics::Twist_T<T> twist;
        twist.v << x.vx, x.vy, x.vz;
        twist.w << x.p, x.q, x.r;
        return twist;
    }

    template <typename T>
    aerodynamics::ControlSurfaceInputs_T<T> _trim_controls_T(const TrimInput<T>& u, const TrimFixedControls& fixed_controls) {
        return aerodynamics::ControlSurfaceInputs_T<T>{
            .elevator = u.elevator,
            .aileron = u.aileron,
            .rudder = u.rudder,
            .flap = T(fixed_controls.flap),
            .spoiler = T(fixed_controls.spoiler),
        };
    }

    template <typename T>
    dynamics::Vector3_T<T> _gB_T(const T& phi, const T& theta) {
        dynamics::Vector3_T<T> gB;
        gB << -T(global::gravity) * global::sin(theta),
              T(global::gravity) * global::sin(phi) * global::cos(theta),
              T(global::gravity) * global::cos(phi) * global::cos(theta);
        return gB;
    }

    template <typename T>
    TrimResidual<T> evaluate_trim_residual(const TrimState<T>& x, const TrimInput<T>& u, const TrimModelContext& model, const TrimTarget& target, const TrimConditions& conditions) {
        const dynamics::Twist_T<T> twist = _trim_twist_T(x);
        const aerodynamics::ControlSurfaceInputs_T<T> controls = _trim_controls_T(u, model.fixed_controls);
        const aerodynamics::AerodynamicLoad_T<T> aero = aerodynamics::step_aero_forces_moments_T<T>(
            model.aerodynamic,
            model.structural,
            twist,
            conditions.rho,
            controls,
            model.control,
            conditions.windB,
            false
        );
        const aerodynamics::AerodynamicState_T<T> ads = aerodynamics::compute_aerodynamic_state_T<T>(twist, conditions.windB);

        const dynamics::Vector3_T<T> FB_net = aero.F + T(model.structural.Mass.data) * _gB_T(x.phi, x.theta);
        const dynamics::Vector3_T<T> MB_net = aero.M;
        const dynamics::Vector3_T<T> v_dot = dynamics::_ddtB_vB_BI_T<T>(twist.v, twist.w, model.structural.Mass.data, FB_net);
        const dynamics::Vector3_T<T> w_dot = dynamics::_ddtB_wB_BI_T<T>(twist.w, model.structural.J.data, MB_net);
        const dynamics::Vector3_T<T> eul_dot = dynamics::_wB_BI2eul_dot_T<T>(twist.w, x.theta, x.phi);

        return TrimResidual<T> {
            .vx_dot = v_dot.x(),
            .vy_dot = v_dot.y(),
            .vz_dot = v_dot.z(),
            .p_dot = w_dot.x(),
            .q_dot = w_dot.y(),
            .r_dot = w_dot.z(),
            .phi_dot = eul_dot.x(),
            .theta_dot = eul_dot.y(),
            .beta_error = ads.beta - T(target.beta),
            .phi_error = x.phi - T(target.phi),
            .theta_error = x.theta - T(target.theta),
        };
    }

    template <typename T>
    std::vector<T> pack_trim_variables_T(const TrimState<T>& x, const TrimInput<T>& u) {
        return std::vector<T>{
            x.vx, x.vy, x.vz,
            x.p, x.q, x.r,
            x.phi, x.theta,
            u.elevator, u.aileron, u.rudder
        };
    }

    template <typename T>
    TrimState<T> unpack_trim_state_T(const std::vector<T>& z) {
        _check_trim_variables_size(z.size());
        return TrimState<T>{
            .vx = z[0],
            .vy = z[1],
            .vz = z[2],
            .p = z[3],
            .q = z[4],
            .r = z[5],
            .phi = z[6],
            .theta = z[7],
        };
    }

    template <typename T>
    TrimInput<T> unpack_trim_input_T(const std::vector<T>& z) {
        _check_trim_variables_size(z.size());
        return TrimInput<T>{
            .elevator = z[8],
            .aileron = z[9],
            .rudder = z[10],
        };
    }

    template <typename T>
    std::vector<T> pack_trim_residual_T(const TrimResidual<T>& residual) {
        return std::vector<T>{
            residual.vx_dot,
            residual.vy_dot,
            residual.vz_dot,
            residual.p_dot,
            residual.q_dot,
            residual.r_dot,
            residual.phi_dot,
            residual.theta_dot,
            residual.beta_error,
            residual.phi_error,
            residual.theta_error
        };
    }

    template <typename T>
    std::vector<T> evaluate_trim_residual_vector_T(const std::vector<T>& z, const TrimModelContext& model, const TrimTarget& target, const TrimConditions& conditions) {
        const TrimState<T> x = unpack_trim_state_T<T>(z);
        const TrimInput<T> u = unpack_trim_input_T<T>(z);
        const TrimResidual<T> residual = evaluate_trim_residual<T>(x, u, model, target, conditions);
        return pack_trim_residual_T(residual);
    }
}
