#pragma once

namespace aerodynamics {
    template <typename T>
    ControlSurfaceInputs_T<T> clamp_control_inputs_T(const ControlSurfaceInputs_T<T>& u, const control::ControlProperties& cp) {
        return ControlSurfaceInputs_T<T>{
            .elevator = global::clamp_symmetric(u.elevator, cp.limits.elevator_max),
            .aileron = global::clamp_symmetric(u.aileron, cp.limits.aileron_max),
            .rudder = global::clamp_symmetric(u.rudder, cp.limits.rudder_max),
            .flap = global::clamp_positive(u.flap, cp.limits.flap_max),
            .spoiler = global::clamp_positive(u.spoiler, cp.limits.spoiler_max),
        };
    }

    template <typename T>
    SurfaceKinematics_T<T> compute_surface_kinematics_T(const Surface& s, const structural::StructuralProperties& structural_properties, const dynamics::Twist_T<T>& twist, const atmospheric::Density& rho, const atmospheric::Wind& windB) {
        SurfaceKinematics_T<T> out;
        out.r_ac_B = s.p_ac.template cast<T>() - structural_properties.CG.data.template cast<T>();
        out.v_rel_B = (twist.v - windB.data.template cast<T>()) + twist.w.cross(out.r_ac_B);
        out.V = global::vector_norm(out.v_rel_B);

        if (out.V < T(global::eps)) {
            return out;
        }

        const Vector3_T<T> n_B = s.n.template cast<T>();
        const Vector3_T<T> n_hat = global::norm(n_B);
        const T arg = global::clamp_to_1(out.v_rel_B.dot(n_hat) / out.V);

        out.alpha = T(s.i) - global::asin(arg);
        out.qbar = T(0.5 * rho.data) * out.V * out.V;
        out.p_hat = twist.w.x() * T(s.span / 2.0) / out.V;
        out.q_hat = twist.w.y() * T(s.chord / 2.0) / out.V;
        out.r_hat = twist.w.z() * T(s.span / 2.0) / out.V;
        return out;
    }

    template <typename T>
    SurfaceCoefficients_T<T> compute_surface_coefficients_T(const Surface& s, const SurfaceKinematics_T<T>& sk, const ControlSurfaceInputs_T<T>& u, const control::ControlProperties& cp, bool clamp_controls) {
        const ControlSurfaceInputs_T<T> u_eval = clamp_controls ? clamp_control_inputs_T(u, cp) : u;
        const double CLalpha = 2.0 * global::pi * (s.AR / (2.0 + s.AR));

        SurfaceCoefficients_T<T> out;
        out.CL = T(s.CL0) + T(CLalpha) * sk.alpha;
        out.CM = T(s.CM0) + T(s.CMa) * sk.alpha;

        T dCD_extra = T(0);

        out.CL += T(s.dyn.CL_phat) * sk.p_hat + T(s.dyn.CL_qhat) * sk.q_hat + T(s.dyn.CL_rhat) * sk.r_hat;
        out.CM += T(s.dyn.CM_phat) * sk.p_hat + T(s.dyn.CM_qhat) * sk.q_hat + T(s.dyn.CM_rhat) * sk.r_hat;
        dCD_extra += T(s.dyn.CD_phat) * sk.p_hat + T(s.dyn.CD_qhat) * sk.q_hat + T(s.dyn.CD_rhat) * sk.r_hat;

        out.CL += T(s.ctrl.dCL_de) * u_eval.elevator + T(s.ctrl.dCL_da) * u_eval.aileron + T(s.ctrl.dCL_dr) * u_eval.rudder + T(s.ctrl.dCL_df) * u_eval.flap + T(s.ctrl.dCL_ds) * u_eval.spoiler;
        out.CM += T(s.ctrl.dCM_de) * u_eval.elevator + T(s.ctrl.dCM_da) * u_eval.aileron + T(s.ctrl.dCM_dr) * u_eval.rudder + T(s.ctrl.dCM_df) * u_eval.flap + T(s.ctrl.dCM_ds) * u_eval.spoiler;
        dCD_extra += T(s.ctrl.dCD_de) * global::abs(u_eval.elevator) + T(s.ctrl.dCD_da) * global::abs(u_eval.aileron) + T(s.ctrl.dCD_dr) * global::abs(u_eval.rudder) + T(s.ctrl.dCD_df) * global::abs(u_eval.flap) + T(s.ctrl.dCD_ds) * global::abs(u_eval.spoiler);

        out.CD = T(s.CD0) + T(s.CDa) * (sk.alpha - T(s.a0)) * (sk.alpha - T(s.a0)) + (out.CL * out.CL) / T(global::pi * s.e * s.AR) + dCD_extra;
        return out;
    }

    template <typename T>
    AerodynamicLoad_T<T> compute_surface_loads_T(const Surface& s, const SurfaceKinematics_T<T>& sk, const SurfaceCoefficients_T<T>& sc) {
        AerodynamicLoad_T<T> out;
        if (sk.V < T(global::eps)) {
            return out;
        }

        const Vector3_T<T> n_B = s.n.template cast<T>();
        const Vector3_T<T> n_hat = global::norm(n_B);
        const Vector3_T<T> d_hat = -sk.v_rel_B / sk.V;
        const Vector3_T<T> lift_axis = n_hat - n_hat.dot(d_hat) * d_hat;
        const Vector3_T<T> l_hat = global::norm(lift_axis);
        const Vector3_T<T> moment_axis = l_hat.cross(d_hat);
        const Vector3_T<T> m_hat = global::norm(moment_axis);

        const T L = sk.qbar * T(s.area) * sc.CL;
        const T D = sk.qbar * T(s.area) * sc.CD;
        const T Mmag = sk.qbar * T(s.area * s.chord) * sc.CM;

        out.F = L * l_hat + D * d_hat;
        out.M = sk.r_ac_B.cross(out.F) + Mmag * m_hat;
        return out;
    }

    template <typename T>
    AerodynamicLoad_T<T> step_aero_forces_moments_T(const AerodynamicProperties& aerodynamic_properties, const structural::StructuralProperties& structural_properties, const dynamics::Twist_T<T>& twist, const atmospheric::Density& rho, const ControlSurfaceInputs_T<T>& u, const control::ControlProperties& cp, const atmospheric::Wind& windB, bool clamp_controls) {
        AerodynamicLoad_T<T> total;
        for (const Surface& s : aerodynamic_properties.surfaces) {
            const SurfaceKinematics_T<T> sk = compute_surface_kinematics_T<T>(s, structural_properties, twist, rho, windB);
            const SurfaceCoefficients_T<T> sc = compute_surface_coefficients_T<T>(s, sk, u, cp, clamp_controls);
            const AerodynamicLoad_T<T> loads = compute_surface_loads_T<T>(s, sk, sc);
            total.F += loads.F;
            total.M += loads.M;
        }
        return total;
    }

    template <typename T>
    AerodynamicState_T<T> compute_aerodynamic_state_T(const dynamics::Twist_T<T>& twist, const atmospheric::Wind& windB) {
        AerodynamicState_T<T> out;
        const Vector3_T<T> vB_rel = twist.v - windB.data.template cast<T>();
        out.Vinf = global::vector_norm(vB_rel);

        if (out.Vinf > T(global::eps)) {
            out.alpha = global::atan2(vB_rel.z(), vB_rel.x());
            out.beta = global::asin(global::clamp_to_1(vB_rel.y() / out.Vinf));
        }

        return out;
    }
}
