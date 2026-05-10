#pragma once
#include "simulation/actuators/surface/shared.hpp"
#include "simulation/atmospheric/shared.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/structural/structural.hpp"
#include "simulation/util/util.hpp"

namespace aerodynamics {

    template <typename T>
    SurfaceKinematics_T<T> compute_surface_kinematics_T(const Surface& s, const structural::StructuralProperties& structural_properties, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& static_atm_state, const atmospheric::Wind& windB) {
        SurfaceKinematics_T<T> out;
        out.rB_ac = s.p_ac.template cast<T>() - structural_properties.CG.data.template cast<T>();
        out.vB_rel = (twist.v - windB.data.template cast<T>()) + twist.w.cross(out.rB_ac);
        out.V = util::vector_norm(out.vB_rel);

        if (out.V < T(constants::eps)) {
            return out;
        }

        const constants::Vector3_T<T> n_B = s.n.template cast<T>();
        const constants::Vector3_T<T> n_hat = util::norm(n_B);
        const T arg = util::clamp_to_1(out.vB_rel.dot(n_hat) / out.V);

        out.alpha = T(s.i) - util::asin(arg);
        out.qbar = T(0.5 * static_atm_state.rho.data) * out.V * out.V;
        out.p_hat = twist.w.x() * T(s.span / 2.0) / out.V;
        out.q_hat = twist.w.y() * T(s.chord / 2.0) / out.V;
        out.r_hat = twist.w.z() * T(s.span / 2.0) / out.V;
        return out;
    }

    template <typename T>
    SurfaceCoefficients_T<T> compute_surface_coefficients_T(const Surface& s, const SurfaceKinematics_T<T>& sk, const actuators::SurfaceActuatorInputs_T<T>& u) {
        const double CLalpha = 2.0 * constants::pi * (s.AR / (2.0 + s.AR));

        SurfaceCoefficients_T<T> out;
        out.CL = T(s.CL0) + T(CLalpha) * sk.alpha;
        out.CM = T(s.CM0) + T(s.CMa) * sk.alpha;

        const T elevator_cmd_abs = util::smooth_abs(u.elevator_cmd);
        const T aileron_cmd_abs = util::smooth_abs(u.aileron_cmd);
        const T rudder_cmd_abs = util::smooth_abs(u.rudder_cmd);
        const T flap_cmd_abs = util::smooth_abs(u.flap_cmd);
        const T spoiler_cmd_abs = util::smooth_abs(u.spoiler_cmd);

        T dCD_extra = T(0);

        out.CL += T(s.dyn.CL_phat) * sk.p_hat + T(s.dyn.CL_qhat) * sk.q_hat + T(s.dyn.CL_rhat) * sk.r_hat;
        out.CM += T(s.dyn.CM_phat) * sk.p_hat + T(s.dyn.CM_qhat) * sk.q_hat + T(s.dyn.CM_rhat) * sk.r_hat;
        dCD_extra += T(s.dyn.CD_phat) * sk.p_hat + T(s.dyn.CD_qhat) * sk.q_hat + T(s.dyn.CD_rhat) * sk.r_hat;

        out.CL += T(s.ctrl.dCL_de) * u.elevator_cmd + T(s.ctrl.dCL_da) * u.aileron_cmd + T(s.ctrl.dCL_dr) * u.rudder_cmd + T(s.ctrl.dCL_df) * u.flap_cmd + T(s.ctrl.dCL_ds) * u.spoiler_cmd;
        out.CM += T(s.ctrl.dCM_de) * u.elevator_cmd + T(s.ctrl.dCM_da) * u.aileron_cmd + T(s.ctrl.dCM_dr) * u.rudder_cmd + T(s.ctrl.dCM_df) * u.flap_cmd + T(s.ctrl.dCM_ds) * u.spoiler_cmd;
        dCD_extra += T(s.ctrl.dCD_de) * elevator_cmd_abs + T(s.ctrl.dCD_da) * aileron_cmd_abs + T(s.ctrl.dCD_dr) * rudder_cmd_abs + T(s.ctrl.dCD_df) * flap_cmd_abs + T(s.ctrl.dCD_ds) * spoiler_cmd_abs;

        out.CD = T(s.CD0) + T(s.CDa) * (sk.alpha - T(s.a0)) * (sk.alpha - T(s.a0)) + (out.CL * out.CL) / T(constants::pi * s.e * s.AR) + dCD_extra;
        return out;
    }

    template <typename T>
    dynamics::Wrench_T<T> compute_surface_loads_T(const Surface& s, const SurfaceKinematics_T<T>& sk, const SurfaceCoefficients_T<T>& sc) {
        dynamics::Wrench_T<T> out;
        if (sk.V < T(constants::eps)) {
            return out;
        }

        const constants::Vector3_T<T> n_B = s.n.template cast<T>();
        const constants::Vector3_T<T> n_hat = util::norm(n_B);
        const constants::Vector3_T<T> d_hat = -sk.vB_rel / sk.V;
        const constants::Vector3_T<T> lift_axis = n_hat - n_hat.dot(d_hat) * d_hat;
        const constants::Vector3_T<T> l_hat = util::norm(lift_axis);
        const constants::Vector3_T<T> moment_axis = l_hat.cross(d_hat);
        const constants::Vector3_T<T> m_hat = util::norm(moment_axis);

        const T L = sk.qbar * T(s.area) * sc.CL;
        const T D = sk.qbar * T(s.area) * sc.CD;
        const T Mmag = sk.qbar * T(s.area * s.chord) * sc.CM;

        out.F = L * l_hat + D * d_hat;
        out.M = sk.rB_ac.cross(out.F) + Mmag * m_hat;
        return out;
    }

    template <typename T>
    dynamics::Wrench_T<T> step_aero_forces_moments_T(const AerodynamicProperties& aerodynamic_properties, const structural::StructuralProperties& structural_properties, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& static_atm_state, const actuators::SurfaceActuatorInputs_T<T>& u, const atmospheric::Wind& windB) {
        dynamics::Wrench_T<T> total;
        for (const Surface& s : aerodynamic_properties.surfaces) {
            const SurfaceKinematics_T<T> sk = compute_surface_kinematics_T<T>(s, structural_properties, twist, static_atm_state, windB);
            const SurfaceCoefficients_T<T> sc = compute_surface_coefficients_T<T>(s, sk, u);
            const dynamics::Wrench_T<T> loads = compute_surface_loads_T<T>(s, sk, sc);
            total.F += loads.F;
            total.M += loads.M;
        }
        return total;
    }

    template <typename T>
    AerodynamicState_T<T> compute_aerodynamic_state_T(const dynamics::Twist_T<T>& twist, const atmospheric::Wind& windB) {
        AerodynamicState_T<T> out;
        const constants::Vector3_T<T> vB_rel = twist.v - windB.data.template cast<T>();
        out.Vinf = util::vector_norm(vB_rel);

        if (out.Vinf > T(constants::eps)) {
            out.alpha = util::atan2(vB_rel.z(), vB_rel.x());
            out.beta = util::asin(util::clamp_to_1(vB_rel.y() / out.Vinf));
        }

        return out;
    }
}
