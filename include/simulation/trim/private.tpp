#pragma once
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/propulsion/public.hpp"
#include "simulation/trim/public.hpp"
#include "simulation/util/public.hpp"

namespace trim {

    template <typename T>
    TrimResidual<T> compute_trim_residual(const dynamics::State_T<T>& x, const actuators::ActuatorInputs_T<T>& u, const TrimModel& model, const TrimTarget& target, const operating::OperatingConditions& conditions) {
        const dynamics::StateDot_T<T> trim_state_dot = compute_trim_state_dot_T<T>(x, u, model, conditions);
        const dynamics::Twist_T<T> twist = build_twist_from_trim_state_T(x);
        const aerodynamics::AerodynamicState_T<T> ads = aerodynamics::compute_aerodynamic_state_T<T>(twist, conditions.windB);
        const constants::Vector3_T<T> eul_dot = dynamics::wB_BI_to_eul_dot_T<T>(twist.w, x.theta, x.phi);

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
    TrimResidualVector_T<T> compute_trim_residual_vector_T(const TrimVariablesVector_T<T>& z, const TrimModel& model, const TrimTarget& target, const operating::OperatingConditions& conditions, bool use_physical_controls) {
        const dynamics::State_T<T> x = pack_trim_state_T<T>(z);

        actuators::ActuatorInputs_T<T> u;
        if (use_physical_controls) u = pack_trim_actuator_inputs_T<T>(z);
        else u = pack_trim_actuator_inputs_T<T>(z, model.actuator_limits);

        const TrimResidual<T> residual = compute_trim_residual<T>(x, u, model, target, conditions);
        return unpack_trim_residual_T(residual);
    }
}
