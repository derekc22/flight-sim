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
#include "simulation/autodiff/public.hpp"
#include "simulation/operating/public.hpp"

namespace trim {

    template <typename T>
    TrimResidual_T<T> pack_trim_residual_T(const TrimResidualVector_T<T>& residual) {
        return {
            .vx_dot = residual(0),
            .vy_dot = residual(1),
            .vz_dot = residual(2),
            .p_dot = residual(3),
            .q_dot = residual(4),
            .r_dot = residual(5),
            .phi_dot = residual(6),
            .theta_dot = residual(7),
            .beta_err = residual(8),
            .phi_err = residual(9),
            .theta_err = residual(10),
            .vx_err = residual(11),
            .vz_err = residual(12),
            .psi_dot_err = residual(13)
        };
    }

    template <typename T>
    TrimResidualVector_T<T> compute_trim_residual_vector_T(const operating::StateInputVector_T<T>& xu, autodiff::AutoDiffModel& model, const TrimTarget& target, const operating::OperatingConditions& conditions) {
        const operating::OperatingPoint_T<T> operating_point = operating::pack_state_input_T(xu);
        const dynamics::State_T<T> x = operating_point.state;

        const dynamics::StateDot_T<T> x_dot = autodiff::compute_state_dot_T<T>(operating_point, model, conditions, constants::dt);
        const dynamics::Twist_T<T> twist = dynamics::build_twist_from_state_T(x);
        const aerodynamics::AerodynamicState_T<T> aero = aerodynamics::compute_aerodynamic_state_T<T>(twist, conditions.windB);
        const constants::Vector3_T<T> eul_dot = dynamics::wB_BI_to_eul_dot_T<T>(twist.w, x.theta, x.phi);

        TrimResidualVector_T<T> residual;
        residual << x_dot.vx_dot,
            x_dot.vy_dot,
            x_dot.vz_dot,
            x_dot.p_dot,
            x_dot.q_dot,
            x_dot.r_dot,
            x_dot.phi_dot,
            x_dot.theta_dot,
            aero.beta - T(target.beta),
            x.phi - T(target.phi),
            x.theta - T(target.theta),
            x.vx - T(target.vx),
            x.vz - T(target.vz),
            eul_dot.z() - T(target.psi_dot);
        return residual;
    }
}
