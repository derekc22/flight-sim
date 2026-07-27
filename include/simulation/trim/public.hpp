#pragma once
#include <cstddef>
#include <Eigen/Dense>
#include <string>
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/propulsion/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/structural/public.hpp"
#include "simulation/operating/public.hpp"
#include "simulation/autodiff/public.hpp"

namespace vehicles { struct Aircraft; } // forward declare

namespace trim {

    template <typename T>
    struct TrimResidual_T {
        T vx_dot = T(0.0);
        T vy_dot = T(0.0);
        T vz_dot = T(0.0);
        T p_dot = T(0.0);
        T q_dot = T(0.0);
        T r_dot = T(0.0);
        T phi_dot = T(0.0);
        T theta_dot = T(0.0);
        T beta_err = T(0.0);
        T phi_err = T(0.0);
        T theta_err = T(0.0);
        T vx_err = T(0.0);
        T vz_err = T(0.0);
        T psi_dot_err = T(0.0);
    };

    struct TrimSolution {
        operating::OperatingPoint_T<double> operating_point;
        operating::OperatingConditions conditions;
        dynamics::Wrench_T<double> wrench{};
        TrimResidual_T<double> residual;
        TrimResidual_T<double> weighted_residual;
        bool attempted = false;
        bool converged = false;
        std::size_t iterations = 0;
        double weighted_residual_norm_2 = 0.0;
        double weighted_residual_norm_inf = 0.0;
    };

    TrimSolution inspect_trim(vehicles::Aircraft& aircraft, autodiff::AutoDiffModel& model, const atmospheric::Wind& wind);

    std::string print_trim_solution(const TrimSolution& trim_sol);

    dynamics::RigidBodyState update_state_from_trim(const dynamics::RigidBodyState& Xt, const dynamics::State_T<double>& trim_state);

    /** @deprecated */
    // void update_actuators_lag_from_trim(actuators::SurfaceActuators& surface_actuators, actuators::PropulsorActuators& propulsor_actuators, const TrimSolution& trim_sol);

    /** @deprecated */
    // actuators::ActuatorInputs_T<double> set_actuator_inputs_from_trim(actuators::ActuatorInputs_T<double> actuator_inputs, const actuators::ActuatorInputs_T<double>& trim_inputs);

}
