#pragma once
#include <Eigen/Dense>
#include "simulation/control/shared/public.hpp"
#include "simulation/control/linear_quadratic/controllers/lqr/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/constants/public.hpp"

namespace control {

    inline constexpr std::size_t integrated_state_dim = 3;  // p, q, r are the integrated states
    using IntegratedStateVector = Eigen::Matrix<double, integrated_state_dim, 1>;
    using AugmentedStateVector = Eigen::Matrix<double, constants::state_dim + integrated_state_dim, 1>;

    struct LinearQuadraticIntegratorParameters : LinearQuadraticRegulatorParameters {
        Eigen::MatrixXd Qi;
    };

    struct LinearQuadraticIntegrator : LinearQuadraticRegulator {
        IntegratedStateVector integral = Eigen::Vector3d::Zero();

        LinearQuadraticIntegrator(const LinearQuadraticIntegratorParameters& params);
        ControlOutput step(const LinearQuadraticControllerInput& input);

        LinearQuadraticPolicyInput make_linear_quadratic_policy_input(const LinearQuadraticControllerInput& input, const IntegratedStateVector& integral_candidate);
        IntegratedStateVector integrate_state_err(const dynamics::StateVector_T<double>& zt, const dynamics::StateVector_T<double>& zt_des);
    };

}
