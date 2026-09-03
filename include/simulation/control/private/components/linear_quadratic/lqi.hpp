#pragma once
#include <Eigen/Dense>
#include "simulation/control/private/components/linear_quadratic/lqr.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/constants/public/dimensions.hpp"

namespace control {

    inline constexpr std::size_t integrated_state_dim = 3;  // phi, theta, r are the integrated states
    using IntegratedStateVector = constants::MatrixX_T<double, integrated_state_dim, 1>;
    using AugmentedStateVector = constants::MatrixX_T<double, constants::state_dim + integrated_state_dim, 1>;

    struct LinearQuadraticIntegrator : LinearQuadraticRegulator {
        IntegratedStateVector integral = Eigen::Vector3d::Zero();

        LinearQuadraticIntegrator(const LinearQuadraticIntegratorParameters& params);
        VirtualControlOutput_T<double> step(const LinearQuadraticControlInput& input, double dt);

        LinearQuadraticPolicyInput make_linear_quadratic_policy_input(const LinearQuadraticControlInput& input, const IntegratedStateVector& integral_candidate);
        IntegratedStateVector integrate_state_err(const dynamics::StateVector_T<double>& zt, const dynamics::StateVector_T<double>& zt_des, double dt);
    };

}
