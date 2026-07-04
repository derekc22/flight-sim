#pragma once
#include <Eigen/Dense>
#include <optional>
#include "simulation/dynamics/public.hpp"
#include "simulation/estimation/shared/public.hpp"
#include "simulation/estimation/kalman/linear_kalman/public.hpp"

namespace estimation {

    struct LinearKalmanFilterParameters : LinearKalmanPolicyParameters {};

    struct LinearKalmanFilter {
        LinearKalmanFilterParameters params;
        std::optional<KalmanState> state;
        LinearKalmanPolicy policy;

        LinearKalmanFilter(const LinearKalmanFilterParameters& params);
        EstimationOutput step(const LinearKalmanEstimatorInput& input, double dt);

        LinearKalmanPolicyInput make_linear_kalman_policy_input(const LinearKalmanEstimatorInput& input, double dt);
        dynamics::RigidBodyState make_lkf_state_estimate(const LinearKalmanEstimatorInput& input, const dynamics::StateVector_T<double>& zt);

    };

}
