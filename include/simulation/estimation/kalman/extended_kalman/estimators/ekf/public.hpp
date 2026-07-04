#pragma once
#include <Eigen/Dense>
#include <optional>
#include "simulation/dynamics/public.hpp"
#include "simulation/estimation/shared/public.hpp"
#include "simulation/estimation/kalman/extended_kalman/public.hpp"

namespace estimation {

    struct ExtendedKalmanFilterParameters : ExtendedKalmanPolicyParameters {};

    struct ExtendedKalmanFilter {
        ExtendedKalmanFilterParameters params;
        std::optional<KalmanState> state;
        ExtendedKalmanPolicy policy;

        ExtendedKalmanFilter(const ExtendedKalmanFilterParameters& params);
        EstimationOutput step(const ExtendedKalmanEstimatorInput& input, double dt);

        ExtendedKalmanPolicyInput make_extended_kalman_policy_input(const ExtendedKalmanEstimatorInput& input);
        dynamics::RigidBodyState make_ekf_state_estimate(const ExtendedKalmanEstimatorInput& input, const dynamics::StateVector_T<double>& zt);

    };

}
