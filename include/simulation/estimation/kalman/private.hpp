#pragma once
#include <Eigen/Dense>
#include "simulation/dynamics/public.hpp"

namespace estimation {

    struct KalmanState {
        dynamics::StateVector_T<double> z;  // state estimate
        Eigen::MatrixXd P;  // state estimate error covariance matrix
    };

    struct KalmanPolicyParameters {
        Eigen::MatrixXd P0; // initial state estimate error covariance matrix
        Eigen::MatrixXd Q;  // measurement noise covariance matrix
        Eigen::MatrixXd R;  // process noise covariance matrix
    };

    dynamics::RigidBodyState pack_state_estimate(const dynamics::RigidBodyState& yN_t, const dynamics::StateVector_T<double>& zN_t);

}