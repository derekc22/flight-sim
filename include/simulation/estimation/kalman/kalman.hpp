#pragma once
#include <Eigen/Dense>
#include "simulation/estimation/shared.hpp"

namespace estimation {

    struct KalmanState {
        Eigen::VectorXd x;  // state estimate
        Eigen::MatrixXd P;  // state estimate error covariance matrix
    };

    struct KalmanFilterParameters : KalmanFilterEstimatorParameters {
        Eigen::MatrixXd C;   // measurement matrix
        Eigen::MatrixXd P0;  // initial state estimate error covariance matrix
        Eigen::MatrixXd Q0;  // measurement noise covariance matrix
        Eigen::MatrixXd R0;  // process noise covariance matrix
    };

    struct KalmanFilter {
        KalmanFilterParameters params;

        KalmanFilter(const KalmanFilterParameters& params);
        KalmanState predict(const KalmanState& prev, const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::VectorXd& ut_1);
        KalmanState correct(const KalmanState& pred, const Eigen::VectorXd& zt);
    };
}
