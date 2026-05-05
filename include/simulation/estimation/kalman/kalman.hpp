#pragma once
#include <Eigen/Dense>
#include "simulation/estimation/shared.hpp"

namespace estimation {

    struct KalmanState {
        Eigen::VectorXd x;
        Eigen::MatrixXd P;
    };

    struct KalmanFilterParameters : EstimatorParameters {
        Eigen::MatrixXd C;
        Eigen::MatrixXd P0;
        Eigen::MatrixXd Q0;
        Eigen::MatrixXd R0;
    };

    struct KalmanFilter {
        KalmanFilterParameters params;

        KalmanFilter(const KalmanFilterParameters& params);
        KalmanState predict(const KalmanState& prev, const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::VectorXd& u) const;
        KalmanState correct(const KalmanState& pred, const Eigen::VectorXd& z) const;
    };
}
