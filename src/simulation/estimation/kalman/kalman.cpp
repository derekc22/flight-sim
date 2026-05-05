#include "simulation/estimation/kalman/kalman.hpp"

namespace estimation {

    KalmanFilter::KalmanFilter(const KalmanFilterParameters& params) : params(params) {}

    KalmanState KalmanFilter::predict(const KalmanState& prev, const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::VectorXd& u) const {
        return {
            .x = A * prev.x + B * u,
            .P = A * prev.P * A.transpose() + params.R0
        };
    }

    KalmanState KalmanFilter::correct(const KalmanState& pred, const Eigen::VectorXd& z) const {
        const Eigen::MatrixXd K = pred.P * params.C.transpose() * (params.C * pred.P * params.C.transpose() + params.Q0).inverse();
        const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(pred.P.rows(), pred.P.cols());

        return {
            .x = pred.x + K * (z - params.C * pred.x),
            .P = (I - K * params.C) * pred.P * (I - K * params.C).transpose() + K * params.Q0 * K.transpose()
        };
    }
}
