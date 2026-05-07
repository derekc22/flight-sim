#include "simulation/estimation/kalman/kalman.hpp"

namespace estimation {

    KalmanFilter::KalmanFilter(const KalmanFilterParameters& params) : params(params) {}

    KalmanState KalmanFilter::predict(const KalmanState& prev, const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::VectorXd& ut_1) {
        Eigen::VectorXd xt_bar = A * prev.x + B * ut_1;

        Eigen::MatrixXd Pt_bar = A * prev.P * A.transpose() + params.R0;

        return { .x = xt_bar, .P = Pt_bar };
    }

    KalmanState KalmanFilter::correct(const KalmanState& pred, const Eigen::VectorXd& zt) {
        Eigen::MatrixXd Kt = pred.P * params.C.transpose() * (params.C * pred.P * params.C.transpose() + params.Q0).inverse(); // Kalman gain

        Eigen::VectorXd yt = zt - params.C * pred.x; // Innovation

        Eigen::VectorXd xt = pred.x + Kt * yt;

        Eigen::MatrixXd Inxn = Eigen::MatrixXd::Identity(pred.P.rows(), pred.P.cols());

        Eigen::MatrixXd Pt = (Inxn - Kt * params.C) * pred.P * (Inxn - Kt * params.C).transpose() + Kt * params.Q0 * Kt.transpose();

        return { .x = xt, .P = Pt };
    }
}
