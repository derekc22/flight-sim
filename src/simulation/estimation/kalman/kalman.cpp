#include "simulation/estimation/kalman/kalman.hpp"

namespace estimation {

    KalmanFilter::KalmanFilter(const KalmanFilterParameters& params) : params(params) {}

    KalmanState KalmanFilter::predict(const KalmanState& prev, const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const types::ActuatorInputsVector_T<double>& ut_1) {
        types::StateVector_T<double> zN_t_bar = A * prev.z + B * ut_1;

        Eigen::MatrixXd Pt_bar = A * prev.P * A.transpose() + params.R0;

        return { .z = zN_t_bar, .P = Pt_bar };
    }

    KalmanState KalmanFilter::correct(const KalmanState& pred, const types::StateVector_T<double>& yN_t) {
        Eigen::MatrixXd Kt = pred.P * params.C.transpose() * (params.C * pred.P * params.C.transpose() + params.Q0).inverse(); // Kalman gain

        types::StateVector_T<double> Lt = yN_t - params.C * pred.z; // Innovation

        types::StateVector_T<double> zN_t = pred.z + Kt * Lt;

        Eigen::MatrixXd Inxn = Eigen::MatrixXd::Identity(pred.P.rows(), pred.P.cols());

        Eigen::MatrixXd Pt = (Inxn - Kt * params.C) * pred.P * (Inxn - Kt * params.C).transpose() + Kt * params.Q0 * Kt.transpose();

        return { .z = zN_t, .P = Pt };
    }
}
