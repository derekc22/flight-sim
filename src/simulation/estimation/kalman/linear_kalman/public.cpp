#include "simulation/dynamics/public.hpp"
#include "simulation/estimation/kalman/linear_kalman/public.hpp"
#include "simulation/estimation/kalman/private.hpp"

namespace estimation {

    LinearKalmanPolicy::LinearKalmanPolicy(const LinearKalmanPolicyParameters& params) : params(params) {}

    KalmanState LinearKalmanPolicy::predict(const LinearKalmanPolicyInput& input) {
        KalmanState prev = state.value();

        dynamics::StateVector_T<double> zt_vec_bar = input.A * prev.zt_vec + input.B * input.ut_1_vec;

        Eigen::MatrixXd Pt_bar = input.A * prev.P * input.A.transpose() + params.R;

        return { .zt_vec = zt_vec_bar, .P = Pt_bar };
    }

    KalmanState LinearKalmanPolicy::correct(const LinearKalmanPolicyInput& input) {
        KalmanState pred = state.value();

        Eigen::MatrixXd Kt = pred.P * input.C.transpose() * (input.C * pred.P * input.C.transpose() + params.Q).inverse(); // Kalman gain

        dynamics::StateVector_T<double> Lt = input.yt_vec - input.C * pred.zt_vec; // Innovation

        dynamics::StateVector_T<double> zt_vec = pred.zt_vec + Kt * Lt;

        Eigen::MatrixXd Inxn = Eigen::MatrixXd::Identity(pred.P.rows(), pred.P.cols());

        Eigen::MatrixXd Pt = (Inxn - Kt * input.C) * pred.P * (Inxn - Kt * input.C).transpose() + Kt * params.Q * Kt.transpose();

        return { .zt_vec = zt_vec, .P = Pt };
    }

    KalmanState LinearKalmanPolicy::step(const LinearKalmanPolicyInput& input) {
        if (!state.has_value()) {
            state = KalmanState{ .zt_vec = input.yt_vec, .P = params.P0 };
            return state.value();
        }

        state = predict(input);
        state = correct(input);

        return state.value();
    }

}
