#include <tuple>
#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/autodiff/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/operating/public.hpp"
#include "simulation/estimation/kalman/extended_kalman/public.hpp"
#include "simulation/estimation/kalman/private.hpp"

namespace estimation {

    ExtendedKalmanPolicy::ExtendedKalmanPolicy(const ExtendedKalmanPolicyParameters& params) : params(params) {}

    std::tuple<KalmanState, linearization::OutputJacobian> ExtendedKalmanPolicy::predict(const ExtendedKalmanPolicyInput& input) {
        KalmanState prev = state.value();

        dynamics::State_T<double> xt = dynamics::pack_state_T(prev.zt_vec);
        actuators::ActuatorInputs_T<double> u = actuators::pack_actuator_inputs_T<double>(input.ut_1_vec);
        autodiff::AutoDiffModel model = autodiff::build_autodiff_model(input.aircraft);

        // A @ zt_bar + B @ ut_1 -> f(zt_bar, ut_1)
        dynamics::StateDot_T<double> xt_dot = autodiff::compute_state_dot_T(xt, u, model, input.conditions);
        dynamics::StateVector_T<double> zt_vec_bar = prev.zt_vec + dynamics::unpack_state_dot_T(xt_dot) * constants::dt;

        // A -> Ft
        operating::OperatingPoint operating_point{ .state = xt, .input = u };
        linearization::LocalLinearization lin_sol = linearization::linearize_operating_point(input.aircraft, operating_point, input.conditions);
        linearization::StateJacobian Ft = linearization::discretize_euler(lin_sol).A;

        Eigen::MatrixXd Pt_bar = Ft * prev.P * Ft.transpose() + params.R;

        return { { .zt_vec = zt_vec_bar, .P = Pt_bar }, lin_sol.C };
    }

    KalmanState ExtendedKalmanPolicy::correct(const ExtendedKalmanPolicyInput& input, const linearization::OutputJacobian& C) {
        KalmanState pred = state.value();

        // C = I -> Ht = I
        linearization::OutputJacobian Ht = C;

        Eigen::MatrixXd Kt = pred.P * Ht.transpose() * (Ht * pred.P * Ht.transpose() + params.Q).inverse(); // Kalman gain

        // C @ zt_bar = I @ zt_bar -> h(zt_bar) = zt_bar
        dynamics::StateVector_T<double> Lt = input.yt_vec - pred.zt_vec; // Innovation

        dynamics::StateVector_T<double> zt_vec = pred.zt_vec + Kt * Lt;

        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(pred.P.rows(), pred.P.cols());

        Eigen::MatrixXd Pt = (I - Kt * Ht) * pred.P * (I - Kt * Ht).transpose() + Kt * params.Q * Kt.transpose();

        return { .zt_vec = zt_vec, .P = Pt };
    }

    KalmanState ExtendedKalmanPolicy::step(const ExtendedKalmanPolicyInput& input) {
        if (!state.has_value()) {
            state = KalmanState{ .zt_vec = input.yt_vec, .P = params.P0 };
            return state.value();
        }

        auto [temp, C] = predict(input);
        state = temp;
        state = correct(input, C);

        return state.value();
    }

}
