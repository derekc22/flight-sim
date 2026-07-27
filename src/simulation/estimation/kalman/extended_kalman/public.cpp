#include <tuple>
#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/autodiff/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/operating/public.hpp"
#include "simulation/estimation/kalman/extended_kalman/public.hpp"

namespace estimation {

    ExtendedKalmanPolicy::ExtendedKalmanPolicy(const ExtendedKalmanPolicyParameters& params) : params(params) {}

    std::tuple<KalmanState, linearization::OutputJacobian> ExtendedKalmanPolicy::predict(const ExtendedKalmanPolicyInput& input, double dt) {
        KalmanState prev = state.value();

        dynamics::State_T<double> zt_1 = dynamics::pack_state_T(prev.zt);
        actuators::ActuatorInputs_T<double> ut_1 = actuators::pack_actuator_inputs_T(input.ut_1);
        operating::OperatingPoint_T<double> operating_point{ .state = zt_1, .input = ut_1 };

        // A @ zt_1 + B @ ut_1 -> f(zt_1, ut_1)
        dynamics::StateDot_T<double> zt_1_dot = autodiff::compute_state_dot_T(operating_point, input.model, input.conditions, dt);
        dynamics::StateVector_T<double> zt_bar = prev.zt + dynamics::unpack_state_dot_T(zt_1_dot) * dt;

        // A -> Ft
        linearization::LocalLinearization lin_sol = linearization::linearize_operating_point(input.model, operating_point, input.conditions);
        linearization::StateJacobian Ft = linearization::discretize_euler(lin_sol, dt).A;

        Eigen::MatrixXd Pt_bar = Ft * prev.Pt * Ft.transpose() + params.R;

        return { { .zt = zt_bar, .Pt = Pt_bar }, lin_sol.C };
    }

    KalmanState ExtendedKalmanPolicy::correct(const ExtendedKalmanPolicyInput& input, const linearization::OutputJacobian& C) {
        KalmanState pred = state.value();

        // C = I -> Ht = I
        linearization::OutputJacobian Ht = C;

        Eigen::MatrixXd Kt = pred.Pt * Ht.transpose() * (Ht * pred.Pt * Ht.transpose() + params.Q).inverse(); // Kalman gain

        // C @ zt_bar = I @ zt_bar -> h(zt_bar) = zt_bar
        dynamics::StateVector_T<double> Lt = input.yt - pred.zt; // Innovation

        dynamics::StateVector_T<double> zt = pred.zt + Kt * Lt;

        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(pred.Pt.rows(), pred.Pt.cols());

        Eigen::MatrixXd Pt = (I - Kt * Ht) * pred.Pt * (I - Kt * Ht).transpose() + Kt * params.Q * Kt.transpose();

        return { .zt = zt, .Pt = Pt };
    }

    KalmanState ExtendedKalmanPolicy::step(const ExtendedKalmanPolicyInput& input, double dt) {
        if (!state.has_value()) {
            state = KalmanState{ .zt = input.yt, .Pt = params.P0 };
            return state.value();
        }

        auto [temporary, C] = predict(input, dt);
        state = temporary;
        state = correct(input, C);

        return state.value();
    }

}
