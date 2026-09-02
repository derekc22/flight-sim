#include <Eigen/Dense>
#include <tuple>
#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/autodiff/public/detail/evaluation.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public/data/helpers.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/estimation/public/components/extended_kalman.hpp"
#include "simulation/estimation/public/data/helpers.hpp"
#include "simulation/linearization/public/detail/discretization.hpp"
#include "simulation/linearization/public/detail/jacobian.hpp"
#include "simulation/operating/public/data/types.hpp"

namespace estimation {

    ExtendedKalmanEstimator::ExtendedKalmanEstimator(const ExtendedKalmanFilterParameters& params) : params(params) {}

    dynamics::RigidBodyState ExtendedKalmanEstimator::step(const ExtendedKalmanEstimatorInput& input, double dt) {
        dynamics::StateVector_T<double> yt = dynamics::unpack_state(input.Yt);
        actuators::ActuatorInputsVector_T<double> ut_1 = actuators::unpack_actuator_inputs_T(input.u_actual_t_1);

        if (!state.has_value()) {
            state = KalmanState{ .zt = yt, .Pt = params.P0 };
        } else {
            auto [temporary, C] = predict(input, ut_1, dt);
            state = temporary;
            state = correct(yt, C);
        }

        // EKF predicts full state, so do not add back trim state
        dynamics::RigidBodyState Zt = make_kalman_state_estimate(input.Yt, state.value().zt);

        return Zt;
    }

    std::tuple<KalmanState, linearization::OutputJacobian> ExtendedKalmanEstimator::predict(const ExtendedKalmanEstimatorInput& input, const actuators::ActuatorInputsVector_T<double>& ut_1, double dt) {
        KalmanState prev = state.value();

        dynamics::State_T<double> zt_1 = dynamics::pack_state_T(prev.zt);
        actuators::ActuatorInputs_T<double> u_actual_t_1 = actuators::pack_actuator_inputs_T(ut_1);
        operating::OperatingPoint_T<double> operating_point{ .state = zt_1, .input = u_actual_t_1 };

        // A @ zt_1 + B @ ut_1 -> f(zt_1, ut_1)
        dynamics::StateDot_T<double> zt_1_dot = autodiff::compute_state_dot_T(operating_point, input.model, input.conditions, dt);
        dynamics::StateVector_T<double> zt_bar = prev.zt + dynamics::unpack_state_dot_T(zt_1_dot) * dt;

        // A -> Ft
        linearization::LocalLinearization lin_sol = linearization::linearize_operating_point(input.model, operating_point, input.conditions);
        linearization::StateJacobian Ft = linearization::discretize_euler(lin_sol, dt).A;

        Eigen::MatrixXd Pt_bar = Ft * prev.Pt * Ft.transpose() + params.R;

        return { { .zt = zt_bar, .Pt = Pt_bar }, lin_sol.C };
    }

    KalmanState ExtendedKalmanEstimator::correct(const dynamics::StateVector_T<double>& yt, const linearization::OutputJacobian& output_jacobian) {
        KalmanState pred = state.value();

        // C = I -> Ht = I
        linearization::OutputJacobian Ht = output_jacobian;

        Eigen::MatrixXd Kt = pred.Pt * Ht.transpose() * (Ht * pred.Pt * Ht.transpose() + params.Q).inverse(); // Kalman gain

        // C @ zt_bar = I @ zt_bar -> h(zt_bar) = zt_bar
        dynamics::StateVector_T<double> Lt = yt - pred.zt; // Innovation

        dynamics::StateVector_T<double> zt = pred.zt + Kt * Lt;

        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(pred.Pt.rows(), pred.Pt.cols());

        Eigen::MatrixXd Pt = (I - Kt * Ht) * pred.Pt * (I - Kt * Ht).transpose() + Kt * params.Q * Kt.transpose();

        return { .zt = zt, .Pt = Pt };
    }

}
