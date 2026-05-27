#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/estimation/kalman/public.hpp"
#include "simulation/operating/public.hpp"
#include "simulation/trim/public.hpp"

namespace estimation {

    KalmanFilter::KalmanFilter(const KalmanFilterParameters& params) : params(params) {}

    KalmanState KalmanFilter::predict_linear(const KalmanFilterInput& filter_input) {
        KalmanState prev = state.value();

        dynamics::StateVector_T<double> zN_t_bar = filter_input.A * prev.z + filter_input.B * filter_input.ut_1;

        Eigen::MatrixXd Pt_bar = filter_input.A * prev.P * filter_input.A.transpose() + params.R;

        return { .z = zN_t_bar, .P = Pt_bar };
    }

    KalmanState KalmanFilter::correct_linear(const KalmanFilterInput& filter_input) {
        KalmanState pred = state.value();

        Eigen::MatrixXd Kt = pred.P * filter_input.C.transpose() * (filter_input.C * pred.P * filter_input.C.transpose() + params.Q).inverse(); // Kalman gain

        dynamics::StateVector_T<double> Lt = filter_input.yN_t - filter_input.C * pred.z; // Innovation

        dynamics::StateVector_T<double> zN_t = pred.z + Kt * Lt;

        Eigen::MatrixXd Inxn = Eigen::MatrixXd::Identity(pred.P.rows(), pred.P.cols());

        Eigen::MatrixXd Pt = (Inxn - Kt * filter_input.C) * pred.P * (Inxn - Kt * filter_input.C).transpose() + Kt * params.Q * Kt.transpose();

        return { .z = zN_t, .P = Pt };
    }

    KalmanState KalmanFilter::predict_nonlinear(const KalmanFilterInput& filter_input) {
        KalmanState prev = state.value();

        dynamics::State_T<double> x = dynamics::pack_state_vector(prev.z);
        actuators::ActuatorInputs_T<double> u = actuators::pack_actuator_inputs_T<double>(filter_input.ut_1);
        trim::TrimModel model = trim::build_trim_model(filter_input.aircraft);

        // A @ zN_t_bar + B @ ut_1 -> f(zN_t_bar, ut_1)
        dynamics::StateDot_T<double> x_dot = trim::compute_trim_state_dot_T(x, u, model, filter_input.conditions);
        dynamics::StateVector_T<double> zN_t_bar = prev.z + dynamics::unpack_state_dot_T(x_dot) * constants::dt;

        // A -> Ft
        operating::OperatingPoint operating_point{ .state = x, .input = u };
        linearization::LocalLinearization lin_sol = linearization::linearize_operating_point(filter_input.aircraft, operating_point, filter_input.conditions);
        linearization::StateJacobian Ft = linearization::discretize_euler(lin_sol).A;

        Eigen::MatrixXd Pt_bar = Ft * prev.P * Ft.transpose() + params.R;

        return { .z = zN_t_bar, .P = Pt_bar };
    }

    KalmanState KalmanFilter::correct_nonlinear(const KalmanFilterInput& filter_input) {
        KalmanState pred = state.value();

        // C = I -> Ht = I
        linearization::OutputJacobian Ht = filter_input.C;

        Eigen::MatrixXd Kt = pred.P * Ht.transpose() * (Ht * pred.P * Ht.transpose() + params.Q).inverse(); // Kalman gain

        // C @ zN_t_bar = I @ zN_t_bar -> h(zN_t_bar) = zN_t_bar
        dynamics::StateVector_T<double> Lt = filter_input.yN_t - pred.z; // Innovation

        dynamics::StateVector_T<double> zN_t = pred.z + Kt * Lt;

        Eigen::MatrixXd Inxn = Eigen::MatrixXd::Identity(pred.P.rows(), pred.P.cols());

        Eigen::MatrixXd Pt = (Inxn - Kt * Ht) * pred.P * (Inxn - Kt * Ht).transpose() + Kt * params.Q * Kt.transpose();

        return { .z = zN_t, .P = Pt };
    }


    KalmanState KalmanFilter::step(const KalmanFilterInput& filter_input) {
        if (!state.has_value()) {
            state = KalmanState{ .z = filter_input.yN_t, .P = params.P0 };
            return state.value();
        }

        state = filter_input.nonlinear_bool ? predict_nonlinear(filter_input) : predict_linear(filter_input);
        state = filter_input.nonlinear_bool ? correct_nonlinear(filter_input) : correct_linear(filter_input);

        return state.value();
    }

}
