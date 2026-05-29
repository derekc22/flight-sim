#include <tuple>
#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/operating/public.hpp"
#include "simulation/trim/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/estimation/kalman/extended_kalman/public.hpp"
#include "simulation/estimation/kalman/private.hpp"

namespace estimation {

    ExtendedKalmanPolicy::ExtendedKalmanPolicy(const ExtendedKalmanPolicyParameters& params) : params(params) {}

    std::tuple<KalmanState, linearization::OutputJacobian> ExtendedKalmanPolicy::predict(const ExtendedKalmanPolicyInput& input) {
        KalmanState prev = state.value();

        dynamics::State_T<double> x = dynamics::pack_state_vector(prev.z);
        actuators::ActuatorInputs_T<double> u = actuators::pack_actuator_inputs_T<double>(input.ut_1);
        trim::TrimModel model = trim::build_trim_model(input.aircraft);

        // A @ zN_t_bar + B @ ut_1 -> f(zN_t_bar, ut_1)
        dynamics::StateDot_T<double> x_dot = trim::compute_trim_state_dot_T(x, u, model, input.conditions);
        dynamics::StateVector_T<double> zN_t_bar = prev.z + dynamics::unpack_state_dot_T(x_dot) * constants::dt;

        // A -> Ft
        operating::OperatingPoint operating_point{ .state = x, .input = u };
        linearization::LocalLinearization lin_sol = linearization::linearize_operating_point(input.aircraft, operating_point, input.conditions);
        linearization::StateJacobian Ft = linearization::discretize_euler(lin_sol).A;

        Eigen::MatrixXd Pt_bar = Ft * prev.P * Ft.transpose() + params.R;

        return { { .z = zN_t_bar, .P = Pt_bar }, lin_sol.C };
    }

    KalmanState ExtendedKalmanPolicy::correct(const ExtendedKalmanPolicyInput& input, const linearization::OutputJacobian& C) {
        KalmanState pred = state.value();

        // C = I -> Ht = I
        linearization::OutputJacobian Ht = C;

        Eigen::MatrixXd Kt = pred.P * Ht.transpose() * (Ht * pred.P * Ht.transpose() + params.Q).inverse(); // Kalman gain

        // C @ zN_t_bar = I @ zN_t_bar -> h(zN_t_bar) = zN_t_bar
        dynamics::StateVector_T<double> Lt = input.yN_t - pred.z; // Innovation

        dynamics::StateVector_T<double> zN_t = pred.z + Kt * Lt;

        Eigen::MatrixXd Inxn = Eigen::MatrixXd::Identity(pred.P.rows(), pred.P.cols());

        Eigen::MatrixXd Pt = (Inxn - Kt * Ht) * pred.P * (Inxn - Kt * Ht).transpose() + Kt * params.Q * Kt.transpose();

        return { .z = zN_t, .P = Pt };
    }

    KalmanState ExtendedKalmanPolicy::step(const ExtendedKalmanPolicyInput& input) {
        if (!state.has_value()) {
            state = KalmanState{ .z = input.yN_t, .P = params.P0 };
            return state.value();
        }

        auto [temp, C] = predict(input);
        state = temp;
        state = correct(input, C);

        return state.value();
    }

}
