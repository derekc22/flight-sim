#include <Eigen/Dense>
#include "simulation/estimation/kalman/private.hpp"
#include "simulation/estimation/kalman/linear_kalman/public.hpp"
#include "simulation/estimation/kalman/linear_kalman/estimators/lkf/public.hpp"

namespace estimation {

    LinearKalmanFilter::LinearKalmanFilter(const LinearKalmanFilterParameters& params) :
        policy(params)
    {};

    LinearKalmanPolicyInput LinearKalmanFilter::make_linear_kalman_policy_input(const LinearKalmanEstimatorInput& input, double dt) {
        dynamics::StateVector_T<double> yt_deviation = dynamics::unpack_state(
            input.Yt) - dynamics::unpack_state_T(input.operating_point.state);

        actuators::ActuatorInputsVector_T<double> ut_1_deviation = actuators::unpack_actuator_inputs_T(input.u_cmd_t_1) -
            actuators::unpack_actuator_inputs_T(input.operating_point.input);

        linearization::DiscretizedLocalLinearization lin_sol_k = linearization::discretize(input.lin_sol, dt);

        return {
            .A = lin_sol_k.A,
            .B = lin_sol_k.B,
            .C = lin_sol_k.C,
            .yt = yt_deviation,
            .ut_1 = ut_1_deviation,
        };
    }

    dynamics::RigidBodyState LinearKalmanFilter::make_lkf_state_estimate(const LinearKalmanEstimatorInput& input, const dynamics::StateVector_T<double>& zt) {
        dynamics::StateVector_T<double> x_trim = dynamics::unpack_state_T(input.operating_point.state);
        dynamics::StateVector_T<double> zt_full = zt + x_trim;  // LKF predicts deviation state, so trim state is added back

        return make_kalman_state_estimate(input.Yt, zt_full);
    }

    EstimationOutput LinearKalmanFilter::step(const LinearKalmanEstimatorInput& input, double dt) {
        KalmanState kalman_state = policy.step(
            make_linear_kalman_policy_input(input, dt)
        );

        dynamics::RigidBodyState Zt = make_lkf_state_estimate(input, kalman_state.zt);

        return { .Zt = Zt };
    }

}
