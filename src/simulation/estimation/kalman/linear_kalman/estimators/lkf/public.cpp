#include <Eigen/Dense>
#include "simulation/estimation/kalman/linear_kalman/public.hpp"
#include "simulation/estimation/kalman/linear_kalman/estimators/lkf/public.hpp"

namespace estimation {

    LinearKalmanFilter::LinearKalmanFilter(const LinearKalmanFilterParameters& params) :
        params(params), policy(params)
    {};

    LinearKalmanPolicyInput LinearKalmanFilter::make_linear_kalman_policy_input(const LinearKalmanEstimatorInput& input) {
        dynamics::StateVector_T<double> yt_vec_deviation = dynamics::unpack_state(
            input.Yt) - dynamics::unpack_state_T(input.operating_point.state);

        actuators::ActuatorInputsVector_T<double> ut_1_vec_deviation = actuators::unpack_actuator_inputs(
            input.u_surface_actual_prev, input.u_propulsor_actual_prev) - 
            actuators::unpack_actuator_inputs_T(input.operating_point.input);

        linearization::DiscretizedLocalLinearization lin_sol_k = linearization::discretize(input.lin_sol);

        return {
            .A = lin_sol_k.A,
            .B = lin_sol_k.B,
            .C = lin_sol_k.C,
            .yt_vec = yt_vec_deviation,
            .ut_1_vec = ut_1_vec_deviation,
        };
    }

    dynamics::RigidBodyState LinearKalmanFilter::make_lkf_state_estimate(const LinearKalmanEstimatorInput& input, const dynamics::StateVector_T<double>& zt_vec_pred) {
        dynamics::StateVector_T<double> zt_vec_trim = dynamics::unpack_state_T(input.operating_point.state);
        dynamics::StateVector_T<double> zt_vec = zt_vec_pred + zt_vec_trim;  // LKF predicts deviation state, so trim state is added back

        return make_kalman_state_estimate(input.Yt, zt_vec);
    }

    EstimationOutput LinearKalmanFilter::step(const LinearKalmanEstimatorInput& input) {
        KalmanState kalman_state = policy.step(
            make_linear_kalman_policy_input(input)
        );

        dynamics::RigidBodyState Zt = make_lkf_state_estimate(input, kalman_state.zt_vec);

        return { .Zt = Zt };
    }

}
