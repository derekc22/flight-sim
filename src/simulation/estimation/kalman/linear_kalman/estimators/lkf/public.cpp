#include <Eigen/Dense>
#include "simulation/estimation/kalman/linear_kalman/public.hpp"
#include "simulation/estimation/kalman/linear_kalman/estimators/lkf/public.hpp"

namespace estimation {

    LinearKalmanFilter::LinearKalmanFilter(const LinearKalmanFilterParameters& params) :
        params(params), policy(params)
    {};

    LinearKalmanPolicyInput LinearKalmanFilter::make_linear_kalman_policy_input(const LinearKalmanEstimatorInput& input) {
        dynamics::StateVector_T<double> yN_t_deviation = dynamics::unpack_rigid_body_state(
            input.yN_t) - dynamics::unpack_state_T(input.operating_point.state);

        actuators::ActuatorInputsVector_T<double> ut_1_deviation = actuators::unpack_actuator_inputs(
            input.u_surface_actual_prev, input.u_propulsor_actual_prev) - 
            actuators::unpack_actuator_inputs_T(input.operating_point.input);

        linearization::DiscretizedLocalLinearization lin_sol_k = linearization::discretize(input.lin_sol);

        return {
            .A = lin_sol_k.A,
            .B = lin_sol_k.B,
            .C = lin_sol_k.C,
            .yN_t = yN_t_deviation,
            .ut_1 = ut_1_deviation,
        };
    }

    dynamics::RigidBodyState LinearKalmanFilter::pack_linear_kalman_policy_state_estimate(const LinearKalmanEstimatorInput& input, const dynamics::StateVector_T<double>& zN_t_pred) {
        dynamics::StateVector_T<double> zN_trim = dynamics::unpack_state_T(input.operating_point.state);
        dynamics::StateVector_T<double> zN_t = zN_t_pred + zN_trim;  // LKF predicts deviation state, so trim state is added back

        return pack_kalman_state_estimate(input.yN_t, zN_t);
    }

    EstimationOutput LinearKalmanFilter::step(const LinearKalmanEstimatorInput& input) {
        KalmanState kalman_state = policy.step(
            make_linear_kalman_policy_input(input)
        );

        dynamics::RigidBodyState zN_t = pack_linear_kalman_policy_state_estimate(input, kalman_state.z);

        return { .zN_t = zN_t };
    }

}
