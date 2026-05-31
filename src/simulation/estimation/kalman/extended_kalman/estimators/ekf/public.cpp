#include <Eigen/Dense>
#include "simulation/estimation/kalman/extended_kalman/public.hpp"
#include "simulation/estimation/kalman/extended_kalman/estimators/ekf/public.hpp"

namespace estimation {

    ExtendedKalmanFilter::ExtendedKalmanFilter(const ExtendedKalmanFilterParameters& params) :
        params(params), policy(params)
    {};

    ExtendedKalmanPolicyInput ExtendedKalmanFilter::make_extended_kalman_policy_input(const ExtendedKalmanEstimatorInput& input) {
        dynamics::StateVector_T<double> yN_T = dynamics::unpack_state(input.yN_t);

        actuators::ActuatorInputsVector_T<double> ut_1 = actuators::unpack_actuator_inputs(
            input.u_surface_actual_prev, 
            input.u_propulsor_actual_prev
        );

        return { 
            .yN_t = yN_T,
            .ut_1 = ut_1,
            .conditions = input.conditions,
            .aircraft = input.aircraft,
        };
    }

    dynamics::RigidBodyState ExtendedKalmanFilter::make_ekf_state_estimate(const ExtendedKalmanEstimatorInput& input, const dynamics::StateVector_T<double>& zN_t_pred) {
        dynamics::StateVector_T<double> zN_t = zN_t_pred;  // EKF predicts full state, so do not add back trim state

        return make_kalman_state_estimate(input.yN_t, zN_t);
    }

    EstimationOutput ExtendedKalmanFilter::step(const ExtendedKalmanEstimatorInput& input) {
        KalmanState kalman_state = policy.step(
            make_extended_kalman_policy_input(input)
        );

        dynamics::RigidBodyState zN_t = make_ekf_state_estimate(input, kalman_state.z);

        return { .zN_t = zN_t };
    }

}
