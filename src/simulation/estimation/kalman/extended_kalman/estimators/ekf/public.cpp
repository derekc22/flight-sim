#include <Eigen/Dense>
#include "simulation/estimation/kalman/extended_kalman/public.hpp"
#include "simulation/estimation/kalman/extended_kalman/estimators/ekf/public.hpp"

namespace estimation {

    ExtendedKalmanFilter::ExtendedKalmanFilter(const ExtendedKalmanFilterParameters& params) :
        params(params), policy(params)
    {};

    ExtendedKalmanPolicyInput ExtendedKalmanFilter::make_extended_kalman_policy_input(const ExtendedKalmanEstimatorInput& input) {
        dynamics::StateVector_T<double> yt_vec = dynamics::unpack_state(input.Yt);

        actuators::ActuatorInputsVector_T<double> ut_1_vec = actuators::unpack_actuator_inputs(
            input.u_surface_actual_prev, 
            input.u_propulsor_actual_prev
        );

        return { 
            .yt_vec = yt_vec,
            .ut_1_vec = ut_1_vec,
            .conditions = input.conditions,
            .aircraft = input.aircraft,
        };
    }

    dynamics::RigidBodyState ExtendedKalmanFilter::make_ekf_state_estimate(const ExtendedKalmanEstimatorInput& input, const dynamics::StateVector_T<double>& zt_vec_pred) {
        dynamics::StateVector_T<double> zt_vec = zt_vec_pred;  // EKF predicts full state, so do not add back trim state

        return make_kalman_state_estimate(input.Yt, zt_vec);
    }

    EstimationOutput ExtendedKalmanFilter::step(const ExtendedKalmanEstimatorInput& input) {
        KalmanState kalman_state = policy.step(
            make_extended_kalman_policy_input(input)
        );

        dynamics::RigidBodyState Zt = make_ekf_state_estimate(input, kalman_state.zt_vec);

        return { .Zt = Zt };
    }

}
