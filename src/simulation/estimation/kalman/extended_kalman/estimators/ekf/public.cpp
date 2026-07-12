#include <Eigen/Dense>
#include "simulation/estimation/kalman/private.hpp"
#include "simulation/estimation/kalman/extended_kalman/public.hpp"
#include "simulation/estimation/kalman/extended_kalman/estimators/ekf/public.hpp"

namespace estimation {

    ExtendedKalmanFilter::ExtendedKalmanFilter(const ExtendedKalmanFilterParameters& params) :
        params(params), policy(params)
    {};

    ExtendedKalmanPolicyInput ExtendedKalmanFilter::make_extended_kalman_policy_input(const ExtendedKalmanEstimatorInput& input) {
        dynamics::StateVector yt = dynamics::unpack_state(input.Yt);

        actuators::ActuatorInputsVector ut_1 = actuators::unpack_actuator_inputs(input.u_actual_prev);

        return { 
            .yt = yt,
            .ut_1 = ut_1,
            .conditions = input.conditions,
            .model = input.model,
        };
    }

    dynamics::RigidBodyState ExtendedKalmanFilter::make_ekf_state_estimate(const ExtendedKalmanEstimatorInput& input, const dynamics::StateVector& zt) {
        // EKF predicts full state, so do not add back trim state
        return make_kalman_state_estimate(input.Yt, zt);
    }

    EstimationOutput ExtendedKalmanFilter::step(const ExtendedKalmanEstimatorInput& input, double dt) {
        KalmanState kalman_state = policy.step(
            make_extended_kalman_policy_input(input),
            dt
        );

        dynamics::RigidBodyState Zt = make_ekf_state_estimate(input, kalman_state.zt);

        return { .Zt = Zt };
    }

}
