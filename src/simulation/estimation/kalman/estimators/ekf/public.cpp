#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/estimation/kalman/estimators/ekf/public.hpp"
#include "simulation/estimation/kalman/private.hpp"

namespace estimation {

    ExtendedKalmanFilter::ExtendedKalmanFilter(const ExtendedKalmanFilterParameters& params) : params(params), policy(params) {}

    EstimationOutput ExtendedKalmanFilter::step(const KalmanFilterEstimatorInput& estimator_input) {

        KalmanState kalman_state = policy.step(
            make_kalman_filter_input(estimator_input, true)
        );

        dynamics::RigidBodyState zN_t = pack_state_estimate(estimator_input, kalman_state.z, true);

        return { .zN_t = zN_t };
    }
}
