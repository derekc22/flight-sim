#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/estimation/kalman/estimators/lkf/public.hpp"
#include "simulation/estimation/kalman/private.hpp"

namespace estimation {

    LinearKalmanFilter::LinearKalmanFilter(const LinearKalmanFilterParameters& params) : params(params), policy(params) {}

    EstimationOutput LinearKalmanFilter::step(const KalmanFilterEstimatorInput& estimator_input) {

        KalmanState kalman_state = policy.step(
            make_kalman_filter_input(estimator_input, false)
        );

        dynamics::RigidBodyState zN_t = pack_state_estimate(estimator_input, kalman_state.z, false);

        return { .zN_t = zN_t };
    }
}
