#include <stdexcept>
#include "simulation/estimation/kalman/estimators/ekf/public.hpp"

namespace estimation {

    ExtendedKalmanFilter::ExtendedKalmanFilter(const ExtendedKalmanFilterParameters& params) {}

    EstimationOutput ExtendedKalmanFilter::step(const KalmanFilterInput& estimator_input) {
        throw std::runtime_error("TODO: NOT IMPLEMENTED");
    }
}
