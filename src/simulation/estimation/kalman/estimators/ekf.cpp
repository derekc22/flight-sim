#include <stdexcept>
#include "simulation/estimation/kalman/estimators/ekf.hpp"

namespace estimation {

    ExtendedKalmanFilter::ExtendedKalmanFilter(const ExtendedKalmanFilterParameters& params) {}

    EstimationOutput ExtendedKalmanFilter::step(const KalmanFilterInput& kalman_filter_input) {
        throw std::runtime_error("TODO: NOT IMPLEMENTED");
    }
}
