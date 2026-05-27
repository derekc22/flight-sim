#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/estimation/public.hpp"
#include "simulation/estimation/kalman/public.hpp"

namespace estimation {

    dynamics::StateVector_T<double> make_measurement_deviation(const KalmanFilterEstimatorInput& estimator_input);

    actuators::ActuatorInputsVector_T<double> make_input_deviation(const KalmanFilterEstimatorInput& estimator_input);

    dynamics::RigidBodyState pack_state_estimate(const KalmanFilterEstimatorInput& estimator_input, const dynamics::StateVector_T<double>& zN_t_deviation, bool nonlinear_bool);

    KalmanFilterInput make_kalman_filter_input(const KalmanFilterEstimatorInput& estimator_input, bool nonlinear_bool);

}