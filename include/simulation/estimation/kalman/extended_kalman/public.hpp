#pragma once
#include <Eigen/Dense>
#include <optional>
#include <tuple>
#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/operating/public.hpp"
#include "simulation/estimation/kalman/public.hpp"

namespace vehicles { struct Aircraft; } // forward declare

namespace estimation {

    struct ExtendedKalmanPolicyInput {
        dynamics::StateVector_T<double> yt;
        actuators::ActuatorInputsVector_T<double> ut_1;
        operating::OperatingConditions conditions;
        vehicles::Aircraft& aircraft;
    };

    struct ExtendedKalmanPolicyParameters : KalmanPolicyParameters {};

    struct ExtendedKalmanPolicy {
        ExtendedKalmanPolicyParameters params;
        std::optional<KalmanState> state;

        ExtendedKalmanPolicy(const ExtendedKalmanPolicyParameters& params);
        KalmanState step(const ExtendedKalmanPolicyInput& input, double dt);

        std::tuple<KalmanState, linearization::OutputJacobian> predict(const ExtendedKalmanPolicyInput& input, double dt);
        KalmanState correct(const ExtendedKalmanPolicyInput& input, const linearization::OutputJacobian& C);
    };

}
