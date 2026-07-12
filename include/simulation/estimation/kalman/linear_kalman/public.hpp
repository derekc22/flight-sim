#pragma once
#include <Eigen/Dense>
#include <optional>
#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/estimation/kalman/public.hpp"

namespace estimation {

    struct LinearKalmanPolicyInput {
        linearization::StateJacobian A; 
        linearization::InputJacobian B;
        linearization::OutputJacobian C;
        dynamics::StateVector yt;
        actuators::ActuatorInputsVector ut_1;
    };

    struct LinearKalmanPolicyParameters : KalmanPolicyParameters {};

    struct LinearKalmanPolicy {
        LinearKalmanPolicyParameters params;
        std::optional<KalmanState> state;

        LinearKalmanPolicy(const LinearKalmanPolicyParameters& params);
        KalmanState step(const LinearKalmanPolicyInput& input);

        KalmanState predict(const LinearKalmanPolicyInput& input);
        KalmanState correct(const LinearKalmanPolicyInput& input);
    };

}
