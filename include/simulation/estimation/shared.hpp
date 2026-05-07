#pragma once
#include "simulation/control/shared.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/linearization/types.hpp"
#include "simulation/trim/types.hpp"

namespace estimation {

    struct EstimationOutput {
        dynamics::RigidBodyState state_estimate;
    };

    struct KalmanFilterInput {
        dynamics::RigidBodyState zN_t;
        linearization::TrimLinearization lin_sol;
        trim::TrimSolution trim_sol;
        control::SurfaceActuatorInputs u_surface_actual_prev;
        control::PropulsorActuatorInputs u_propulsor_actual_prev;
    };

    struct KalmanFilterEstimatorParameters {};
}
