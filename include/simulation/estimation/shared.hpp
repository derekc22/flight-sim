#pragma once
#include "simulation/control/shared.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/linearization/types.hpp"

namespace estimation {

    struct EstimationOutput {
        dynamics::RigidBodyState zN_t;  // state estimate
    };

    struct KalmanFilterInput {
        dynamics::RigidBodyState yN_t;
        linearization::TrimLinearization lin_sol;
        trim::TrimSolution trim_sol;
        types::SurfaceActuatorInputs_T<double> u_surface_actual_prev;
        types::PropulsorActuatorInputs_T<double> u_propulsor_actual_prev;
    };

    struct KalmanFilterEstimatorParameters {};
}
