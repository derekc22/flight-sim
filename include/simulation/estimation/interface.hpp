#pragma once
#include "simulation/actuators/propulsor/shared.hpp"
#include "simulation/actuators/surface/shared.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/linearization/shared.hpp"
#include "simulation/trim/shared.hpp"

namespace estimation {

    struct EstimationOutput {
        dynamics::RigidBodyState zN_t;  // state estimate
    };

    struct KalmanFilterInput {
        dynamics::RigidBodyState yN_t;
        linearization::TrimLinearization lin_sol;
        trim::TrimSolution trim_sol;
        actuators::SurfaceActuatorInputs_T<double> u_surface_actual_prev;
        actuators::PropulsorActuatorInputs_T<double> u_propulsor_actual_prev;
    };

    struct KalmanFilterEstimatorParameters {};
}
