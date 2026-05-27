#pragma once
#include <functional>
#include <optional>
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/operating/public.hpp"

namespace vehicles { struct Aircraft; }

namespace estimation {

    struct EstimationOutput {
        dynamics::RigidBodyState zN_t;  // state estimate
    };

    struct KalmanFilterEstimatorInput {
        dynamics::RigidBodyState yN_t;
        linearization::LocalLinearization lin_sol;
        operating::OperatingPoint operating_point;
        actuators::SurfaceActuatorInputs_T<double> u_surface_actual_prev;
        actuators::PropulsorActuatorInputs_T<double> u_propulsor_actual_prev;
        operating::OperatingConditions conditions;
        vehicles::Aircraft& aircraft;
    };
    struct KalmanFilterEstimatorParameters {};

}