#pragma once
#include <functional>
#include <optional>
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/operating/public.hpp"

namespace vehicles { struct Aircraft; } // forward declare

namespace estimation {

    struct EstimationOutput {
        dynamics::RigidBodyState Zt;  // state estimate
    };

    struct LinearKalmanEstimatorInput {
        dynamics::RigidBodyState Yt;
        operating::OperatingPoint operating_point;
        linearization::LocalLinearization lin_sol;
        actuators::SurfaceActuatorInputs_T<double> u_surface_actual_prev;
        actuators::PropulsorActuatorInputs_T<double> u_propulsor_actual_prev;
    };

    struct ExtendedKalmanEstimatorInput {
        dynamics::RigidBodyState Yt;
        operating::OperatingPoint operating_point;
        actuators::SurfaceActuatorInputs_T<double> u_surface_actual_prev;
        actuators::PropulsorActuatorInputs_T<double> u_propulsor_actual_prev;
        vehicles::Aircraft& aircraft;
        operating::OperatingConditions conditions;
    };

}
