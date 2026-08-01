#pragma once
#include <functional>
#include <optional>
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/trim/public.hpp"
#include "simulation/operating/public.hpp"
#include "simulation/autodiff/public.hpp"

namespace estimation {

    struct EstimationOutput {
        dynamics::RigidBodyState Zt;  // state estimate
    };

    struct LinearKalmanEstimatorInput {
        dynamics::RigidBodyState Yt;
        operating::OperatingPoint_T<double> operating_point;
        linearization::LocalLinearization lin_sol;
        actuators::ActuatorInputs_T<double> u_actual_t_1;
    };

    struct ExtendedKalmanEstimatorInput {
        dynamics::RigidBodyState Yt;
        actuators::ActuatorInputs_T<double> u_actual_t_1;
        autodiff::AutoDiffModel& model;
        operating::OperatingConditions conditions;
    };

}
