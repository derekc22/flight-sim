#pragma once
#include "simulation/actuators/propulsor/shared.hpp"
#include "simulation/actuators/surface/shared.hpp"
#include "simulation/actuators/shared.hpp"
#include "simulation/control/shared.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/guidance/shared.hpp"
#include "simulation/linearization/shared.hpp"

namespace control {

    struct AxialControllerInput {
        dynamics::RigidBodyState zN_t;
        actuators::SurfaceActuators surface_actuators;
        guidance::AxialSetpoint setpoint;
    };
    struct AxialControllerParameters {};

    struct VelocityControllerInput {
        dynamics::RigidBodyState zN_t;
        actuators::PropulsorActuators propulsor_actuators;
        guidance::VelocitySetpoint setpoint;
    };
    struct VelocityControllerParameters {};

    struct LinearFullStateFeedbackControllerInput {
        dynamics::RigidBodyState zN_t;
        actuators::ActuatorInputs_T<double> u_sol_trim;
        linearization::StateJacobian A;
        linearization::TrimInputJacobian B;
        guidance::LinearFullStateFeedbackSetpoint setpoint;
    };
    struct LinearFullStateFeedbackControllerParameters {};

    struct NonlinearControllerInput {
        dynamics::RigidBodyState zN_t;
        actuators::SurfaceActuators surface_actuators;
        actuators::PropulsorActuators propulsor_actuators;
        guidance::NonlinearSetpoint setpoint;
    };
    struct NonlinearControllerParameters {};
}
