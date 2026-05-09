#pragma once
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/actuators/surface.hpp"
#include "simulation/actuators/propulsor.hpp"
#include "simulation/guidance/guidance.hpp"
#include "simulation/linearization/types.hpp"
#include "simulation/types/types.hpp"

namespace control {

    struct ControlOutput {
        types::SurfaceActuatorInputs_T<double> surface_inputs;
        types::PropulsorActuatorInputs_T<double> propulsor_inputs;
    };

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
        types::ActuatorInputs_T<double> u_sol_trim;
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
