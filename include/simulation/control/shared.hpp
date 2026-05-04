#pragma once
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/actuators/surface.hpp"
#include "simulation/actuators/propulsor.hpp"
#include "simulation/guidance/guidance.hpp"
#include "simulation/trim/types.hpp"
#include "simulation/linearization/types.hpp"

namespace control {

    struct SurfaceActuatorInputs {
        double elevator_cmd = 0.0;  // [rad]
        double aileron_cmd = 0.0;   // [rad]
        double rudder_cmd = 0.0;    // [rad]
        double flap_cmd = 0.0;      // [rad]
        double spoiler_cmd = 0.0;   // [rad]
    };

    struct PropulsorActuatorInputs {
        double front_propulsor_cmd = 0.0;  // [N]
        double left_propulsor_cmd = 0.0;   // [N]
        double right_propulsor_cmd = 0.0;  // [N]
    };

    struct ControlOutput {
        SurfaceActuatorInputs surface_inputs;
        PropulsorActuatorInputs propulsor_inputs;
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
        trim::TrimActuatorInputs<double> u_sol_trim;
        linearization::TrimStateJacobian A;
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
