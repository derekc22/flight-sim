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

    struct AxialControlLawInput {
        dynamics::RigidBodyState zN_t;
        actuators::SurfaceActuators surface_actuators;
        guidance::AxialSetpoint setpoint;
    };
    struct AxialControlLawParameters {};

    struct VelocityControlLawInput {
        dynamics::RigidBodyState zN_t;
        actuators::PropulsorActuators propulsor_actuators;
        guidance::VelocitySetpoint setpoint;
    };
    struct VelocityControlLawParameters {};

    struct LinearFullStateFeedbackControlLawInput {
        dynamics::RigidBodyState zN_t;
        trim::TrimActuatorInputs<double> u_sol_trim;
        linearization::TrimStateJacobian A;
        linearization::TrimInputJacobian B;
        guidance::LinearFullStateFeedbackSetpoint setpoint;
    };
    struct LinearFullStateFeedbackControlLawParameters {};

    struct NonlinearControlLawInput {
        dynamics::RigidBodyState zN_t;
        actuators::SurfaceActuators surface_actuators;
        actuators::PropulsorActuators propulsor_actuators;
        guidance::NonlinearSetpoint setpoint;
    };
    struct NonlinearControlLawParameters {};
}
