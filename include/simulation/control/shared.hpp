#pragma once
#include "simulation/dynamics/dynamics.hpp"

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

    struct AxialControlLawInput {};
    struct AxialControlLawParameters {};

    struct VelocityControlLawInput {};
    struct VelocityControlLawParameters {};

    struct LinearFullStateFeedbackControlLawInput {};
    struct LinearFullStateFeedbackControlLawParameters {};

    struct NonlinearControlLawInput {};
    struct NonlinearControlLawParameters {};
}
