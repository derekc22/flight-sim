#pragma once
#include <optional>

namespace control { struct ControlSurfaceInputs; } // forward declare

namespace actuators {


    struct Actuator {
        double limit_max;
        double limit_min;
        double tau;
        std::optional<double> prev_cmd;
        double _step(double cmd, std::optional<double>& lag_state);
    };

    struct Elevator : Actuator {};
    struct Aileron : Actuator {};
    struct Rudder : Actuator {};
    struct Flap : Actuator {};
    struct Spoiler : Actuator {};

    /** @deprecated */
    // struct ActuatorLimits {
    //     // Assume symmetric limits
    //     double elevator_max = 0.0;  // rad
    //     double aileron_max = 0.0;   // rad
    //     double rudder_max = 0.0;    // rad
    //     // Assume lower limit is zero
    //     double flap_max = 0.0;      // rad
    //     double spoiler_max = 0.0;   // rad
    // };

    struct Actuators {
        Elevator elevator;
        Aileron aileron;
        Rudder rudder;
        Flap flaps;
        Spoiler spoilers;
    };

    struct ActuatorProperties {
        Actuators actuators;
        control::ControlSurfaceInputs step(const control::ControlSurfaceInputs& u_cmd);
    };

}
