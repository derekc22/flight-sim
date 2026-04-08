#pragma once

namespace actuators {

    struct ActuatorLimits {
        // Assume symmetric limits
        double elevator_max = 0.0;  // rad
        double aileron_max = 0.0;   // rad
        double rudder_max = 0.0;    // rad
        // Assume lower limit is zero
        double flap_max = 0.0;      // rad
        double spoiler_max = 0.0;   // rad
    };

    struct ActuatorProperties {
        ActuatorLimits limits;
    };

}
