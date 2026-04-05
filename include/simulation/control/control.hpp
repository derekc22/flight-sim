#pragma once

namespace control {

    struct ControlSurfaceInputs {
        // u ∈ [−umax, +umax]
        double elevator = 0.0;  // rad
        double aileron = 0.0;   // rad
        double rudder = 0.0;    // rad
        // u ∈ [0, +umax]
        double flap = 0.0;      // rad
        double spoiler = 0.0;   // rad
    };

    struct ControlSurfaceLimits {
        // Assume symmetric limits
        double elevator_max = 0.0;  // rad
        double aileron_max = 0.0;   // rad
        double rudder_max = 0.0;    // rad
        // Assume lower limit is zero
        double flap_max = 0.0;      // rad
        double spoiler_max = 0.0;   // rad
    };

    struct ControlProperties {
        ControlSurfaceLimits limits;
    };


    struct LongitudinalPIDController{
        double Kp = 1;
        double Kd = 0;
        double Ki = 0;
    };

}
