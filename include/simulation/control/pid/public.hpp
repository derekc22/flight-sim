#pragma once
#include <optional>

namespace control {

    struct PIDControllerInput {
        double meas;
        double meas_des;
        std::optional<double> meas_dot;
        double limit_max;
        double limit_min;
    };

    struct PIDControllerParameters {
        double Kp;
        double Kd;
        double Ki;
        double tau;
    };

    struct PIDController {
        PIDControllerParameters params;
        double integral = 0.0;
        double d_filtered = 0.0;
        double prev_err = 0.0;

        PIDController(const PIDControllerParameters& params);
        double step(const PIDControllerInput& controller_input);
    };
}
