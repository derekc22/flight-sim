#pragma once
#include <optional>

namespace control {

    struct PIDControlLawInput {
        double meas;
        double meas_des;
        std::optional<double> meas_dot;
        double limit_max;
        double limit_min;
    };

    struct PIDControlLawParamters {
        double Kp;
        double Kd;
        double Ki;
        double tau;
    };

    struct PIDControlLaw {
        const PIDControlLawParamters& params;
        double integral = 0.0;
        double d_filtered = 0.0;
        double prev_err = 0.0;

        PIDControlLaw(const PIDControlLawParamters& params);
        double step(const PIDControlLawInput& ctrl_law_input);
    };
}
