#pragma once
#include <optional>

namespace control {

    struct PIDPolicyInput {
        double meas;
        double meas_des;
        std::optional<double> meas_dot;
        double limit_max;
        double limit_min;
    };

    struct PIDPolicyParameters {
        double Kp;
        double Kd;
        double Ki;
        double tau;
    };

    struct PIDPolicy {
        PIDPolicyParameters params;
        double integral = 0.0;
        double d_filtered = 0.0;
        double prev_err = 0.0;

        PIDPolicy(const PIDPolicyParameters& params);
        double step(const PIDPolicyInput& input);
    };
}
