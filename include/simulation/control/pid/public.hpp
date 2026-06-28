#pragma once
#include <optional>

namespace control {

    struct PIDPolicyInput {
        double x;
        double x_des;
        std::optional<double> x_dot;
        double limit_min;
        double limit_max;
    };

    struct PIDPolicyParameters {
        double Kp;
        double Ki;
        double Kd;
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
