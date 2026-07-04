#pragma once
#include <optional>

namespace actuators {

    struct Actuator {
        double limit_max;
        double limit_min;
        double tau;
        std::optional<double> prev_cmd;
        double step(double cmd, std::optional<double>& lag_state, double dt);

        Actuator(double limit_max, double limit_min, double tau);
        Actuator();
    };

}
