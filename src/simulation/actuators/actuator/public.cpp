#include <optional>
#include "simulation/actuators/actuator/public.hpp"
#include "simulation/util/public.hpp"

namespace actuators {

    Actuator::Actuator(double limit_max, double limit_min, double tau) : limit_max(limit_max), limit_min(limit_min), tau(tau) {}

    double Actuator::step(double cmd, std::optional<double>& lag_state) {
        double cmd_clamped = util::clamp(cmd, limit_min, limit_max);
        double init_lag_state = util::clamp(0.0, limit_min, limit_max);

        double prev_cmd = lag_state ? lag_state.value() : init_lag_state;
        double cmd_lagged = util::first_order_lag(cmd_clamped, prev_cmd, tau);
        lag_state = cmd_lagged;
        return cmd_lagged;
    }

}
