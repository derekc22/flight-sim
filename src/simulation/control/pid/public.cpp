#include <algorithm>
#include "simulation/control/pid/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/util/public.hpp"

namespace control {

    PIDController::PIDController(const PIDControllerParameters& params) : params(params) {}

    double PIDController::step(const PIDControllerInput& controller_input) {
        double err = controller_input.meas_des - controller_input.meas;

        double d_term = controller_input.meas_dot.has_value()
                        ? controller_input.meas_dot.value()         // PI-D
                        : (prev_err - err) / constants::dt;         // PID

        // filtered deriative
        d_filtered = util::first_order_lag(d_term, d_filtered, params.tau);

        // integral candidate
        double i_new = integral + err * constants::dt;

        // unsaturated control
        double u_unsat = params.Kp * err - params.Kd * d_filtered + params.Ki * i_new;

        // saturate
        double u = std::clamp(u_unsat, controller_input.limit_min, controller_input.limit_max);

        // anti-windup
        if ((u == u_unsat) || (u == controller_input.limit_max && err < 0.0) || (u == controller_input.limit_min && err > 0.0)) { integral = i_new; }

        prev_err = err;

        return u;
    }

}
