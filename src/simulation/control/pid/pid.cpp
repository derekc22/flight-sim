#include <algorithm>
#include "simulation/control/pid/pid.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/util/util.hpp"

namespace control {

    PIDController::PIDController(const PIDControllerParameters& params) : params(params) {}

    double PIDController::step(const PIDControllerInput& ctrl_law_input) {
        double err = ctrl_law_input.meas_des - ctrl_law_input.meas;

        double d_term = ctrl_law_input.meas_dot.has_value()
                        ? ctrl_law_input.meas_dot.value()           // PI-D
                        : (prev_err - err) / constants::dt;         // PID

        // filtered deriative
        d_filtered = util::first_order_lag(d_term, d_filtered, params.tau);

        // integral candidate
        double i_new = integral + err * constants::dt;

        // unsaturated control
        double u_unsat = params.Kp * err - params.Kd * d_filtered + params.Ki * i_new;

        // saturate
        double u = std::clamp(u_unsat, ctrl_law_input.limit_min, ctrl_law_input.limit_max);

        // anti-windup
        if ((u == u_unsat) || (u == ctrl_law_input.limit_max && err < 0.0) || (u == ctrl_law_input.limit_min && err > 0.0)) { integral = i_new; }

        prev_err = err;

        return u;
    }

}
