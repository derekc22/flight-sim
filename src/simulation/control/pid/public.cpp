#include "simulation/control/pid/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/util/public.hpp"

namespace control {

    PIDPolicy::PIDPolicy(const PIDPolicyParameters& params) : params(params) {}

    double PIDPolicy::step(const PIDPolicyInput& input) {
        double err = input.meas_des - input.meas;

        double d_term = input.meas_dot.has_value()
                        ? input.meas_dot.value()         // PI-D
                        : (prev_err - err) / constants::dt;         // PID

        // filtered deriative
        d_filtered = util::first_order_lag(d_term, d_filtered, params.tau);

        // integral candidate
        double integral_new = integral + err * constants::dt;

        // unsaturated control
        double u_unsat = params.Kp * err - params.Kd * d_filtered + params.Ki * integral_new;

        // saturate
        double u = util::clamp(u_unsat, input.limit_min, input.limit_max);

        // anti-windup
        if ((u == u_unsat) || (u == input.limit_max && err < 0.0) || (u == input.limit_min && err > 0.0)) { integral = integral_new; }

        prev_err = err;

        return u;
    }

}
