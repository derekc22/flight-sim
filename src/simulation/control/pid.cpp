#include <algorithm>
#include "simulation/control/control.hpp"
#include "simulation/control/pid.hpp"
#include "simulation/constants/constants.hpp"

namespace control {

    PIDController::PIDController(const ControlLawParameters& params) :
        Kp(params.gains.Kp),
        Kd(params.gains.Kd),
        Ki(params.gains.Ki),
        tau(params.tau)
    {}

    RollPIDController::RollPIDController(const ControlLawParameters& params) : PIDController(params) {}

    PitchPIDController::PitchPIDController(const ControlLawParameters& params) : PIDController(params) {}

    YawPIDController::YawPIDController(const ControlLawParameters& params) : PIDController(params) {}

    RollDamper::RollDamper(const ControlLawParameters& params) : PIDController(params) {
        Kd = 0.0;
        Ki = 0.0;
    }

    PitchDamper::PitchDamper(const ControlLawParameters& params) : PIDController(params) {
        Kd = 0.0;
        Ki = 0.0;
    }

    YawDamper::YawDamper(const ControlLawParameters& params) : PIDController(params) {
        Kd = 0.0;
        Ki = 0.0;
    }

    double PIDController::_step(const AxisControlLawInput& ctrl_law_input) {
        double err = ctrl_law_input.meas_des - ctrl_law_input.meas;

        double d_term = ctrl_law_input.meas_dot.has_value()
                ? ctrl_law_input.meas_dot.value()           // PI-D
                : (prev_err - err) / constants::dt;         // PID

        // filtered deriative
        d_filtered = util::first_order_lag(d_term, d_filtered, tau);

        // integral candidate
        double i_new = integral + err * constants::dt;

        // unsaturated control
        double u_unsat = Kp * err - Kd * d_filtered + Ki * i_new;

        // saturate
        double u = std::clamp(u_unsat, ctrl_law_input.limit_min, ctrl_law_input.limit_max);

        // anti-windup
        if ((u == u_unsat) || (u == ctrl_law_input.limit_max && err < 0.0) || (u == ctrl_law_input.limit_min && err > 0.0)) { integral = i_new; }

        prev_err = err;

        return u;
    }

}
