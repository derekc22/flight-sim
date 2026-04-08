#include "simulation/control/control.hpp"
#include <algorithm>
#include "simulation/constants/constants.hpp"
#include "simulation/transforms/transforms.hpp"
#include "simulation/actuators/actuators.hpp"

namespace control {

    PIDController::PIDController(const ControlLawParameters& params) :
        Kp(params.gains[0]),
        Kd(params.gains[1]),
        Ki(params.gains[2])
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

    double PIDController::_step(const ControlLawInput& ctrl_law_input) {
        double err = ctrl_law_input.meas_des - ctrl_law_input.meas;

        // filtered derivative
        double alpha = std::exp(-constants::dt / tau);
        d_filtered = alpha * d_filtered + (1.0 - alpha) * ctrl_law_input.meas_dot;

        // derivative error
        double meas_dot_filtered = d_filtered;
        double meas_dot_err = ctrl_law_input.meas_dot_des - meas_dot_filtered; // technically PI-D, not PID

        // integral candidate
        double i_new = integral + err * constants::dt;

        // unsaturated control
        double u_unsat = Kp * err + Kd * meas_dot_err + Ki * i_new;

        // saturate
        double u = std::clamp(u_unsat, ctrl_law_input.limit_min, ctrl_law_input.limit_max);

        // anti-windup
        if ((u == u_unsat) || (u == ctrl_law_input.limit_max && err < 0.0) || (u == ctrl_law_input.limit_min && err > 0.0)) { integral = i_new; }

        return u;
    }


    ControlSurfaceInputs ControlProperties::step(const dynamics::RigidBodyState& xN_meas_t, const actuators::ActuatorLimits& limits) {
        ControlSurfaceInputs u;
        ControlLawInput ctrl_law_input;

        if (full_state) {
            ; // placeholder
        }
        else {
            dynamics::EulerAngles eul_meas_t;
            eul_meas_t.set(xN_meas_t.q);

            dynamics::EulerAngles eul_des_t;
            eul_des_t.set(xN_des_t.q);


            if (lateral_controller) {
                if (lateral_control_type == "RollPIDController") {
                    ctrl_law_input = {
                        .meas = eul_meas_t.phi(),
                        .meas_dot = xN_meas_t.w.p(),
                        .meas_des = eul_des_t.phi(),
                        .meas_dot_des = 0.0,
                        .limit_max = limits.aileron_max,
                        .limit_min = -limits.aileron_max
                    };
                    u.aileron = lateral_controller(ctrl_law_input);
                }
                else if (lateral_control_type == "RollDamper") {
                    ctrl_law_input = {
                        .meas = xN_meas_t.w.p(),
                        .meas_dot = 0.0,
                        .meas_des = xN_des_t.w.p(),
                        .meas_dot_des = 0.0,
                        .limit_max = limits.aileron_max,
                        .limit_min = -limits.aileron_max
                    };
                    u.aileron = lateral_controller(ctrl_law_input);
                }
            }

            if (longitudinal_controller) {
                if (longitudinal_control_type == "PitchPIDController") {
                    ctrl_law_input = {
                        .meas = eul_meas_t.theta(),
                        .meas_dot = xN_meas_t.w.q(),
                        .meas_des = eul_des_t.theta(),
                        .meas_dot_des = 0.0,
                        .limit_max = limits.elevator_max,
                        .limit_min = -limits.elevator_max
                    };
                    u.elevator = longitudinal_controller(ctrl_law_input);
                }
                else if (longitudinal_control_type == "PitchDamper") {
                    ctrl_law_input = {
                        .meas = xN_meas_t.w.q(),
                        .meas_dot = 0.0,
                        .meas_des = xN_des_t.w.q(),
                        .meas_dot_des = 0.0,
                        .limit_max = limits.elevator_max,
                        .limit_min = -limits.elevator_max
                    };
                    u.elevator = longitudinal_controller(ctrl_law_input);
                }
            }

            if (vertical_controller) {
                if (vertical_control_type == "YawPIDController") {
                    ctrl_law_input = {
                        .meas = eul_meas_t.psi(),
                        .meas_dot = xN_meas_t.w.r(),
                        .meas_des = eul_des_t.psi(),
                        .meas_dot_des = 0.0,
                        .limit_max = limits.rudder_max,
                        .limit_min = -limits.rudder_max,
                    };
                    u.rudder = vertical_controller(ctrl_law_input);
                }
                else if (vertical_control_type == "YawDamper") {
                    ctrl_law_input = {
                        .meas = xN_meas_t.w.r(),
                        .meas_dot = 0.0,
                        .meas_des = xN_des_t.w.r(),
                        .meas_dot_des = 0.0,
                        .limit_max = limits.rudder_max,
                        .limit_min = -limits.rudder_max,
                    };
                    u.rudder = vertical_controller(ctrl_law_input);
                }
            }
        }
        return u;
    }




}
