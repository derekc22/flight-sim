#include "simulation/control/control.hpp"
#include <algorithm>
#include "simulation/constants/constants.hpp"
#include "simulation/transforms/transforms.hpp"
#include "simulation/actuators/actuators.hpp"

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

        // filtered derivative
        // double alpha = std::exp(-constants::dt / tau);
        // d_filtered = alpha * d_filtered + (1.0 - alpha) * ctrl_law_input.meas_dot;
        d_filtered = util::first_order_lag(ctrl_law_input.meas_dot, d_filtered, tau);

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


    AxisControlLawInput ControlProperties::make_axis_control_input(const dynamics::RigidBodyState& xN_meas_t, const actuators::Actuator& actuator, ControlType control_type) {
        dynamics::EulerAngles eul_meas_t;
        eul_meas_t.set(xN_meas_t.q);

        dynamics::EulerAngles eul_des_t;
        eul_des_t.set(xN_des_t.q);

        switch (control_type) {
            case ControlType::RollDamper:
                return {
                    .meas = xN_meas_t.w.p(),
                    .meas_dot = 0.0,
                    .meas_des = xN_des_t.w.p(),
                    .meas_dot_des = 0.0,
                    .limit_max = actuator.limit_max,
                    .limit_min = actuator.limit_min
                };

            case ControlType::RollPIDController:
                return {
                    .meas = eul_meas_t.phi(),
                    .meas_dot = xN_meas_t.w.p(),
                    .meas_des = eul_des_t.phi(),
                    .meas_dot_des = 0.0,
                    .limit_max = actuator.limit_max,
                    .limit_min = actuator.limit_min
                };

            case ControlType::PitchDamper:
                return {
                    .meas = xN_meas_t.w.q(),
                    .meas_dot = 0.0,
                    .meas_des = xN_des_t.w.q(),
                    .meas_dot_des = 0.0,
                    .limit_max = actuator.limit_max,
                    .limit_min = actuator.limit_min
                };

            case ControlType::PitchPIDController:
                return {
                    .meas = eul_meas_t.theta(),
                    .meas_dot = xN_meas_t.w.q(),
                    .meas_des = eul_des_t.theta(),
                    .meas_dot_des = 0.0,
                    .limit_max = actuator.limit_max,
                    .limit_min = actuator.limit_min
                };

            case ControlType::YawDamper:
                return {
                    .meas = xN_meas_t.w.r(),
                    .meas_dot = 0.0,
                    .meas_des = xN_des_t.w.r(),
                    .meas_dot_des = 0.0,
                    .limit_max = actuator.limit_max,
                    .limit_min = actuator.limit_min
                };

            case ControlType::YawPIDController:
                return {
                    .meas = eul_meas_t.psi(),
                    .meas_dot = xN_meas_t.w.r(),
                    .meas_des = eul_des_t.psi(),
                    .meas_dot_des = 0.0,
                    .limit_max = actuator.limit_max,
                    .limit_min = actuator.limit_min
                };

            default:
                throw std::runtime_error("control::make_axis_control_input invalid control type");
        }
    }

    ControlSurfaceInputs ControlProperties::step(const dynamics::RigidBodyState& xN_meas_t, const actuators::Actuators& actuators) {
        ControlSurfaceInputs u;

        if (full_state_controller) {
            ; // placeholder
        }
        else {
            if (lateral_controller) {
                u.aileron = lateral_controller(
                    make_axis_control_input(xN_meas_t, actuators.aileron, lateral_control_type)
                );
            }

            if (longitudinal_controller) {
                u.elevator = longitudinal_controller(
                    make_axis_control_input(xN_meas_t, actuators.elevator, longitudinal_control_type)
                );
            }

            if (vertical_controller) {
                u.rudder = vertical_controller(
                    make_axis_control_input(xN_meas_t, actuators.rudder, vertical_control_type)
                );
            }
        }

        return u;
    }


}
