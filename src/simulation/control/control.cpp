#include "simulation/control/control.hpp"
#include <algorithm>
#include "simulation/constants/constants.hpp"
#include "simulation/transforms/transforms.hpp"
namespace control {

    PIDController::PIDController(const ControlLawParameters& params) :
        Kp(params.gains[0]),
        Kd(params.gains[1]),
        Ki(params.gains[2]),
        u_max(params.ctrl_surface_max),
        u_min(params.ctrl_surface_min) 
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

    double PIDController::_step(double meas, double meas_dot, double setpoint) {
        double err = setpoint - meas;

        // filtered derivative
        double alpha = std::exp(-constants::dt / tau);
        d_filtered = alpha * d_filtered + (1.0 - alpha) * meas_dot;

        // integral candidate
        double i_new = integral + err * constants::dt;

        // unsaturated control
        double u_unsat = Kp * err + Ki * i_new - Kd * d_filtered;

        // saturate
        double u = std::clamp(u_unsat, u_min, u_max);

        // anti-windup
        if ((u == u_unsat) || (u == u_max && err < 0.0) || (u == u_min && err > 0.0)) { integral = i_new; }

        return u;
    }


    ControlSurfaceInputs ControlProperties::step(const dynamics::RigidBodyState& xN_meas_t) {
        ControlSurfaceInputs u;

        if (full_state) {
            ; // placeholder
        }
        else {
            dynamics::EulerAngles eul_meas_t;
            eul_meas_t.set(xN_meas_t.q);

            dynamics::EulerAngleRates eul_dot_meas_t;
            eul_dot_meas_t.set(xN_meas_t.w, eul_meas_t);

            dynamics::EulerAngles eul_des_t;
            eul_des_t.set(xN_des_t.q);

            dynamics::EulerAngleRates eul_dot_des_t;
            eul_dot_des_t.set(xN_des_t.w, eul_des_t);

            if (longitudinal_controller && longitudinal_control_type == "RollPIDController") {
                u.aileron = longitudinal_controller(eul_meas_t.phi(), eul_dot_meas_t.phi_dot(), eul_des_t.phi());
            }
            else if (longitudinal_controller && longitudinal_control_type == "RollDamper") {
                u.aileron = longitudinal_controller(eul_dot_meas_t.phi_dot(), 0.0, eul_dot_des_t.phi_dot());
            }

            if (lateral_controller && lateral_control_type == "PitchPIDController") {
                u.elevator = lateral_controller(eul_meas_t.theta(), eul_dot_meas_t.theta_dot(), eul_des_t.theta());
            }
            else if (lateral_controller && lateral_control_type == "PitchDamper") {
                u.elevator = lateral_controller(eul_dot_meas_t.theta_dot(), 0.0, eul_dot_des_t.theta_dot());
            }

            if (vertical_controller && vertical_control_type == "YawPIDController") {
                u.rudder = vertical_controller(eul_meas_t.psi(), eul_dot_meas_t.psi_dot(), eul_des_t.psi());
            }
            else if (vertical_controller && vertical_control_type == "YawDamper") {
                u.rudder = vertical_controller(eul_dot_meas_t.psi_dot(), 0.0, eul_dot_des_t.psi_dot());
            }
        }
        return u;
    }




}
