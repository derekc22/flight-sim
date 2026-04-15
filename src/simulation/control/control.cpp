#include "simulation/control/control.hpp"
#include <algorithm>
#include "simulation/constants/constants.hpp"
#include "simulation/transforms/transforms.hpp"
#include "simulation/actuators/actuators.hpp"
#include "simulation/analysis/analysis.hpp"
#include "simulation/control/care.hpp"
#include "simulation/autopilot/autopilot.hpp"
#include "simulation/analysis/analysis.hpp"

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


    AxisControlLawInput ControlProperties::make_axis_control_input(const dynamics::RigidBodyState& xN_meas_t, const actuators::Actuator& actuator, ControlType control_type) {
        dynamics::EulerAngles eul_meas_t;
        eul_meas_t.set(xN_meas_t.q);

        // dynamics::EulerAngles eul_des_t;
        // eul_des_t.set(xN_des_t.q);

        switch (control_type) {
            case ControlType::RollDamper:
                return {
                    .meas = xN_meas_t.w.p(),
                    .meas_des = axis_setpoint.wB_BI.p(),
                    .meas_dot = 0.0,
                    .limit_max = actuator.limit_max,
                    .limit_min = actuator.limit_min
                };

            case ControlType::RollPIDController:
                return {
                    .meas = eul_meas_t.phi(),
                    .meas_des = axis_setpoint.eulIB.phi(),
                    .meas_dot = xN_meas_t.w.p(),
                    .limit_max = actuator.limit_max,
                    .limit_min = actuator.limit_min
                };

            case ControlType::PitchDamper:
                return {
                    .meas = xN_meas_t.w.q(),
                    .meas_des = axis_setpoint.wB_BI.q(),
                    .meas_dot = 0.0,
                    .limit_max = actuator.limit_max,
                    .limit_min = actuator.limit_min
                };

            case ControlType::PitchPIDController:
                return {
                    .meas = eul_meas_t.theta(),
                    .meas_des = axis_setpoint.eulIB.theta(),
                    .meas_dot = xN_meas_t.w.q(),
                    .limit_max = actuator.limit_max,
                    .limit_min = actuator.limit_min
                };

            case ControlType::YawDamper:
                return {
                    .meas = xN_meas_t.w.r(),
                    .meas_des = axis_setpoint.wB_BI.r(),
                    .meas_dot = 0.0,
                    .limit_max = actuator.limit_max,
                    .limit_min = actuator.limit_min
                };

            case ControlType::YawPIDController:
                return {
                    .meas = eul_meas_t.psi(),
                    .meas_des = axis_setpoint.eulIB.psi(),
                    .meas_dot = xN_meas_t.w.r(),
                    .limit_max = actuator.limit_max,
                    .limit_min = actuator.limit_min
                };

            default:
                throw std::runtime_error("control::make_axis_control_input invalid control type");
        }
    }

    ControlSurfaceInputs ControlProperties::step(const dynamics::RigidBodyState& xN_meas_t, const actuators::Actuators& actuators) {
        ControlSurfaceInputs u;

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
        return u;
    }

    LinearQuadraticController::LinearQuadraticController(const ControlLawParameters& params) :
        Q(params.gains.Q),
        R(params.gains.R)
    {}

    LinearQuadraticRegulator::LinearQuadraticRegulator(const ControlLawParameters& params) : LinearQuadraticController(params) {};

    LinearQuadraticTracker::LinearQuadraticTracker(const ControlLawParameters& params) : LinearQuadraticController(params) {};



    Eigen::VectorXd LinearQuadraticController::_step(const FullStateControlLawInput& ctrl_law_input) {

        if (!K_lqr.has_value()){
            const control::CareSolution care_sol = control::solve_care(ctrl_law_input.A, ctrl_law_input.B, Q, R);
            K_lqr = control::lqr_gain(ctrl_law_input.B, R, care_sol.P);
        }

        autopilot::TrimStateVector_T<double> meas_deviation = ctrl_law_input.meas - ctrl_law_input.meas_des;
        Eigen::VectorXd u_deviation = -K_lqr.value() * meas_deviation;

        return u_deviation;
    }

    autopilot::TrimStateVector_T<double> full_state_control_setpoint_to_trim_state_vector(const FullStateControlSetpoint& full_state_setpoint){
        dynamics::LinearVelocity vB_BI = full_state_setpoint.vB_BI;
        dynamics::AngularVelocity wB_BI = full_state_setpoint.wB_BI;
        dynamics::EulerAngles eulIB = full_state_setpoint.eulIB;

        autopilot::TrimState<double> full_state_setpoint_in_trim_state_form { 
            .vx = vB_BI.data(0),
            .vy = vB_BI.data(1),
            .vz = vB_BI.data(2),
            .p = wB_BI.p(),
            .q = wB_BI.q(),
            .r = wB_BI.r(),
            .phi = eulIB.phi(),
            .theta = eulIB.theta(),
        };
        return autopilot::pack_trim_state_T(full_state_setpoint_in_trim_state_form);
    }

    autopilot::TrimStateVector_T<double> rigid_body_state_to_trim_state_vector(const dynamics::RigidBodyState& xN_meas_t){
        dynamics::LinearVelocity vB_BI = xN_meas_t.v;
        dynamics::AngularVelocity wB_BI = xN_meas_t.w;
        dynamics::EulerAngles eulIB;
        eulIB.set(xN_meas_t.q);

        autopilot::TrimState<double> xN_meas_in_trim_state_form { 
            .vx = vB_BI.data(0),
            .vy = vB_BI.data(1),
            .vz = vB_BI.data(2),
            .p = wB_BI.p(),
            .q = wB_BI.q(),
            .r = wB_BI.r(),
            .phi = eulIB.phi(),
            .theta = eulIB.theta(),
        };
        return autopilot::pack_trim_state_T(xN_meas_in_trim_state_form);
    }

    FullStateControlLawInput ControlProperties::make_full_state_control_input(const dynamics::RigidBodyState& xN_meas_t, const analysis::TrimLinearization& lin_sol, const actuators::Actuators& actuators, ControlType control_type){

        switch (control_type) {
            case ControlType::LinearQuadraticRegulator:
            case ControlType::LinearQuadraticTracker:        
                return {
                    .A = lin_sol.A,
                    .B = lin_sol.B,
                    .meas = rigid_body_state_to_trim_state_vector(xN_meas_t),
                    .meas_des = full_state_control_setpoint_to_trim_state_vector(full_state_setpoint),
                    .actuators = actuators
                };

            default:
                throw std::runtime_error("control::make_full_state_control_input invalid control type");
        }
    }

    ControlSurfaceInputs ControlProperties::step(const dynamics::RigidBodyState& xN_meas_t, const analysis::TrimLinearization& lin_sol, const autopilot::TrimControlSurfaceInputs<double>& trim_sol_input, const actuators::Actuators& actuators) {
        ControlSurfaceInputs u;

        Eigen::VectorXd u_deviation = full_state_controller(
            make_full_state_control_input(xN_meas_t, lin_sol, actuators, full_state_control_type)
        );
        Eigen::VectorXd u_sol_trim = autopilot::pack_trim_control_surface_inputs_T(trim_sol_input);

        Eigen::VectorXd u_cmd = u_deviation + u_sol_trim;

        u.elevator = u_cmd[0];
        u.aileron = u_cmd[1];
        u.rudder = u_cmd[2];

        return u;
    }


}
