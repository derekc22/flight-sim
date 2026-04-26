#include <algorithm>
#include <tuple>
#include "simulation/control/control.hpp"
#include "simulation/actuators/actuators.hpp"
#include "simulation/trim/trim.hpp"
#include "simulation/util/util.hpp"

namespace control {

    std::tuple<double, double, double> allocate_thrust(double T_tot, const actuators::PropulsorActuators& propulsor_actuators) {
        const actuators::PropulsorActuator& front = propulsor_actuators.front_propulsor;
        const actuators::PropulsorActuator& left = propulsor_actuators.left_propulsor;
        const actuators::PropulsorActuator& right = propulsor_actuators.right_propulsor;

        double T_min = front.limit_min + left.limit_min + right.limit_min;
        double T_max = front.limit_max + left.limit_max + right.limit_max;
        double T_cmd = util::clamp(T_tot, T_min, T_max);

        int n = 0;
        if (front.limit_max > front.limit_min) { n++; }
        if (left.limit_max > left.limit_min) { n++; }
        if (right.limit_max > right.limit_min) { n++; }
        if (n == 0) { return { front.limit_min, left.limit_min, right.limit_min }; }

        double T_each = T_cmd / n;

        double T_front = util::clamp(T_each, front.limit_min, front.limit_max);
        double T_left = util::clamp(T_each, left.limit_min, left.limit_max);
        double T_right = util::clamp(T_each, right.limit_min, right.limit_max);

        double T_remaining = T_cmd - T_front - T_left - T_right;

        T_front += util::clamp(T_remaining, 0.0, front.limit_max - T_front);
        T_remaining = T_cmd - T_front - T_left - T_right;

        T_left += util::clamp(T_remaining, 0.0, left.limit_max - T_left);
        T_remaining = T_cmd - T_front - T_left - T_right;

        T_right += util::clamp(T_remaining, 0.0, right.limit_max - T_right);

        return { T_front, T_left, T_right };
    }

    PIDControlLawInput ControlProperties::make_axis_control_input(
        const dynamics::RigidBodyState& xN_meas_t, 
        const actuators::SurfaceActuator& surface_actuator, 
        ControlType control_type
    ){

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
                    .limit_max = surface_actuator.limit_max,
                    .limit_min = surface_actuator.limit_min
                };

            case ControlType::RollPIDController:
                return {
                    .meas = eul_meas_t.phi(),
                    .meas_des = axis_setpoint.eulIB.phi(),
                    .meas_dot = xN_meas_t.w.p(),
                    .limit_max = surface_actuator.limit_max,
                    .limit_min = surface_actuator.limit_min
                };

            case ControlType::PitchDamper:
                return {
                    .meas = xN_meas_t.w.q(),
                    .meas_des = axis_setpoint.wB_BI.q(),
                    .meas_dot = 0.0,
                    .limit_max = surface_actuator.limit_max,
                    .limit_min = surface_actuator.limit_min
                };

            case ControlType::PitchPIDController:
                return {
                    .meas = eul_meas_t.theta(),
                    .meas_des = axis_setpoint.eulIB.theta(),
                    .meas_dot = xN_meas_t.w.q(),
                    .limit_max = surface_actuator.limit_max,
                    .limit_min = surface_actuator.limit_min
                };

            case ControlType::YawDamper:
                return {
                    .meas = xN_meas_t.w.r(),
                    .meas_des = axis_setpoint.wB_BI.r(),
                    .meas_dot = 0.0,
                    .limit_max = surface_actuator.limit_max,
                    .limit_min = surface_actuator.limit_min
                };

            case ControlType::YawPIDController:
                return {
                    .meas = eul_meas_t.psi(),
                    .meas_des = axis_setpoint.eulIB.psi(),
                    .meas_dot = xN_meas_t.w.r(),
                    .limit_max = surface_actuator.limit_max,
                    .limit_min = surface_actuator.limit_min
                };

            default:
                throw std::runtime_error("control::make_axis_control_input invalid control type");
        }
    }

    ControlInputs ControlProperties::step(
        const dynamics::RigidBodyState& xN_meas_t, 
        const actuators::SurfaceActuators& surface_actuators
    ){
        SurfaceActuatorInputs u_surface;
        PropulsorActuatorInputs u_propulsor;

        if (lateral_controller) {
            u_surface.aileron_cmd = lateral_controller(
                make_axis_control_input(xN_meas_t, surface_actuators.aileron, lateral_control_type)
            );
        }

        if (longitudinal_controller) {
            u_surface.elevator_cmd = longitudinal_controller(
                make_axis_control_input(xN_meas_t, surface_actuators.elevator, longitudinal_control_type)
            );
        }

        if (vertical_controller) {
            u_surface.rudder_cmd = vertical_controller(
                make_axis_control_input(xN_meas_t, surface_actuators.rudder, vertical_control_type)
            );
        }

        return { u_surface, u_propulsor };
    }

    PIDControlLawInput ControlProperties::make_velocity_control_input(
        const dynamics::RigidBodyState& xN_meas_t, 
        const actuators::PropulsorActuators& propulsor_actuators, 
        ControlType control_type
    ){

        double limit_max_overall = propulsor_actuators.front_propulsor.limit_max + propulsor_actuators.left_propulsor.limit_max + propulsor_actuators.right_propulsor.limit_max;
        double limit_min_overall = propulsor_actuators.front_propulsor.limit_min + propulsor_actuators.left_propulsor.limit_min + propulsor_actuators.right_propulsor.limit_min;

        switch (control_type) {
            case ControlType::VelocityPIDController:
                return {
                    .meas = xN_meas_t.v.data(0),
                    .meas_des = velocity_setpoint.vB_BI.data(0),
                    .limit_max = limit_max_overall,
                    .limit_min = limit_min_overall
                };
            default:
                throw std::runtime_error("control::make_velocity_control_input invalid control type");
        }
    }

    ControlInputs ControlProperties::step(
        const dynamics::RigidBodyState& xN_meas_t, 
        const actuators::PropulsorActuators& propulsor_actuators
    ){
        SurfaceActuatorInputs u_surface;
        PropulsorActuatorInputs u_propulsor;

        double T_tot = velocity_controller(
            make_velocity_control_input(xN_meas_t, propulsor_actuators, velocity_control_type)
        );

        auto [T_front, T_left, T_right] = allocate_thrust(T_tot, propulsor_actuators);

        u_propulsor.front_propulsor_cmd = T_front;
        u_propulsor.left_propulsor_cmd = T_left;
        u_propulsor.right_propulsor_cmd = T_right;

        return { u_surface, u_propulsor };
    }

    trim::TrimStateVector_T<double> full_state_control_setpoint_to_trim_state_vector(const FullStateControlSetpoint& full_state_setpoint){
        dynamics::LinearVelocity vB_BI = full_state_setpoint.vB_BI;
        dynamics::AngularVelocity wB_BI = full_state_setpoint.wB_BI;
        dynamics::EulerAngles eulIB = full_state_setpoint.eulIB;

        trim::TrimState<double> full_state_setpoint_in_trim_state_form { 
            .vx = vB_BI.data(0),
            .vy = vB_BI.data(1),
            .vz = vB_BI.data(2),
            .p = wB_BI.p(),
            .q = wB_BI.q(),
            .r = wB_BI.r(),
            .phi = eulIB.phi(),
            .theta = eulIB.theta(),
        };
        return trim::pack_trim_state_T(full_state_setpoint_in_trim_state_form);
    }

    trim::TrimStateVector_T<double> rigid_body_state_to_trim_state_vector(const dynamics::RigidBodyState& xN_meas_t){
        dynamics::LinearVelocity vB_BI = xN_meas_t.v;
        dynamics::AngularVelocity wB_BI = xN_meas_t.w;
        dynamics::EulerAngles eulIB;
        eulIB.set(xN_meas_t.q);

        trim::TrimState<double> xN_meas_in_trim_state_form { 
            .vx = vB_BI.data(0),
            .vy = vB_BI.data(1),
            .vz = vB_BI.data(2),
            .p = wB_BI.p(),
            .q = wB_BI.q(),
            .r = wB_BI.r(),
            .phi = eulIB.phi(),
            .theta = eulIB.theta(),
        };
        return trim::pack_trim_state_T(xN_meas_in_trim_state_form);
    }

    FullStateControlLawInput ControlProperties::make_full_state_control_input(
        const dynamics::RigidBodyState& xN_meas_t, 
        const linearization::TrimLinearization& lin_sol, 
        const actuators::SurfaceActuators& surface_actuators,
        const actuators::PropulsorActuators& propulsor_actuators,
        ControlType control_type
    ){

        switch (control_type) {
            case ControlType::LinearQuadraticRegulator:
            case ControlType::LinearQuadraticTracker:        
                return {
                    .A = lin_sol.A,
                    .B = lin_sol.B,
                    .meas = rigid_body_state_to_trim_state_vector(xN_meas_t),
                    .meas_des = full_state_control_setpoint_to_trim_state_vector(full_state_setpoint),
                    .surface_actuators = surface_actuators,
                    .propulsor_actuators = propulsor_actuators
                };

            default:
                throw std::runtime_error("control::make_full_state_control_input invalid control type");
        }
    }

        ControlInputs ControlProperties::step(
            const dynamics::RigidBodyState& xN_meas_t, 
            const linearization::TrimLinearization& lin_sol, 
            const trim::TrimActuatorInputs<double>& trim_sol_input, 
            const actuators::SurfaceActuators& surface_actuators,
            const actuators::PropulsorActuators& propulsor_actuators
        ){
        SurfaceActuatorInputs u_surface;
        PropulsorActuatorInputs u_propulsor;

        Eigen::VectorXd u_deviation = full_state_controller(
            make_full_state_control_input(xN_meas_t, lin_sol, surface_actuators, propulsor_actuators, full_state_control_type)
        );
        Eigen::VectorXd u_sol_trim = trim::pack_trim_control_surface_inputs_T(trim_sol_input);

        Eigen::VectorXd u_cmd = u_deviation + u_sol_trim;

        u_surface.elevator_cmd = u_cmd[0];
        u_surface.aileron_cmd = u_cmd[1];
        u_surface.rudder_cmd = u_cmd[2];

        u_propulsor.front_propulsor_cmd = u_cmd[3];
        u_propulsor.left_propulsor_cmd = u_cmd[4];
        u_propulsor.right_propulsor_cmd = u_cmd[5];

        return { u_surface, u_propulsor };
    }

    Eigen::VectorXd surface_actuator_inputs_to_vector(const SurfaceActuatorInputs& u_surface) {
        Eigen::VectorXd out(5);
        out << u_surface.elevator_cmd, u_surface.aileron_cmd, u_surface.rudder_cmd, u_surface.flap_cmd, u_surface.spoiler_cmd;
        return out;
    }

    Eigen::VectorXd propulsor_actuator_inputs_to_vector(const PropulsorActuatorInputs& u_propulsor) {
        Eigen::VectorXd out(3);
        out << u_propulsor.front_propulsor_cmd, u_propulsor.left_propulsor_cmd, u_propulsor.right_propulsor_cmd;
        return out;
    }
}
