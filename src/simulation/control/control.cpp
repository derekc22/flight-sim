#include <Eigen/Dense>
#include "simulation/control/control.hpp"

namespace control {

    ControlOutput ControlProperties::step(const ControlLawInput& ctrl_law_input, bool trim_bool) {
        ControlOutput out{};

        if (!trim_bool) {
            if (axial_control_law) {
                out.surface_inputs = axial_control_law(ctrl_law_input.axial_control_law_input).surface_inputs;
            }
            if (velocity_control_law) {
                out.propulsor_inputs = velocity_control_law(ctrl_law_input.velocity_control_law_input).propulsor_inputs;
            }
        }

        if (trim_bool) {
            if (linear_full_state_feedback_control_law) {
                out = linear_full_state_feedback_control_law(ctrl_law_input.linear_full_state_feedback_control_law_input);
            }
            if (nonlinear_control_law) {
                out = nonlinear_control_law(ctrl_law_input.nonlinear_control_law_input);
            }
        }

        return out;
    }

    Eigen::VectorXd unpack_full_surface_actuator_inputs(const SurfaceActuatorInputs& u_surface) {
        Eigen::VectorXd out(constants::full_surface_input_dim);
        out << u_surface.elevator_cmd, 
               u_surface.aileron_cmd, 
               u_surface.rudder_cmd, 
               u_surface.flap_cmd,
               u_surface.spoiler_cmd;
        return out;
    }

    Eigen::VectorXd unpack_full_propulsor_actuator_inputs(const PropulsorActuatorInputs& u_propulsor) {
        Eigen::VectorXd out(constants::full_propulsor_input_dim);
        out << u_propulsor.front_propulsor_cmd, 
               u_propulsor.left_propulsor_cmd, 
               u_propulsor.right_propulsor_cmd;
        return out;
    }

    Eigen::VectorXd unpack_actuator_inputs(const SurfaceActuatorInputs& u_surface, const PropulsorActuatorInputs& u_propulsor) {
        Eigen::VectorXd out(constants::input_dim);
        out << u_surface.elevator_cmd, 
               u_surface.aileron_cmd, 
               u_surface.rudder_cmd, 
               u_propulsor.front_propulsor_cmd, 
               u_propulsor.left_propulsor_cmd, 
               u_propulsor.right_propulsor_cmd;
        return out;
    }
}
