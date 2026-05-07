#include <Eigen/Dense>
#include "simulation/control/control.hpp"

namespace control {

    ControlOutput ControlProperties::step(const ControllerInput& controller_input, bool trim_bool) {
        ControlOutput out{};

        if (!trim_bool) {
            if (axial_controller) {
                out.surface_inputs = axial_controller(controller_input.axial_controller_input).surface_inputs;
            }
            if (velocity_controller) {
                out.propulsor_inputs = velocity_controller(controller_input.velocity_controller_input).propulsor_inputs;
            }
        }

        if (trim_bool) {
            if (linear_full_state_feedback_controller) {
                out = linear_full_state_feedback_controller(controller_input.linear_full_state_feedback_controller_input);
            }
            if (nonlinear_controller) {
                out = nonlinear_controller(controller_input.nonlinear_controller_input);
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
