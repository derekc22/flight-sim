#include <Eigen/Dense>
#include "simulation/control/control.hpp"
#include "simulation/types/types.hpp"

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
}
