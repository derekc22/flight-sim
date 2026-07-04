#include <Eigen/Dense>
#include <stdexcept>
#include "simulation/control/public.hpp"

namespace control {

    ControlOutput ControlProperties::step(const ControllerInputs& inputs, double dt, bool trim_flag) {
        ControlOutput out{};

        if (!trim_flag) {
            if (attitude_controller) {
                out.surface_inputs = attitude_controller(inputs.attitude_controller_input, dt).surface_inputs;
            }
            if (velocity_controller) {
                out.propulsor_inputs = velocity_controller(inputs.velocity_controller_input, dt).propulsor_inputs;
            }
            if (nonlinear_controller) {
                out = nonlinear_controller(inputs.nonlinear_controller_input, dt);
            }
            if (linear_quadratic_controller) { 
                throw std::runtime_error("control::ControlProperties::step LinearQuadraticController requires trim"); 
            }
        }

        if (trim_flag) {
            if (linear_quadratic_controller) {
                out = linear_quadratic_controller(inputs.linear_quadratic_controller_input, dt);
            }
        }

        return out;
    }
}
