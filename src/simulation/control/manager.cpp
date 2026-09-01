#include "simulation/control/public/manager.hpp"

namespace control {

    ControlManagerOutput ControlManager::step(const ControlManagerInput& input) {
        const ControlComponentInputs component_inputs = build_component_inputs(
            input.estimated_state,
            input.trim_solution,
            input.linearization,
            input.surface_actuators,
            input.propulsor_actuators,
            input.setpoint,
            input.previous_control_residual
        );
        return step(component_inputs, input.dt);
    }

    ControlManagerOutput ControlManager::step(const ControlComponentInputs& inputs, double dt) {
        ControlManagerOutput output{};
        VirtualControlOutput_T<double> virtual_control{};

        if (attitude_control.has_value() && inputs.attitude_input.has_value()) {
            const ControlComponentOutput component_output = attitude_control.value().step(inputs.attitude_input.value(), dt);
            virtual_control.F += component_output.virtual_control.F;
            virtual_control.M += component_output.virtual_control.M;
            for (std::size_t i = 0; i < output.active_mask.size(); ++i) {
                output.active_mask[i] = output.active_mask[i] || component_output.active_mask[i];
            }
            for (std::size_t i = 0; i < output.actuator_mask.size(); ++i) {
                output.actuator_mask[i] = output.actuator_mask[i] || component_output.actuator_mask[i];
            }
        }
        if (velocity_control.has_value() && inputs.velocity_input.has_value()) {
            const ControlComponentOutput component_output = velocity_control.value().step(inputs.velocity_input.value(), dt);
            virtual_control.F += component_output.virtual_control.F;
            virtual_control.M += component_output.virtual_control.M;
            for (std::size_t i = 0; i < output.active_mask.size(); ++i) {
                output.active_mask[i] = output.active_mask[i] || component_output.active_mask[i];
            }
            for (std::size_t i = 0; i < output.actuator_mask.size(); ++i) {
                output.actuator_mask[i] = output.actuator_mask[i] || component_output.actuator_mask[i];
            }
        }
        if (full_state_control.has_value() && inputs.full_state_input.has_value()) {
            const ControlComponentOutput component_output = full_state_control.value().step(inputs.full_state_input.value(), dt);
            virtual_control.F += component_output.virtual_control.F;
            virtual_control.M += component_output.virtual_control.M;
            for (std::size_t i = 0; i < output.active_mask.size(); ++i) {
                output.active_mask[i] = output.active_mask[i] || component_output.active_mask[i];
            }
            for (std::size_t i = 0; i < output.actuator_mask.size(); ++i) {
                output.actuator_mask[i] = output.actuator_mask[i] || component_output.actuator_mask[i];
            }
        }

        output.virtual_control = {
            { virtual_control.F },
            { virtual_control.M }
        };
        return output;
    }

    ControlComponentInputs ControlManager::build_component_inputs(
        const dynamics::RigidBodyState& Zt, 
        const trim::TrimSolution& trim_sol, 
        const linearization::VirtualLocalLinearization& virtual_lin_sol, 
        const actuators::SurfaceActuators& surface_actuators, 
        const actuators::PropulsorActuators& propulsor_actuators, 
        const guidance::GuidanceSetpoint& setpoint,
        const dynamics::WrenchVector_T<double> delta_mu_vec_t_1
    ) {
        ControlComponentInputs component_inputs;

        if (attitude_control.has_value()) {
            component_inputs.attitude_input.emplace(
				AttitudeControlInput{
			    	.estimated_state = Zt,
			    	.setpoint = guidance::AttitudeSetpoint{ setpoint },
			    	.previous_control_residual = delta_mu_vec_t_1
			    }
            );
        }
        if (velocity_control.has_value()) {
            component_inputs.velocity_input.emplace(
                VelocityControlInput{
                    .estimated_state = Zt,
                    .setpoint = guidance::VelocitySetpoint{ setpoint },
                    .previous_control_residual = delta_mu_vec_t_1
                }
            );
        }
        if (full_state_control.has_value()) {
            component_inputs.full_state_input.emplace(
				    FullStateControlInput{
				    	.estimated_state = Zt,
				    	.linearization = virtual_lin_sol,
				    	.trim_state = trim_sol.operating_point.state,
                    .surface_actuators = surface_actuators,
                    .propulsor_actuators = propulsor_actuators,
				    .setpoint = setpoint,
				    	.previous_control_residual = delta_mu_vec_t_1
				    }
            );
        }

		return component_inputs;
	}

}
