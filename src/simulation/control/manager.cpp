#include "simulation/control/public/manager.hpp"

namespace control
{

	ControlManagerOutput ControlManager::step(
		const ControlManagerInput& input)
	{
		const ControlComponentInputs component_inputs = build_component_inputs(
			input.Zt, input.trim_sol, input.virtual_lin_sol, input.setpoint, input.delta_mu_vec_t_1);
		return aggregate_components(component_inputs, input.dt);
	}

	ControlManagerOutput ControlManager::aggregate_components(
		const ControlComponentInputs& inputs,
		double dt)
	{
		ControlManagerOutput output{};
		VirtualControlOutput_T<double> mu{};

		if (attitude_control.has_value() && inputs.attitude_input.has_value()) {
			const ControlComponentOutput component_output =
				attitude_control.value().step(inputs.attitude_input.value(), dt);
			mu.F += component_output.mu.F;
			mu.M += component_output.mu.M;
			for (std::size_t i = 0; i < output.active_mask.size(); ++i) {
				output.active_mask[i] = output.active_mask[i] || component_output.active_mask[i];
			}
			for (std::size_t i = 0; i < output.actuator_mask.size(); ++i) {
				output.actuator_mask[i] = output.actuator_mask[i] || component_output.actuator_mask[i];
			}
		}
		if (velocity_control.has_value() && inputs.velocity_input.has_value()) {
			const ControlComponentOutput component_output =
				velocity_control.value().step(inputs.velocity_input.value(), dt);
			mu.F += component_output.mu.F;
			mu.M += component_output.mu.M;
			for (std::size_t i = 0; i < output.active_mask.size(); ++i) {
				output.active_mask[i] = output.active_mask[i] || component_output.active_mask[i];
			}
			for (std::size_t i = 0; i < output.actuator_mask.size(); ++i) {
				output.actuator_mask[i] = output.actuator_mask[i] || component_output.actuator_mask[i];
			}
		}
		if (linear_quadratic_control.has_value() && inputs.linear_quadratic_input.has_value()) {
			const ControlComponentOutput component_output =
				linear_quadratic_control.value().step(inputs.linear_quadratic_input.value(), dt);
			mu.F += component_output.mu.F;
			mu.M += component_output.mu.M;
			for (std::size_t i = 0; i < output.active_mask.size(); ++i) {
				output.active_mask[i] = output.active_mask[i] || component_output.active_mask[i];
			}
			for (std::size_t i = 0; i < output.actuator_mask.size(); ++i) {
				output.actuator_mask[i] = output.actuator_mask[i] || component_output.actuator_mask[i];
			}
		}

		output.mu = {{mu.F}, {mu.M}};
		return output;
	}

	ControlComponentInputs ControlManager::build_component_inputs(
		const dynamics::RigidBodyState& Zt,
		const trim::TrimSolution& trim_sol,
		const linearization::VirtualLocalLinearization& virtual_lin_sol,
		const guidance::GuidanceSetpoint& setpoint,
		const dynamics::WrenchVector_T<double> delta_mu_vec_t_1)
	{
		ControlComponentInputs component_inputs;

		if (attitude_control.has_value()) {
			component_inputs.attitude_input.emplace(AttitudeControlInput{
				.Zt = Zt, .setpoint = guidance::AttitudeSetpoint{setpoint}, .delta_mu_vec_t_1 = delta_mu_vec_t_1
			});
		}
		if (velocity_control.has_value()) {
			component_inputs.velocity_input.emplace(VelocityControlInput{
				.Zt = Zt, .setpoint = guidance::VelocitySetpoint{setpoint}, .delta_mu_vec_t_1 = delta_mu_vec_t_1
			});
		}
		if (linear_quadratic_control.has_value()) {
			component_inputs.linear_quadratic_input.emplace(LinearQuadraticControlInput{
				.Zt = Zt,
				.virtual_linearization = virtual_lin_sol,
				.Z_sol_trim = trim_sol.operating_point.state,
				.setpoint = guidance::LinearQuadraticSetpoint{setpoint},
				.delta_mu_vec_t_1 = delta_mu_vec_t_1
			});
		}

		return component_inputs;
	}

} // namespace control
