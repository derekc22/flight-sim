#pragma once
#include "simulation/control/public/components/attitude.hpp"
#include "simulation/control/public/components/linear_quadratic.hpp"
#include "simulation/control/public/components/velocity.hpp"
#include "simulation/control/public/data/types.hpp"

#include <optional>

namespace control
{

	struct ControlManager {
		ControllerType attitude_controller_type = ControllerType::None;
		ControllerType velocity_controller_type = ControllerType::None;
		ControllerType linear_quadratic_controller_type = ControllerType::None;
		ControllerType nonlinear_controller_type = ControllerType::None;

		std::optional<AttitudeControl> attitude_control;
		std::optional<VelocityControl> velocity_control;
		std::optional<LinearQuadraticControl> linear_quadratic_control;

		/**
		 * @brief Advances the configured control components and combines their outputs.
		 *
		 * @param[in] input Current simulation, trim, linearization, guidance, allocation, and timing data.
		 * @return Combined virtual wrench and active-channel masks.
		 */
		ControlManagerOutput step(const ControlManagerInput& input);

		/**
		 * @brief Advances available control components and aggregates their commands and masks.
		 *
		 * @param[in] inputs Optional input for each configured control component.
		 * @param[in] dt Controller step [s].
		 * @return Aggregated virtual wrench and active-channel masks.
		 */
		ControlManagerOutput aggregate_components(const ControlComponentInputs& inputs, double dt);

		/**
		 * @brief Builds inputs for each configured control component.
		 *
		 * @param[in] Zt Current rigid-body state.
		 * @param[in] trim_sol Trim solution supplying the linear-quadratic reference state.
		 * @param[in] virtual_lin_sol Current virtual-system linearization.
		 * @param[in] setpoint Current guidance setpoint.
		 * @param[in] delta_mu_vec_t_1 Previous-step virtual-control allocation residual.
		 * @return Optional component inputs populated for the configured controllers.
		 */
		ControlComponentInputs build_component_inputs(const dynamics::RigidBodyState& Zt,
			const trim::TrimSolution& trim_sol,
			const linearization::VirtualLocalLinearization& virtual_lin_sol,
			const guidance::GuidanceSetpoint& setpoint,
			dynamics::WrenchVector_T<double> delta_mu_vec_t_1);
	};

} // namespace control
