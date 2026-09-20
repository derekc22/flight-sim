#pragma once
#include "core/devices/public/joystick.hpp"
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/runner/public/data/types.hpp"

#include <array>
#include <optional>

namespace runner
{

	struct ControlWrapper {
		std::optional<devices::Joystick> joystick;

		guidance::GuidanceSetpoint setpoint_t_1;
		control::VirtualControlOutput mu_cmd_t_1;
		std::array<bool, constants::nv> active_mask_t_1;
		std::array<bool, constants::nu> actuator_mask_t_1;
		control::ControlOutput u_cmd_t_1;
		control::ControlOutput u_actual_t_1;

		// initialize prior-step delta mu
		dynamics::WrenchVector_T<double> delta_mu_vec_t_1{};

		ControlWrapper(bool joystick_flag,
			const actuators::SurfaceActuators& surface_actuators,
			const actuators::PropulsorActuators& propulsor_actuators);

		/**
		 * @brief Polls the configured joystick.
		 *
		 * @return Current joystick output, or a default output when no joystick is configured.
		 */
		devices::JoystickOutput poll_joystick();

		/**
		 * @brief Runs control selection, allocation, and actuator dynamics for one step.
		 *
		 * @param[in,out] input Aircraft, scheduler, state, and mode data for the step.
		 * @return Guidance setpoint, commanded controls, and actual actuator inputs.
		 */
		ControlWrapperOutput step(const ControlWrapperInput& input);
	};

} // namespace runner
