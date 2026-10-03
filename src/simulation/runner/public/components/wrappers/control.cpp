#include "simulation/runner/public/components/wrappers/control.hpp"

#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/allocator/public/data/helpers.hpp"
#include "simulation/allocator/public/manager.hpp"
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/constants/public/scalars.hpp"
#include "simulation/control/public/manager.hpp"
#include "simulation/guidance/public/manager.hpp"
#include "simulation/runner/public/components/scheduler.hpp"
#include "simulation/util/public/math.hpp"
#include "simulation/vehicles/public/aircraft.hpp"

#include <utility>

namespace runner
{

	ControlWrapper::ControlWrapper(
		bool joystick_flag,
		const actuators::SurfaceActuators& surface_actuators,
		const actuators::PropulsorActuators& propulsor_actuators)
	{
		// create joystick
		if (joystick_flag) {
			actuators::ActuatorLimits actuator_limits =
				actuators::pack_actuator_limits(surface_actuators, propulsor_actuators);
			joystick.emplace(actuator_limits);
		}
	}

	devices::JoystickOutput ControlWrapper::poll_joystick(
		const control::ControlOutput& u_cmd_t_1)
	{
		// declare for state machine
		devices::JoystickOutput joystick_output{};

		// fetch from joystick
		if (joystick) {
			joystick_output = joystick->step({.u_cmd_t_1 = u_cmd_t_1});
		}

		return joystick_output;
	}

	ControlWrapperOutput ControlWrapper::step(
		const ControlWrapperInput& input)
	{
		control::ControlManager& control_manager = input.aircraft.control_manager;
		guidance::GuidanceManager& guidance_manager = input.aircraft.guidance_manager;
		allocator::AllocatorManager& allocator_manager = input.aircraft.allocator_manager;

		// initialize active mask
		std::array<bool, constants::nv> active_mask;
		std::array<bool, constants::nu> actuator_mask;

		// initialize guidance setpoint
		guidance::GuidanceSetpoint setpoint{};

		// initialize virtual control command
		control::VirtualControlOutput mu_cmd{};

		// initialize control command
		control::ControlOutput u_cmd{};

		if (input.current_mode == fsm::FiniteState::Manual) {
			u_cmd = input.joystick_output.u_cmd;
		}

		// no need to rate-limit as the trim command is fixed
		else if (input.current_mode == fsm::FiniteState::AutopilotTrim) {
			mu_cmd = {};
			util::fill_arr(active_mask, 0, 6, true);
			util::fill_arr(actuator_mask, 0, 6, true);
		}

		else if (input.current_mode == fsm::FiniteState::Autopilot) {
			if (input.scheduler.guidance_tick >= constants::hz) {
				setpoint = guidance_manager.step({.kf = input.scheduler.guidance_tf}).setpoint;
				setpoint_t_1 = setpoint;

				input.scheduler.guidance_tick -= constants::hz;
			} else
				setpoint = setpoint_t_1; // perform ZOH

			if (input.scheduler.control_tick >= constants::hz) {
				double control_dt = input.scheduler.control_elapsed_ticks * constants::dt;

				control::ControlManagerOutput virtual_ctrl_out = control_manager.step(
					{.Zt = input.context.Zt,
						.trim_sol = input.trim_sol,
						.virtual_lin_sol = input.virtual_lin_sol,
						.setpoint = setpoint,
						.delta_mu_vec_t_1 = delta_mu_vec_t_1,
						.dt = control_dt});
				mu_cmd = virtual_ctrl_out.mu;
				active_mask = virtual_ctrl_out.active_mask;
				actuator_mask = virtual_ctrl_out.actuator_mask;

				mu_cmd_t_1 = mu_cmd;
				active_mask_t_1 = active_mask;
				actuator_mask_t_1 = actuator_mask;

				input.scheduler.control_tick -= constants::hz;
				input.scheduler.control_elapsed_ticks = 0;
			} else {
				mu_cmd = mu_cmd_t_1; // perform ZOH
				active_mask = active_mask_t_1;
				actuator_mask = actuator_mask_t_1;
			}
		}

		// step control allocator
		if (input.current_mode == fsm::FiniteState::AutopilotTrim ||
			input.current_mode == fsm::FiniteState::Autopilot) {
			allocator::AllocatorManagerOutput ctrl_out = allocator_manager.step(allocator::build_allocator_input(mu_cmd,
				active_mask,
				actuator_mask,
				input.context.Zt,
				input.u_actual_t_1,
				input.trim_sol.converged ? std::make_optional(input.trim_sol.operating_point.input) : std::nullopt,
				input.context.transient_conditions,
				input.context.autodiff_model));
			u_cmd = ctrl_out.u;
			delta_mu_vec_t_1 = ctrl_out.delta_mu_vec_t_1;
		}

		return {.setpoint = setpoint, .u_cmd = u_cmd};
	}

} // namespace runner
