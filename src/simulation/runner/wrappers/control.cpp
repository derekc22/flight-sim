#include "simulation/runner/public/wrappers/control.hpp"

#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/actuators/public/manager.hpp"
#include "simulation/allocator/public/data/helpers.hpp"
#include "simulation/allocator/public/manager.hpp"
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/constants/public/scalars.hpp"
#include "simulation/control/public/manager.hpp"
#include "simulation/guidance/public/manager.hpp"
#include "simulation/runner/public/scheduling/scheduler.hpp"
#include "simulation/util/public/math.hpp"
#include "simulation/vehicles/public/aircraft.hpp"

#include <utility>

namespace runner
{

	ControlWrapper::ControlWrapper(
		const JSONFlags& flags,
		const actuators::SurfaceActuators& surface_actuators,
		const actuators::PropulsorActuators& propulsor_actuators)
	{
		// set u_actual_t_1 to match actuators' neutral initialization
		u_actual_t_1 = actuators::get_neutral_actuator_inputs(surface_actuators, propulsor_actuators);

		// create joystick manager
		if (flags.joystick_flag) {
			actuators::ActuatorLimits actuator_limits =
				actuators::pack_actuator_limits(surface_actuators, propulsor_actuators);
			joystick_manager.emplace(actuator_limits);
		}
	}

	devices::JoystickManagerOutput ControlWrapper::poll_joystick()
	{
		// declare for state machine
		devices::JoystickManagerOutput joystick_output{};

		// fetch from joystick
		if (joystick_manager) {
			joystick_output = joystick_manager->step({.u_cmd_t_1 = u_cmd_t_1});
		}

		return joystick_output;
	}

	ControlWrapperOutput ControlWrapper::step(
		const ControlWrapperInput& input)
	{
		control::ControlManager& control_manager = input.aircraft.control_manager;
		actuators::ActuatorManager& actuator_manager = input.aircraft.actuator_manager;
		guidance::GuidanceManager& guidance_manager = input.aircraft.guidance_manager;
		allocator::AllocatorManager& allocator_manager = input.aircraft.allocator_manager;
		actuators::SurfaceActuators& surface_actuators = actuator_manager.surface_actuators;
		actuators::PropulsorActuators& propulsor_actuators = actuator_manager.propulsor_actuators;

		// initialize active mask
		std::array<bool, constants::virtual_input_dim> active_mask;
		std::array<bool, constants::input_dim> actuator_mask;

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
						.surface_actuators = surface_actuators,
						.propulsor_actuators = propulsor_actuators,
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
				u_actual_t_1,
				input.trim_sol.converged ? std::make_optional(input.trim_sol.operating_point.input) : std::nullopt,
				input.context.transient_conditions,
				input.context.autodiff_model));
			u_cmd = ctrl_out.u;
			delta_mu_vec_t_1 = ctrl_out.delta_mu_vec_t_1;
		}

		// apply fixed actuator inputs
		// apply surface actuator dynamics
		// apply propulsor actuator dynamics
		actuators::ActuatorManagerOutput actuator_output = actuator_manager.step({.u_cmd = u_cmd, .dt = constants::dt});
		u_cmd = actuator_output.u_cmd;

		// update prior-step control command
		u_cmd_t_1 = u_cmd;

		actuators::ActuatorInputs_T<double> u_actual = actuator_output.u_actual;

		// update prior-step actual control
		u_actual_t_1 = u_actual;

		return {.setpoint = setpoint, .u_cmd = u_cmd, .u_actual = u_actual};
	}

} // namespace runner
