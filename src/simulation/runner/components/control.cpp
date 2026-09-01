#include <utility>
#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/actuators/public/manager.hpp"
#include "simulation/allocator/public/data/helpers.hpp"
#include "simulation/allocator/public/manager.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/control/public/manager.hpp"
#include "simulation/guidance/public/manager.hpp"
#include "simulation/runner/public/components/control.hpp"
#include "simulation/runner/public/components/scheduler.hpp"
#include "simulation/util/public.hpp"
#include "simulation/vehicles/public/aircraft.hpp"

namespace runner {

    Control::Control(const JSONFlags& flags, const actuators::SurfaceActuators& surface_actuators, const actuators::PropulsorActuators& propulsor_actuators) :
        // create state machine
        fsm_manager({ .trim_enabled = flags.trim_flag, .control_enabled = flags.control_flag, .joystick_enabled = flags.joystick_flag })
    {
        // set u_actual_t_1 to match actuators' neutral initialization
        u_actual_t_1 = actuators::get_neutral_actuator_inputs(
            surface_actuators,
            propulsor_actuators
        );

        // create joystick manager
        if (flags.joystick_flag) {
            actuators::ActuatorLimits actuator_limits = actuators::pack_actuator_limits(
                surface_actuators,
                propulsor_actuators
            );
            joystick_manager.emplace(actuator_limits);
        }
    }

    ControlOutput Control::step(const ControlInput& input) {
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

        // declare for state machine
        bool mode_toggled = false;
        devices::JoystickOutput joystick_output;

        // fetch from joystick
        if (joystick_manager) {
            joystick_output = joystick_manager->step(u_cmd_t_1);
            mode_toggled = joystick_output.mode_toggled;
        }

        // step state machine
        fsm::FiniteState current_mode = fsm_manager.step({ .mode_toggled = mode_toggled }).current_mode;

        if (current_mode == fsm::FiniteState::Manual) {
            u_cmd = joystick_output.u_cmd;
        }

        // no need to rate-limit as the trim command is fixed
        else if (current_mode == fsm::FiniteState::AutopilotTrim) {
            mu_cmd = {};
            util::fill_arr(active_mask, 0, 6, true);
            util::fill_arr(actuator_mask, 0, 6, true);
        }

        else if (current_mode == fsm::FiniteState::Autopilot) {
            if (input.scheduler.guidance_tick >= constants::hz) {
                setpoint = guidance_manager.step(input.scheduler.guidance_tf);
                setpoint_t_1 = setpoint;

                input.scheduler.guidance_tick -= constants::hz;
            }
            else setpoint = setpoint_t_1; // perform ZOH

            if (input.scheduler.control_tick >= constants::hz) {
                double control_dt = input.scheduler.control_elapsed_ticks * constants::dt;

                control::ControlComponentInputs controller_inputs = control_manager.build_component_inputs(
                    input.context.Zt,
                    input.trim_solution, input.linearization,
                    surface_actuators, propulsor_actuators, 
                    setpoint,
                    delta_mu_vec_t_1
                );

                control::ControlManagerOutput virtual_ctrl_out = control_manager.step(controller_inputs, control_dt);
                mu_cmd = virtual_ctrl_out.virtual_control;
                active_mask = virtual_ctrl_out.active_mask;
                actuator_mask = virtual_ctrl_out.actuator_mask;

                mu_cmd_t_1 = mu_cmd;
                active_mask_t_1 = active_mask;
                actuator_mask_t_1 = actuator_mask;

                input.scheduler.control_tick -= constants::hz;
                input.scheduler.control_elapsed_ticks = 0;
            }
            else {
                mu_cmd = mu_cmd_t_1; // perform ZOH
                active_mask = active_mask_t_1;
                actuator_mask = actuator_mask_t_1;
            }
        }

        // step control allocator
        if (current_mode == fsm::FiniteState::AutopilotTrim || current_mode == fsm::FiniteState::Autopilot) {
            allocator::AllocatorManagerOutput allocator_output = allocator_manager.step(
                allocator::build_allocator_input(
                    mu_cmd,
                    active_mask,
                    actuator_mask,
                    input.context.Zt, u_actual_t_1,
                    input.trim_solution.converged ? std::make_optional(input.trim_solution.operating_point.input) : std::nullopt,
                    input.context.transient_conditions,
                    input.context.autodiff_model
                )
            );
            u_cmd = allocator_output.actuator_command;
            delta_mu_vec_t_1 = allocator_output.control_residual;
        }

        // apply fixed actuator inputs
        actuators::FixedActuatorInputs fixed_inputs = actuator_manager.settings.fixed_actuator_inputs;
        u_cmd.surface_inputs.flap_cmd = fixed_inputs.flap;
        u_cmd.surface_inputs.spoiler_cmd = fixed_inputs.spoiler;

        // update prior-step control command
        u_cmd_t_1 = u_cmd;

        actuators::ActuatorInputs_T<double> u_actual;

        // apply surface actuator dynamics
        u_actual.surface_inputs = actuator_manager.step(u_cmd.surface_inputs, constants::dt);

        // apply propulsor actuator dynamics
        u_actual.propulsor_inputs = actuator_manager.step(u_cmd.propulsor_inputs, constants::dt);

        // update prior-step actual control
        u_actual_t_1 = u_actual;

        return {
            .setpoint = setpoint,
            .commanded_inputs = u_cmd,
            .actual_inputs = u_actual,
            .current_mode = current_mode
        };
    }

}
