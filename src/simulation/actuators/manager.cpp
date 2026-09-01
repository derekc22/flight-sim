#include "simulation/actuators/public/manager.hpp"

namespace actuators {

    ActuatorManagerOutput ActuatorManager::step(const ActuatorManagerInput& input) {
        ActuatorInputs_T<double> commanded_inputs = input.commanded_inputs;
        const FixedActuatorInputs fixed_inputs = settings.fixed_actuator_inputs;
        commanded_inputs.surface_inputs.flap_cmd = fixed_inputs.flap;
        commanded_inputs.surface_inputs.spoiler_cmd = fixed_inputs.spoiler;

        return {
            .commanded_inputs = commanded_inputs,
            .actual_inputs = {
                .surface_inputs = step(commanded_inputs.surface_inputs, input.dt),
                .propulsor_inputs = step(commanded_inputs.propulsor_inputs, input.dt)
            }
        };
    }

    SurfaceActuatorInputs_T<double> ActuatorManager::step(const SurfaceActuatorInputs_T<double>& u_cmd, double dt) {
        return {
            .elevator_cmd = surface_actuators.elevator.step(
                u_cmd.elevator_cmd,
                dt
            ),
            .aileron_cmd = surface_actuators.aileron.step(
                u_cmd.aileron_cmd,
                dt
            ),
            .rudder_cmd = surface_actuators.rudder.step(
                u_cmd.rudder_cmd,
                dt
            ),
            .flap_cmd = surface_actuators.flap.step(
                u_cmd.flap_cmd,
                dt
            ),
            .spoiler_cmd = surface_actuators.spoiler.step(
                u_cmd.spoiler_cmd,
                dt
            )
        };
    }

    PropulsorActuatorInputs_T<double> ActuatorManager::step(const PropulsorActuatorInputs_T<double>& u_cmd, double dt) {
        return {
            .front_propulsor_cmd = propulsor_actuators.front_propulsor.step(
                u_cmd.front_propulsor_cmd,
                dt
            ),
            .left_propulsor_cmd = propulsor_actuators.left_propulsor.step(
                u_cmd.left_propulsor_cmd,
                dt
            ),
            .right_propulsor_cmd = propulsor_actuators.right_propulsor.step(
                u_cmd.right_propulsor_cmd,
                dt
            ),
        };
    }

}
