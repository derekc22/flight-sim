#include <utility> // For std::pair
#include "simulation/actuators/public.hpp"
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"

namespace actuators {

    ActuatorInputs_T<double> pack_actuator_inputs(const SurfaceActuatorInputs_T<double>& u_surface, const PropulsorActuatorInputs_T<double>& u_propulsor) {
        return {
            .elevator_cmd = u_surface.elevator_cmd,
            .aileron_cmd = u_surface.aileron_cmd,
            .rudder_cmd = u_surface.rudder_cmd,
            .front_propulsor_cmd = u_propulsor.front_propulsor_cmd,
            .left_propulsor_cmd = u_propulsor.left_propulsor_cmd,
            .right_propulsor_cmd = u_propulsor.right_propulsor_cmd,
        };
    }

    ActuatorInputsVector_T<double> unpack_actuator_inputs(const SurfaceActuatorInputs_T<double>& u_surface, const PropulsorActuatorInputs_T<double>& u_propulsor) {
        return unpack_actuator_inputs_T(pack_actuator_inputs(u_surface, u_propulsor));
    }

    ActuatorLimits_T<double> pack_actuator_limits(const SurfaceActuators& surface_actuators, const PropulsorActuators& propulsor_actuators) {
        return {
            .limit_min = {
                .elevator_cmd = surface_actuators.elevator.limit_min,
                .aileron_cmd = surface_actuators.aileron.limit_min,
                .rudder_cmd = surface_actuators.rudder.limit_min,
                .front_propulsor_cmd = propulsor_actuators.front_propulsor.limit_min,
                .left_propulsor_cmd = propulsor_actuators.left_propulsor.limit_min,
                .right_propulsor_cmd = propulsor_actuators.right_propulsor.limit_min,
            },
            .limit_max = {
                .elevator_cmd = surface_actuators.elevator.limit_max,
                .aileron_cmd = surface_actuators.aileron.limit_max,
                .rudder_cmd = surface_actuators.rudder.limit_max,
                .front_propulsor_cmd = propulsor_actuators.front_propulsor.limit_max,
                .left_propulsor_cmd = propulsor_actuators.left_propulsor.limit_max,
                .right_propulsor_cmd = propulsor_actuators.right_propulsor.limit_max,
            }
        };
    }

    SurfaceActuatorInputs_T<double> ActuatorProperties::step(const SurfaceActuatorInputs_T<double>& u_cmd, double dt) {
        return {
            .elevator_cmd = surface_actuators.elevator.step(u_cmd.elevator_cmd, surface_actuators.elevator.prev_cmd, dt),
            .aileron_cmd = surface_actuators.aileron.step(u_cmd.aileron_cmd, surface_actuators.aileron.prev_cmd, dt),
            .rudder_cmd = surface_actuators.rudder.step(u_cmd.rudder_cmd, surface_actuators.rudder.prev_cmd, dt),
            .flap_cmd = surface_actuators.flap.step(u_cmd.flap_cmd, surface_actuators.flap.prev_cmd, dt),
            .spoiler_cmd = surface_actuators.spoiler.step(u_cmd.spoiler_cmd, surface_actuators.spoiler.prev_cmd, dt)
        };
    }

    PropulsorActuatorInputs_T<double> ActuatorProperties::step(const PropulsorActuatorInputs_T<double>& u_cmd, double dt) {
        return {
            .front_propulsor_cmd = propulsor_actuators.front_propulsor.step(u_cmd.front_propulsor_cmd, propulsor_actuators.front_propulsor.prev_cmd, dt),
            .left_propulsor_cmd = propulsor_actuators.left_propulsor.step(u_cmd.left_propulsor_cmd, propulsor_actuators.left_propulsor.prev_cmd, dt),
            .right_propulsor_cmd = propulsor_actuators.right_propulsor.step(u_cmd.right_propulsor_cmd, propulsor_actuators.right_propulsor.prev_cmd, dt),
        };
    }

    std::pair<ActuatorInputsVector_T<double>, ActuatorInputsVector_T<double>> unpack_actuator_limits(const SurfaceActuators& surface_actuators, const PropulsorActuators& propulsor_actuators) {
        ActuatorLimits_T<double> max_min_limits = pack_actuator_limits(surface_actuators, propulsor_actuators);
        return { unpack_actuator_inputs_T(max_min_limits.limit_min), unpack_actuator_inputs_T(max_min_limits.limit_max) };
    }

    FixedActuatorInputs Settings::get_fixed_actuator_inputs() {
        return {
            .flap = fixed_actuator_inputs.flap,
            .spoiler = fixed_actuator_inputs.spoiler,
        };
    }

}
