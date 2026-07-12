#include "simulation/actuators/public.hpp"
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"

namespace actuators {

    ActuatorInputs_T<double> pack_actuator_inputs(const ActuatorInputsVector& u) {
        return {
            .surface_inputs = {
                .elevator_cmd = u(0),
                .aileron_cmd = u(1),
                .rudder_cmd = u(2),
            },
            .propulsor_inputs = {
                .front_propulsor_cmd = u(3),
                .left_propulsor_cmd = u(4),
                .right_propulsor_cmd = u(5),
            }
        };
    }

    ActuatorInputsVector unpack_actuator_inputs(const ActuatorInputs_T<double>& u) {
        const SurfaceActuatorInputs_T<double>& surface_inputs = u.surface_inputs;
        const PropulsorActuatorInputs_T<double>& propulsor_inputs = u.propulsor_inputs;
        ActuatorInputsVector out;
        out << surface_inputs.elevator_cmd,
               surface_inputs.aileron_cmd,
               surface_inputs.rudder_cmd,
               propulsor_inputs.front_propulsor_cmd,
               propulsor_inputs.left_propulsor_cmd,
               propulsor_inputs.right_propulsor_cmd;
        return out;
    }

    ActuatorLimits pack_actuator_limits(const ActuatorLimitsVector& limits) {
        const ActuatorInputsVector limit_min = limits.col(0);
        const ActuatorInputsVector limit_max = limits.col(1);
        return {
            .limit_min = pack_actuator_inputs(limit_min),
            .limit_max = pack_actuator_inputs(limit_max),
        };
    }

    ActuatorLimitsVector unpack_actuator_limits(const ActuatorLimits& limits) {
        ActuatorLimitsVector out;
        out.col(0) = unpack_actuator_inputs(limits.limit_min);
        out.col(1) = unpack_actuator_inputs(limits.limit_max);
        return out;
    }

    ActuatorInputs_T<double> pack_actuator_inputs(const SurfaceActuatorInputs_T<double>& u_surface, const PropulsorActuatorInputs_T<double>& u_propulsor) {
        return {
            .surface_inputs = u_surface,
            .propulsor_inputs = u_propulsor
        };
    }

    ActuatorInputsVector unpack_actuator_inputs(const SurfaceActuatorInputs_T<double>& u_surface, const PropulsorActuatorInputs_T<double>& u_propulsor) {
        return unpack_actuator_inputs(pack_actuator_inputs(u_surface, u_propulsor));
    }

    ActuatorLimits pack_actuator_limits(const SurfaceActuators& surface_actuators, const PropulsorActuators& propulsor_actuators) {
        return {
            .limit_min = {
                .surface_inputs = {
                    .elevator_cmd = surface_actuators.elevator.limit_min,
                    .aileron_cmd = surface_actuators.aileron.limit_min,
                    .rudder_cmd = surface_actuators.rudder.limit_min,
                },
                .propulsor_inputs = {
                    .front_propulsor_cmd = propulsor_actuators.front_propulsor.limit_min,
                    .left_propulsor_cmd = propulsor_actuators.left_propulsor.limit_min,
                    .right_propulsor_cmd = propulsor_actuators.right_propulsor.limit_min,
                }
            },
            .limit_max = {
                .surface_inputs = {
                    .elevator_cmd = surface_actuators.elevator.limit_max,
                    .aileron_cmd = surface_actuators.aileron.limit_max,
                    .rudder_cmd = surface_actuators.rudder.limit_max,
                },
                .propulsor_inputs = {
                    .front_propulsor_cmd = propulsor_actuators.front_propulsor.limit_max,
                    .left_propulsor_cmd = propulsor_actuators.left_propulsor.limit_max,
                    .right_propulsor_cmd = propulsor_actuators.right_propulsor.limit_max,
                }
            }
        };
    }

    SurfaceActuatorInputs_T<double> ActuatorProperties::step(const SurfaceActuatorInputs_T<double>& u_cmd, double dt) {
        return {
            .elevator_cmd = surface_actuators.elevator.step(
                u_cmd.elevator_cmd, 
                surface_actuators.elevator.prev_cmd, 
                dt
            ),
            .aileron_cmd = surface_actuators.aileron.step(
                u_cmd.aileron_cmd, 
                surface_actuators.aileron.prev_cmd, 
                dt
            ),
            .rudder_cmd = surface_actuators.rudder.step(
                u_cmd.rudder_cmd, 
                surface_actuators.rudder.prev_cmd, 
                dt
            ),
            .flap_cmd = surface_actuators.flap.step(
                u_cmd.flap_cmd, 
                surface_actuators.flap.prev_cmd, 
                dt
            ),
            .spoiler_cmd = surface_actuators.spoiler.step(
                u_cmd.spoiler_cmd, 
                surface_actuators.spoiler.prev_cmd, 
                dt
            )
        };
    }

    PropulsorActuatorInputs_T<double> ActuatorProperties::step(const PropulsorActuatorInputs_T<double>& u_cmd, double dt) {
        return {
            .front_propulsor_cmd = propulsor_actuators.front_propulsor.step(
                u_cmd.front_propulsor_cmd, 
                propulsor_actuators.front_propulsor.prev_cmd, 
                dt
            ),
            .left_propulsor_cmd = propulsor_actuators.left_propulsor.step(
                u_cmd.left_propulsor_cmd, 
                propulsor_actuators.left_propulsor.prev_cmd, 
                dt
            ),
            .right_propulsor_cmd = propulsor_actuators.right_propulsor.step(
                u_cmd.right_propulsor_cmd, 
                propulsor_actuators.right_propulsor.prev_cmd, 
                dt
            ),
        };
    }

    ActuatorLimitsVector unpack_actuator_limits(const SurfaceActuators& surface_actuators, const PropulsorActuators& propulsor_actuators) {
        return unpack_actuator_limits(pack_actuator_limits(surface_actuators, propulsor_actuators));
    }

    FixedActuatorInputs Settings::get_fixed_actuator_inputs() {
        return {
            .flap = fixed_actuator_inputs.flap,
            .spoiler = fixed_actuator_inputs.spoiler,
        };
    }

}
