#include "simulation/actuators/public.hpp"
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"

namespace actuators {

    ActuatorLimits pack_actuator_limits(const ActuatorLimitsVector& limits) {
        const ActuatorInputsVector_T<double> limit_min = limits.col(0);
        const ActuatorInputsVector_T<double> limit_max = limits.col(1);
        return {
            .limit_min = pack_actuator_inputs_T(limit_min),
            .limit_max = pack_actuator_inputs_T(limit_max),
        };
    }

    ActuatorLimitsVector unpack_actuator_limits(const ActuatorLimits& limits) {
        ActuatorLimitsVector out;
        out.col(0) = unpack_actuator_inputs_T(limits.limit_min);
        out.col(1) = unpack_actuator_inputs_T(limits.limit_max);
        return out;
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

    SurfaceActuatorInputs_T<double> ActuatorManager::step(const SurfaceActuatorInputs_T<double>& u_cmd, double dt) {
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

    PropulsorActuatorInputs_T<double> ActuatorManager::step(const PropulsorActuatorInputs_T<double>& u_cmd, double dt) {
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

    ActuatorInputs_T<double> get_neutral_actuator_inputs(const SurfaceActuators& surface_actuators, const PropulsorActuators& propulsor_actuators) {
        const ActuatorLimitsVector actuator_limits = unpack_actuator_limits(surface_actuators, propulsor_actuators);
        const ActuatorInputsVector_T<double> neutral_actuators = ActuatorInputsVector_T<double>::Zero().cwiseMax(actuator_limits.col(0)).cwiseMin(actuator_limits.col(1));
        return pack_actuator_inputs_T(neutral_actuators);
    }

    FixedActuatorInputs Settings::get_fixed_actuator_inputs() {
        return {
            .flap = fixed_actuator_inputs.flap,
            .spoiler = fixed_actuator_inputs.spoiler,
        };
    }

}
