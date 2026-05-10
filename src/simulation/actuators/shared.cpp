#include "simulation/actuators/shared.hpp"
#include "simulation/actuators/propulsor/shared.hpp"
#include "simulation/actuators/surface/shared.hpp"

namespace actuators {

    ActuatorInputsVector_T<double> unpack_actuator_inputs(const SurfaceActuatorInputs_T<double>& u_surface, const PropulsorActuatorInputs_T<double>& u_propulsor) {
        ActuatorInputsVector_T<double> out;
        out << u_surface.elevator_cmd,
               u_surface.aileron_cmd,
               u_surface.rudder_cmd,
               u_propulsor.front_propulsor_cmd,
               u_propulsor.left_propulsor_cmd,
               u_propulsor.right_propulsor_cmd;
        return out;
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

}
