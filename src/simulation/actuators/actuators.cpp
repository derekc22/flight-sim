#include "simulation/actuators/actuators.hpp"
#include "simulation/actuators/propulsor/shared.hpp"
#include "simulation/actuators/surface/shared.hpp"
#include "simulation/actuators/shared.hpp"

namespace actuators {

    SurfaceActuatorInputs_T<double> ActuatorProperties::step(const SurfaceActuatorInputs_T<double>& u_cmd) {
        return {
            .elevator_cmd = surface_actuators.elevator._step(u_cmd.elevator_cmd, surface_actuators.elevator.prev_cmd),
            .aileron_cmd = surface_actuators.aileron._step(u_cmd.aileron_cmd, surface_actuators.aileron.prev_cmd),
            .rudder_cmd = surface_actuators.rudder._step(u_cmd.rudder_cmd, surface_actuators.rudder.prev_cmd),
            .flap_cmd = surface_actuators.flap._step(u_cmd.flap_cmd, surface_actuators.flap.prev_cmd),
            .spoiler_cmd = surface_actuators.spoiler._step(u_cmd.spoiler_cmd, surface_actuators.spoiler.prev_cmd)
        };
    }

    PropulsorActuatorInputs_T<double> ActuatorProperties::step(const PropulsorActuatorInputs_T<double>& u_cmd) {
        return {
            .front_propulsor_cmd = propulsor_actuators.front_propulsor._step(u_cmd.front_propulsor_cmd, propulsor_actuators.front_propulsor.prev_cmd),
            .left_propulsor_cmd = propulsor_actuators.left_propulsor._step(u_cmd.left_propulsor_cmd, propulsor_actuators.left_propulsor.prev_cmd),
            .right_propulsor_cmd = propulsor_actuators.right_propulsor._step(u_cmd.right_propulsor_cmd, propulsor_actuators.right_propulsor.prev_cmd),
        };
    }

}
