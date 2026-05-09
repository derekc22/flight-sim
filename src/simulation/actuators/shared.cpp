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

}
