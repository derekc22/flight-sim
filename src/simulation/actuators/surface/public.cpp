#include "simulation/actuators/surface/public.hpp"

namespace actuators {

    SurfaceActuatorInputsVector_T<double> unpack_surface_actuator_inputs(const SurfaceActuatorInputs_T<double>& u_surface) {
        SurfaceActuatorInputsVector_T<double> out;
        out << u_surface.elevator_cmd,
               u_surface.aileron_cmd,
               u_surface.rudder_cmd,
               u_surface.flap_cmd,
               u_surface.spoiler_cmd;
        return out;
    }

}
