#include "simulation/types/types.hpp"

namespace types {

    FullSurfaceActuatorInputsVector_T<double> unpack_full_surface_actuator_inputs(const SurfaceActuatorInputs_T<double>& u_surface) {
        FullSurfaceActuatorInputsVector_T<double> out;
        out << u_surface.elevator_cmd, 
               u_surface.aileron_cmd, 
               u_surface.rudder_cmd, 
               u_surface.flap_cmd,
               u_surface.spoiler_cmd;
        return out;
    }

    FullPropulsorActuatorInputsVector_T<double> unpack_full_propulsor_actuator_inputs(const PropulsorActuatorInputs_T<double>& u_propulsor) {
        FullPropulsorActuatorInputsVector_T<double> out;
        out << u_propulsor.front_propulsor_cmd, 
               u_propulsor.left_propulsor_cmd, 
               u_propulsor.right_propulsor_cmd;
        return out;
    }

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