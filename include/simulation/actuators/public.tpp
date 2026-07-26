#pragma once

namespace actuators {

    template <typename T>
    ActuatorInputs_T<T> pack_actuator_inputs_T(const ActuatorInputsVector_T<T>& u) {
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

    template <typename T>
    ActuatorInputsVector_T<T> unpack_actuator_inputs_T(const ActuatorInputs_T<T>& u) {
        const SurfaceActuatorInputs_T<T>& surface_inputs = u.surface_inputs;
        const PropulsorActuatorInputs_T<T>& propulsor_inputs = u.propulsor_inputs;
        ActuatorInputsVector_T<T> out;
        out << surface_inputs.elevator_cmd,
               surface_inputs.aileron_cmd,
               surface_inputs.rudder_cmd,
               propulsor_inputs.front_propulsor_cmd,
               propulsor_inputs.left_propulsor_cmd,
               propulsor_inputs.right_propulsor_cmd;
        return out;
    }

    template <typename T>
    ActuatorInputs_T<T> pack_actuator_inputs_T(const SurfaceActuatorInputs_T<T>& u_surface, const PropulsorActuatorInputs_T<T>& u_propulsor) {
        return {
            .surface_inputs = u_surface,
            .propulsor_inputs = u_propulsor
        };
    }

    template <typename T>
    ActuatorInputsVector_T<T> unpack_actuator_inputs_T(const SurfaceActuatorInputs_T<T>& u_surface, const PropulsorActuatorInputs_T<T>& u_propulsor) {
        return unpack_actuator_inputs_T(pack_actuator_inputs_T(u_surface, u_propulsor));
    }

}
