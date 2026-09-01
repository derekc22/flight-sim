#pragma once
#include "simulation/actuators/public/components/collection.hpp"
#include "simulation/actuators/public/data/types.hpp"

namespace actuators {

    template <typename T>
    ActuatorInputs_T<T> pack_actuator_inputs_T(const ActuatorInputsVector_T<T>& u);

    template <typename T>
    ActuatorInputsVector_T<T> unpack_actuator_inputs_T(const ActuatorInputs_T<T>& u);

    ActuatorLimits pack_actuator_limits(const ActuatorLimitsVector& limits);

    ActuatorLimitsVector unpack_actuator_limits(const ActuatorLimits& limits);

    template <typename T>
    ActuatorInputs_T<T> pack_actuator_inputs_T(const SurfaceActuatorInputs_T<T>& u_surface, const PropulsorActuatorInputs_T<T>& u_propulsor);

    template <typename T>
    ActuatorInputsVector_T<T> unpack_actuator_inputs_T(const SurfaceActuatorInputs_T<T>& u_surface, const PropulsorActuatorInputs_T<T>& u_propulsor);

    ActuatorLimits pack_actuator_limits(const SurfaceActuators& surface_actuators, const PropulsorActuators& propulsor_actuators);

    ActuatorLimitsVector unpack_actuator_limits(const SurfaceActuators& surface_actuators, const PropulsorActuators& propulsor_actuators);

    ActuatorInputs_T<double> get_neutral_actuator_inputs(const SurfaceActuators& surface_actuators, const PropulsorActuators& propulsor_actuators);

    SurfaceActuatorInputsVector unpack_surface_actuator_inputs(const SurfaceActuatorInputs_T<double>& u_surface);

    PropulsorActuatorInputsVector unpack_propulsor_actuator_inputs(const PropulsorActuatorInputs_T<double>& u_propulsor);

}

#include "simulation/actuators/public/data/helpers.tpp"
