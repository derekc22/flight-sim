#pragma once
#include <optional>
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/actuators/actuators.hpp"


namespace propulsion {

    struct PropulsiveWrench : dynamics::Wrench {};

   template <typename T>
    struct PropulsorActuatorInputs_T {
        T front_propulsor_cmd = T(0);
        T left_propulsor_cmd = T(0);
        T right_propulsor_cmd = T(0);
    };

    template <typename T>
    struct PropulsiveWrench_T {
        constants::Vector3_T<T> F = constants::Zero3_T<T>;
        constants::Vector3_T<T> M = constants::Zero3_T<T>;
    };

    template <typename T>
    PropulsiveWrench_T<T> step_propulsive_forces_moments_T(const actuators::PropulsorActuators& propulsor_actuators, const PropulsorActuatorInputs_T<T>& u);

    PropulsiveWrench step_propulsive_forces_moments(const actuators::PropulsorActuators& propulsor_actuators, const control::PropulsorActuatorInputs& u);

}

#include "simulation/propulsion/propulsion.tpp"