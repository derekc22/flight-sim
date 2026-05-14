#pragma once
#include <Eigen/Dense>
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/constants/public.hpp"

namespace actuators {

    template <typename T>
    using ActuatorInputsVector_T = Eigen::Matrix<T, constants::input_dim, 1>;

    template <typename T>
    using ActuatorLimitsVector_T = Eigen::Matrix<T, constants::input_dim, 2>;

    struct FixedActuatorInputs_T {
        double flap = 0.0;
        double spoiler = 0.0;
    };

    template <typename T>
    struct ActuatorInputs_T {
        T elevator_cmd = T(0);
        T aileron_cmd = T(0);
        T rudder_cmd = T(0);
        T front_propulsor_cmd = T(0);
        T left_propulsor_cmd = T(0);
        T right_propulsor_cmd = T(0);
    };

    template <typename T>
    struct ActuatorLimits_T {
        ActuatorInputs_T<T> limit_min;
        ActuatorInputs_T<T> limit_max;
    };

    template <typename T>
    ActuatorInputs_T<T> pack_actuator_inputs_T(const ActuatorInputsVector_T<T>& u);

    template <typename T>
    ActuatorInputsVector_T<T> unpack_actuator_inputs_T(const ActuatorInputs_T<T>& u);

    template <typename T>
    ActuatorLimits_T<T> pack_actuator_limits_T(const ActuatorLimitsVector_T<T>& limits);

    template <typename T>
    ActuatorLimitsVector_T<T> unpack_actuator_limits_T(const ActuatorLimits_T<T>& limits);

    ActuatorInputsVector_T<double> unpack_actuator_inputs(const SurfaceActuatorInputs_T<double>& u_surface, const PropulsorActuatorInputs_T<double>& u_propulsor);

    ActuatorLimits_T<double> pack_actuator_limits(const SurfaceActuators& surface_actuators, const PropulsorActuators& propulsor_actuators);

    struct ActuatorProperties {
        SurfaceActuators surface_actuators;
        SurfaceActuatorInputs_T<double> step(const SurfaceActuatorInputs_T<double>& u_cmd);

        PropulsorActuators propulsor_actuators;
        PropulsorActuatorInputs_T<double> step(const PropulsorActuatorInputs_T<double>& u_cmd);
    };

}

#include "simulation/actuators/public.tpp"
