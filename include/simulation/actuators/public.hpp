#pragma once
#include <Eigen/Dense>
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/constants/public.hpp"

namespace actuators {

    template <typename T>
    using ActuatorInputsVector_T = constants::MatrixX_T<T, constants::input_dim, 1>;

    using ActuatorLimitsVector = constants::MatrixX_T<double, constants::input_dim, 2>;

    struct FixedActuatorInputs {
        double flap = 0.0;
        double spoiler = 0.0;
    };

    template <typename T>
    struct ActuatorInputs_T {
        SurfaceActuatorInputs_T<T> surface_inputs;
        PropulsorActuatorInputs_T<T> propulsor_inputs;
    };

    struct ActuatorLimits {
        ActuatorInputs_T<double> limit_min;
        ActuatorInputs_T<double> limit_max;
    };

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

	struct Settings {  
        FixedActuatorInputs fixed_actuator_inputs{};
        FixedActuatorInputs get_fixed_actuator_inputs();
	};

    struct ActuatorManager {
        SurfaceActuators surface_actuators;
        SurfaceActuatorInputs_T<double> step(const SurfaceActuatorInputs_T<double>& u_cmd, double dt);

        PropulsorActuators propulsor_actuators;
        PropulsorActuatorInputs_T<double> step(const PropulsorActuatorInputs_T<double>& u_cmd, double dt);

        Settings settings;
    };

}

#include "simulation/actuators/public.tpp"
