#pragma once
#include <Eigen/Dense>
#include <tuple>
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/constants/public.hpp"

namespace actuators {

    using ActuatorInputsVector = Eigen::Matrix<double, constants::input_dim, 1>;

    using ActuatorLimitsVector = Eigen::Matrix<double, constants::input_dim, 2>;

    struct FixedActuatorInputs {
        double flap = 0.0;
        double spoiler = 0.0;
    };

    template <typename T>
    struct ActuatorInputs_T {
        T elevator_cmd = T(0.0);
        T aileron_cmd = T(0.0);
        T rudder_cmd = T(0.0);
        T front_propulsor_cmd = T(0.0);
        T left_propulsor_cmd = T(0.0);
        T right_propulsor_cmd = T(0.0);
    };

    struct ActuatorLimits {
        ActuatorInputs_T<double> limit_min;
        ActuatorInputs_T<double> limit_max;
    };

    ActuatorInputs_T<double> pack_actuator_inputs(const ActuatorInputsVector& u);

    ActuatorInputsVector unpack_actuator_inputs(const ActuatorInputs_T<double>& u);

    ActuatorLimits pack_actuator_limits(const ActuatorLimitsVector& limits);

    ActuatorLimitsVector unpack_actuator_limits(const ActuatorLimits& limits);

    ActuatorInputs_T<double> pack_actuator_inputs(const SurfaceActuatorInputs_T<double>& u_surface, const PropulsorActuatorInputs_T<double>& u_propulsor);

    ActuatorInputsVector unpack_actuator_inputs(const SurfaceActuatorInputs_T<double>& u_surface, const PropulsorActuatorInputs_T<double>& u_propulsor);

    ActuatorLimits pack_actuator_limits(const SurfaceActuators& surface_actuators, const PropulsorActuators& propulsor_actuators);

    std::tuple<ActuatorInputsVector, ActuatorInputsVector> unpack_actuator_limits(const SurfaceActuators& surface_actuators, const PropulsorActuators& propulsor_actuators);

	struct Settings {  
        FixedActuatorInputs fixed_actuator_inputs{};
        FixedActuatorInputs get_fixed_actuator_inputs();
	};

    struct ActuatorProperties {
        SurfaceActuators surface_actuators;
        SurfaceActuatorInputs_T<double> step(const SurfaceActuatorInputs_T<double>& u_cmd, double dt);

        PropulsorActuators propulsor_actuators;
        PropulsorActuatorInputs_T<double> step(const PropulsorActuatorInputs_T<double>& u_cmd, double dt);

        Settings settings;
    };

}
