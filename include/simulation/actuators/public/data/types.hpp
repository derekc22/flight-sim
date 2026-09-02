#pragma once
#include <Eigen/Dense>
#include <optional>
#include <string>
#include <vector>
#include "simulation/constants/public.hpp"

namespace actuators {

    template <typename T>
    using ActuatorInputsVector_T = constants::MatrixX_T<T, constants::input_dim, 1>;

    using ActuatorLimitsVector = constants::MatrixX_T<double, constants::input_dim, 2>;

    struct FixedActuatorInputs {
        double flap = 0.0;
        double spoiler = 0.0;
    };

	struct Settings {
        FixedActuatorInputs fixed_actuator_inputs{};
	};

    template <typename T>
    struct SurfaceActuatorInputs_T {
        T elevator_cmd = T(0.0);
        T aileron_cmd = T(0.0);
        T rudder_cmd = T(0.0);
        T flap_cmd = T(0.0);
        T spoiler_cmd = T(0.0);
    };

    template <typename T>
    struct PropulsorActuatorInputs_T {
        T front_propulsor_cmd = T(0.0);
        T left_propulsor_cmd = T(0.0);
        T right_propulsor_cmd = T(0.0);
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

    struct ActuatorManagerInput {
        ActuatorInputs_T<double> u_cmd;
        double dt;
    };

    struct ActuatorManagerOutput {
        ActuatorInputs_T<double> u_cmd;
        ActuatorInputs_T<double> u_actual;
    };

    struct PropellerAssembly {
        std::vector<std::string> geometry_ids;
        double spin_sign;
        double thrust_coeff;
        double torque_coeff;
        double diameter;
        double spin_inertia;
    };

    using SurfaceActuatorInputsVector = constants::MatrixX_T<double, constants::full_surface_input_dim, 1>;

    using PropulsorActuatorInputsVector = constants::MatrixX_T<double, constants::full_propulsor_input_dim, 1>;

}
