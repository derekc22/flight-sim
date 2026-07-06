#pragma once
#include <Eigen/Dense>
#include "simulation/actuators/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"

namespace operating {

    template <typename T>
    using StateInputVector_T = Eigen::Matrix<T, constants::state_input_dim, 1>;

    struct OperatingConditions {
        atmospheric::StaticAtmosphericState atm;
        atmospheric::Wind windB{ constants::Zero3 };
        bool steady_state;
    };

    struct OperatingPoint {
        dynamics::State_T<double> state;
        actuators::ActuatorInputs_T<double> input;
    };

    template <typename T>
    StateInputVector_T<T> unpack_state_input_T(const dynamics::State_T<T>& x, const actuators::ActuatorInputs_T<T>& u);

    template <typename T>
    dynamics::State_T<T> pack_state_T(const StateInputVector_T<T>& xu);

    template <typename T>
    actuators::ActuatorInputs_T<T> pack_actuator_inputs_T(const StateInputVector_T<T>& xu);
}

#include "simulation/operating/public.tpp"
