#pragma once
#include <Eigen/Dense>
#include "simulation/actuators/public/data/types.hpp"
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public/data/types.hpp"

namespace operating {

    template <typename T>
    using StateInputVector_T = constants::MatrixX_T<T, constants::state_input_dim, 1>;

    template <typename T>
    using VirtualStateInputVector_T = constants::MatrixX_T<T, constants::virtual_state_input_dim, 1>;

    struct OperatingConditions {
        atmospheric::StaticAtmosphericState atm;
        atmospheric::Wind windB{ constants::Zero3 };
        atmospheric::Wind windI{ constants::Zero3 };
        bool steady_state;
    };

    template <typename T>
    struct OperatingPoint_T {
        dynamics::State_T<T> state;
        actuators::ActuatorInputs_T<T> input;
    };

    template <typename T>
    struct VirtualOperatingPoint_T {
        dynamics::State_T<T> state;
        dynamics::Wrench_T<T> input;
    };
}
