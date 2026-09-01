#pragma once
#include <optional>
#include "simulation/actuators/public/components/collection.hpp"
#include "simulation/actuators/public/data/types.hpp"
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/dynamics/public/data/types.hpp"

namespace propulsion {

    template <typename T>
    struct PropellerOmegaState_T {
        T omega = T(0.0);
        T omega_dot = T(0.0);
    };

    template <typename T>
    struct PropellerOmegaStateSet_T {
        PropellerOmegaState_T<T> front_propulsor{};
        PropellerOmegaState_T<T> left_propulsor{};
        PropellerOmegaState_T<T> right_propulsor{};
    };

    template <typename T>
    struct PropulsionState_T {
        std::optional<T> front_propulsor_omega;
        std::optional<T> left_propulsor_omega;
        std::optional<T> right_propulsor_omega;
    };

    using PropulsionState = PropulsionState_T<double>;

    template <typename T>
    struct PropulsionManagerInput_T {
        const actuators::PropulsorActuators& propulsors;
        const constants::Vector3_T<T>& center_of_gravity;
        const dynamics::Twist_T<T>& vehicle_twist;
        const atmospheric::StaticAtmosphericState& atmosphere;
        const actuators::PropulsorActuatorInputs_T<T>& actuator_inputs;
        T dt;
        bool steady_state;
    };

    using PropulsionManagerInput = PropulsionManagerInput_T<double>;

    template <typename T>
    struct PropulsionManagerOutput_T {
        dynamics::Wrench_T<T> propulsive_wrench;
        PropulsionState_T<T> next_state;
    };

    using PropulsionManagerOutput = PropulsionManagerOutput_T<double>;

}
