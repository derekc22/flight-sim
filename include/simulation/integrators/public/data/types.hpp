#pragma once
#include "simulation/actuators/public/components/collection.hpp"
#include "simulation/aerodynamics/public/manager.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/propulsion/public/manager.hpp"
#include "simulation/structural/public/data/types.hpp"

namespace integrators {

    struct RK4Model {
        const structural::StructuralState& struc_t;
        const aerodynamics::AerodynamicsManager& aerodynamic;
        actuators::PropulsorActuators& propulsor_actuators;
        propulsion::PropulsionManager& propulsion;
    };

    struct RK4Output {
        dynamics::RigidBodyState Xt1;
        dynamics::WrenchSet WB_set;
    };

    template <typename T>
    struct WrenchEvaluation_T {
        dynamics::WrenchSet_T<T> wrench;
        propulsion::PropulsionState_T<T> next_propulsion_state;
    };

    struct WrenchEvaluation {
        dynamics::WrenchSet wrench;
        propulsion::PropulsionState next_propulsion_state;
    };

}
