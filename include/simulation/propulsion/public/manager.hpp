#pragma once
#include "simulation/propulsion/public/data/types.hpp"

namespace propulsion {

    struct PropulsionManager {
        PropulsionState prev_state;

        PropulsionManagerOutput step(const PropulsionManagerInput& input);
        void commit(const PropulsionState& state_t);

        template <typename T>
        PropulsionManagerOutput_T<T> step_T(const PropulsionManagerInput_T<T>& input);
    };

}

#include "simulation/propulsion/public/manager.tpp"
