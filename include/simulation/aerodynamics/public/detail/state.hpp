#pragma once
#include "simulation/aerodynamics/public/data/types.hpp"
#include "simulation/atmospheric/public/data/types.hpp"

namespace frames { struct Frame; }

namespace aerodynamics {

    AerodynamicState compute_aerodynamic_state(const frames::Frame& F, const frames::Frame& R, const atmospheric::Wind& windB);

    template <typename T>
    AerodynamicState_T<T> compute_aerodynamic_state_T(const dynamics::Twist_T<T>& twist, const atmospheric::Wind& windB);

    AerodynamicState compute_aerodynamic_state(const dynamics::RigidBodyState& X, const atmospheric::Wind& windB);

}
#include "simulation/aerodynamics/public/detail/state.tpp"
