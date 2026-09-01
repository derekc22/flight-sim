#pragma once
#include "simulation/dynamics/public/data/types.hpp"

namespace dynamics {

    template <typename T>
    StateVector_T<T> unpack_state_T(const State_T<T>& x);

    template <typename T>
    State_T<T> pack_state_T(const StateVector_T<T>& x);

    template <typename T>
    StateDotVector_T<T> unpack_state_dot_T(const StateDot_T<T>& x_dot);

    template <typename T>
    WrenchVector_T<T> unpack_wrench_T(const Wrench_T<T>& wrench);

    template <typename T>
    Wrench_T<T> pack_wrench_T(const WrenchVector_T<T>& wrench);

    WrenchVector_T<double> unpack_wrench(const Wrench& wrench);

    Wrench pack_wrench(const WrenchVector_T<double>& wrench);

    Wrench pack_wrench(const Wrench_T<double>& wrench);

    template <typename T>
    Twist_T<T> build_twist_from_state_T(const State_T<T>& x);

    State_T<double> pack_state(const RigidBodyState& Xt);

    StateVector_T<double> unpack_state(const RigidBodyState& Xt);

}
#include "simulation/dynamics/public/data/helpers.tpp"
