#pragma once
#include <cstddef>
#include <Eigen/Dense>
#include "simulation/constants/constants.hpp"


namespace types {

    template <typename T>
    using ActuatorInputsVector_T = Eigen::Matrix<T, constants::input_dim, 1>;

    template <typename T>
    using FullSurfaceActuatorInputsVector_T = Eigen::Matrix<T, constants::full_surface_input_dim, 1>;

    template <typename T>
    using FullPropulsorActuatorInputsVector_T = Eigen::Matrix<T, constants::full_propulsor_input_dim, 1>;

    template <typename T>
    using StateVector_T = Eigen::Matrix<T, constants::state_dim, 1>;

    template <typename T>
    using StateDotVector_T = Eigen::Matrix<T, constants::state_dim, 1>;

    struct FixedActuatorInputs_T {
        double flap = 0.0;
        double spoiler = 0.0;
    };

    template <typename T>
    struct State_T {
        T vx = T(0);
        T vy = T(0);
        T vz = T(0);

        T p = T(0);
        T q = T(0);
        T r = T(0);

        T phi = T(0);
        T theta = T(0);
    };

    template <typename T>
    struct StateDot_T {
        T vx_dot = T(0);
        T vy_dot = T(0);
        T vz_dot = T(0);

        T p_dot = T(0);
        T q_dot = T(0);
        T r_dot = T(0);

        T phi_dot = T(0);
        T theta_dot = T(0);
    };

    template <typename T>
    struct ActuatorInputs_T {
        T elevator_cmd = T(0);
        T aileron_cmd = T(0);
        T rudder_cmd = T(0);
        T front_propulsor_cmd = T(0);
        T left_propulsor_cmd = T(0);
        T right_propulsor_cmd = T(0);
    };

    // struct SurfaceActuatorInputs {
    //     double elevator_cmd = 0.0;  // [rad]
    //     double aileron_cmd = 0.0;   // [rad]
    //     double rudder_cmd = 0.0;    // [rad]
    //     double flap_cmd = 0.0;      // [rad]
    //     double spoiler_cmd = 0.0;   // [rad]
    // };

    // struct PropulsorActuatorInputs {
    //     double front_propulsor_cmd = 0.0;  // [N]
    //     double left_propulsor_cmd = 0.0;   // [N]
    //     double right_propulsor_cmd = 0.0;  // [N]
    // };

    template <typename T>
    struct SurfaceActuatorInputs_T {
        T elevator_cmd = T(0);
        T aileron_cmd = T(0);
        T rudder_cmd = T(0);
        T flap_cmd = T(0);
        T spoiler_cmd = T(0);
    };

   template <typename T>
    struct PropulsorActuatorInputs_T {
        T front_propulsor_cmd = T(0);
        T left_propulsor_cmd = T(0);
        T right_propulsor_cmd = T(0);
    };



    template <typename T>
    ActuatorInputsVector_T<T> unpack_actuator_inputs_T(const ActuatorInputs_T<T>& u);

    template <typename T>
    StateVector_T<T> unpack_state_T(const State_T<T>& x);

    template <typename T>
    StateDotVector_T<T> unpack_state_dot_T(const StateDot_T<T>& x_dot);



    types::FullSurfaceActuatorInputsVector_T<double> unpack_full_surface_actuator_inputs(const SurfaceActuatorInputs_T<double>& u_surface);

    types::FullPropulsorActuatorInputsVector_T<double> unpack_full_propulsor_actuator_inputs(const PropulsorActuatorInputs_T<double>& u_propulsor);

    types::ActuatorInputsVector_T<double> unpack_actuator_inputs(const SurfaceActuatorInputs_T<double>& u_surface, const PropulsorActuatorInputs_T<double>& u_propulsor);

}

#include "simulation/types/types.tpp"
