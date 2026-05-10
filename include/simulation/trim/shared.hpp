#pragma once
#include <cstddef>
#include <Eigen/Dense>
#include "simulation/actuators/propulsor/shared.hpp"
#include "simulation/actuators/surface/shared.hpp"
#include "simulation/actuators/shared.hpp"
#include "simulation/atmospheric/shared.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/propulsion/shared.hpp"
#include "simulation/structural/structural.hpp"

namespace aerodynamics { struct AerodynamicProperties; } // forward declare

namespace trim {

    inline constexpr std::size_t trim_variable_dim = constants::state_dim + constants::input_dim;

    template <typename T>
    using TrimVariablesVector_T = Eigen::Matrix<T, trim_variable_dim, 1>;

    struct TrimConditions {
        atmospheric::StaticAtmosphericState static_atm_state;
        atmospheric::Wind windB{ constants::Zero3 };
    };

    template <typename T>
    struct TrimResidual {
        T vx_dot = T(0);
        T vy_dot = T(0);
        T vz_dot = T(0);
        T p_dot = T(0);
        T q_dot = T(0);
        T r_dot = T(0);
        T phi_dot = T(0);
        T theta_dot = T(0);
        T beta_err = T(0);
        T phi_err = T(0);
        T theta_err = T(0);
        T vx_err = T(0);
        T vz_err = T(0);
        T psi_dot_err = T(0);
    };

    struct TrimSolution {
        dynamics::State_T<double> state;
        actuators::ActuatorInputs_T<double> input;
        TrimConditions conditions;
        dynamics::Wrench wrench{};
        TrimResidual<double> residual;
        TrimResidual<double> weighted_residual;
        TrimVariablesVector_T<double> variables = TrimVariablesVector_T<double>::Zero();
        bool attempted = false;
        bool converged = false;
        std::size_t iterations = 0;
        double weighted_residual_norm_2 = 0.0;
        double weighted_residual_norm_inf = 0.0;
    };

    struct TrimModel {
        const structural::StructuralProperties& structural;
        const aerodynamics::AerodynamicProperties& aerodynamic;
        const actuators::PropulsorActuators& propulsor_actuators;
        actuators::ActuatorLimits_T<double> actuator_limits;
        actuators::FixedActuatorInputs_T fixed_actuator_inputs{};
    };

    template <typename T>
    dynamics::StateDot_T<T> compute_trim_state_dot_T(const dynamics::State_T<T>& x, const actuators::ActuatorInputs_T<T>& u, const TrimModel& model, const TrimConditions& conditions);

    template <typename T>
    TrimVariablesVector_T<T> unpack_trim_variables_T(const dynamics::State_T<T>& x, const actuators::ActuatorInputs_T<T>& u);

    template <typename T>
    dynamics::State_T<T> pack_trim_state_T(const TrimVariablesVector_T<T>& z);

    template <typename T>
    actuators::ActuatorInputs_T<T> pack_trim_actuator_inputs_T(const TrimVariablesVector_T<T>& z);

    dynamics::StateVector_T<double> unpack_rigid_body_state(const dynamics::RigidBodyState& xN_t);

}

#include "simulation/trim/shared.tpp"
