#pragma once
#include <cstddef>
#include <Eigen/Dense>
#include "simulation/actuators/propulsor/shared.hpp"
#include "simulation/actuators/surface/shared.hpp"
#include "simulation/actuators/shared.hpp"
#include "simulation/atmospheric/shared.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/structural/structural.hpp"

namespace trim {

    inline constexpr std::size_t trim_variable_dim = constants::state_dim + constants::input_dim;

    template <typename T>
    using TrimVariablesVector_T = Eigen::Matrix<T, trim_variable_dim, 1>;

    struct TrimConditions {
        atmospheric::StaticAtmosphericState static_atmospheric_state;
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

}
