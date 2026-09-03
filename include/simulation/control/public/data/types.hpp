#pragma once
#include <array>
#include <functional>
#include <optional>
#include <Eigen/Dense>
#include "simulation/actuators/public/components/collection.hpp"
#include "simulation/actuators/public/data/types.hpp"
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/guidance/public/data/types.hpp"
#include "simulation/linearization/public/data/types.hpp"
#include "simulation/trim/public/data/types.hpp"

namespace control {

    enum class ControllerType {
        None,
        // Attitude
        AttitudePID,
        DamperPID,
        // Velocity
        VelocityPID,
        // Linear Quadratic
        LinearQuadraticRegulator,
        LinearQuadraticTracker,
        LinearQuadraticIntegrator,
        // Nonlinear
        FeedbackLinearization,
        NonlinearDynamicInversion,
        IncrementalNonlinearDynamicInversion
    };

    struct AttitudePIDParameters {
        double Kp_roll = 0.0;
        double Ki_roll = 0.0;
        double Kd_roll = 0.0;

        double Kp_pitch = 0.0;
        double Ki_pitch = 0.0;
        double Kd_pitch = 0.0;

        double Kp_yaw = 0.0;
        double Ki_yaw = 0.0;
        double Kd_yaw = 0.0;

        double tau = 0.0;
    };

    struct DamperPIDParameters : AttitudePIDParameters {};

    struct VelocityPIDParameters {
        double Kp;
        double Ki;
        double Kd;
        double tau;
    };

    struct LinearQuadraticRegulatorParameters {
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
        std::optional<Eigen::MatrixXd> K;
    };

    struct LinearQuadraticIntegratorParameters : LinearQuadraticRegulatorParameters {
        Eigen::MatrixXd Qi;
    };

    using ControlOutput = actuators::ActuatorInputs_T<double>;

	using VirtualControlOutput = dynamics::Wrench;

	template <typename T>
	using VirtualControlOutputVector_T = dynamics::WrenchVector_T<T>;

	template <typename T>
    using VirtualControlOutput_T = dynamics::Wrench_T<T>;

    struct AttitudeControlInput {
        const dynamics::RigidBodyState& Zt;
        guidance::AttitudeSetpoint setpoint;
        dynamics::WrenchVector_T<double> delta_mu_vec_t_1;
    };

    struct VelocityControlInput {
        const dynamics::RigidBodyState& Zt;
        guidance::VelocitySetpoint setpoint;
        dynamics::WrenchVector_T<double> delta_mu_vec_t_1;
    };

    struct FullStateControlInput {
        const dynamics::RigidBodyState& Zt;
        const linearization::VirtualLocalLinearization& virtual_linearization;
        const dynamics::State_T<double>& Z_sol_trim;
        const actuators::SurfaceActuators& surface_actuators;
        const actuators::PropulsorActuators& propulsor_actuators;
        guidance::GuidanceSetpoint setpoint;
        dynamics::WrenchVector_T<double> delta_mu_vec_t_1;
    };

    struct ControlComponentInputs {
        std::optional<AttitudeControlInput> attitude_input;
        std::optional<VelocityControlInput> velocity_input;
        std::optional<FullStateControlInput> full_state_input;
    };

    using AttitudeControlImplementation = std::function<VirtualControlOutput_T<double>(const AttitudeControlInput&, double dt)>;

    using VelocityControlImplementation = std::function<VirtualControlOutput_T<double>(const VelocityControlInput&, double dt)>;

    using FullStateControlImplementation = std::function<VirtualControlOutput_T<double>(const FullStateControlInput&, double dt)>;

    struct ControlComponentOutput {
        VirtualControlOutput_T<double> mu;
        std::array<bool, constants::virtual_input_dim> active_mask;
        std::array<bool, constants::input_dim> actuator_mask;
    };

    struct ControlManagerInput {
        const dynamics::RigidBodyState& Zt;
        const trim::TrimSolution& trim_sol;
        const linearization::VirtualLocalLinearization& virtual_lin_sol;
        const actuators::SurfaceActuators& surface_actuators;
        const actuators::PropulsorActuators& propulsor_actuators;
        const guidance::GuidanceSetpoint& setpoint;
        dynamics::WrenchVector_T<double> delta_mu_vec_t_1;
        double dt;
    };

    struct ControlManagerOutput {
        VirtualControlOutput mu;
        std::array<bool, constants::virtual_input_dim> active_mask;
        std::array<bool, constants::input_dim> actuator_mask;
    };

}
