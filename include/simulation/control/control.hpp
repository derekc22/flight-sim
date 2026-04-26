#pragma once
#include <Eigen/Dense>
#include <functional>
#include <optional>
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/actuators/actuators.hpp"
#include "simulation/trim/trim.hpp"
#include "simulation/linearization/linearization.hpp"
#include "simulation/control/pid.hpp"
#include "simulation/control/lqr.hpp"

namespace control {

    struct AxisControlSetpoint {
        dynamics::EulerAngles eulIB;
        dynamics::AngularVelocity wB_BI;
    };

    struct VelocityControlSetpoint {
        dynamics::LinearVelocity vB_BI;
    };

    struct FullStateControlSetpoint {
        dynamics::LinearVelocity vB_BI;
        dynamics::AngularVelocity wB_BI;
        dynamics::EulerAngles eulIB;
    };

    struct SurfaceActuatorInputs {
        double elevator_cmd = 0.0;  // [rad]
        double aileron_cmd = 0.0;   // [rad]
        double rudder_cmd = 0.0;    // [rad]
        double flap_cmd = 0.0;      // [rad]
        double spoiler_cmd = 0.0;   // [rad]
    };

    struct PropulsorActuatorInputs {
        double front_propulsor_cmd = 0.0;  // [N]
        double left_propulsor_cmd = 0.0;   // [N]
        double right_propulsor_cmd = 0.0;  // [N]
    };

    struct ControlInputs {
        SurfaceActuatorInputs surface_inputs;
        PropulsorActuatorInputs propulsor_inputs;
    };

    struct ControlLawGains {
        double Kp;
        double Kd;
        double Ki;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
    };

    struct ControlLawParameters {
        ControlLawGains gains;
        double tau;
    };

    struct PIDControlLawInput {
        double meas;
        double meas_des;

        std::optional<double> meas_dot;

        double limit_max;
        double limit_min;
    };

    struct FullStateControlLawInput {
        const linearization::TrimStateJacobian& A;
        const linearization::TrimInputJacobian& B;
        trim::TrimStateVector_T<double> meas;
        trim::TrimStateVector_T<double> meas_des;
        const actuators::SurfaceActuators& surface_actuators;
        const actuators::PropulsorActuators& propulsor_actuators;
    };

    template <typename ControlLawCommand>
    using AxisControlLaw = std::function<ControlLawCommand(const PIDControlLawInput&)>;

    template <typename ControlLawCommand>
    using VelocityControlLaw = std::function<ControlLawCommand(const PIDControlLawInput&)>;

    template <typename ControlLawCommand>
    using FullStateControlLaw = std::function<ControlLawCommand(const FullStateControlLawInput&)>;

    // struct ControlType {
    //     enum class Longitudinal { None, PitchDamper, PitchPIDController };
    //     enum class Lateral { None, RollDamper, RollPIDController };
    //     enum class Vertical { None, YawDamper, YawPIDController };
    //     enum class FullState { None, LinearQuadraticRegulator, LinearQuadraticTracker };
    // };

    enum class ControlType {
        None,
        PitchDamper,
        PitchPIDController,
        RollDamper,
        RollPIDController,
        YawDamper,
        YawPIDController,
        LinearQuadraticRegulator,
        LinearQuadraticTracker,
        LinearQuadraticIntegrator,
        VelocityPIDController
    };

    struct ControlProperties {

        AxisControlLaw<double> longitudinal_controller;
        AxisControlLaw<double> lateral_controller;
        AxisControlLaw<double> vertical_controller;
        VelocityControlLaw<double> velocity_controller;
        FullStateControlLaw<Eigen::VectorXd> full_state_controller;

        ControlType longitudinal_control_type = ControlType::None;
        ControlType lateral_control_type = ControlType::None;
        ControlType vertical_control_type = ControlType::None;
        ControlType velocity_control_type = ControlType::None;
        ControlType full_state_control_type = ControlType::None;

        AxisControlSetpoint axis_setpoint;
        VelocityControlSetpoint velocity_setpoint;
        FullStateControlSetpoint full_state_setpoint;
        
        PIDControlLawInput make_axis_control_input(
            const dynamics::RigidBodyState& xN_meas_t, 
            const actuators::SurfaceActuator& surface_actuator, 
            ControlType control_type
        );

        PIDControlLawInput make_velocity_control_input(
            const dynamics::RigidBodyState& xN_meas_t, 
            const actuators::PropulsorActuators& propulsor_actuators,
            ControlType control_type
        );

        FullStateControlLawInput make_full_state_control_input(
            const dynamics::RigidBodyState& xN_meas_t, 
            const linearization::TrimLinearization& lin_sol, 
            const actuators::SurfaceActuators& surface_actuators,
            const actuators::PropulsorActuators& propulsor_actuators,
            ControlType control_type
        );

        ControlInputs step(
            const dynamics::RigidBodyState& xN_meas_t, 
            const actuators::SurfaceActuators& surface_actuators
        );

        ControlInputs step(
            const dynamics::RigidBodyState& xN_meas_t, 
            const actuators::PropulsorActuators& propulsor_actuators
        );

        ControlInputs step(
            const dynamics::RigidBodyState& xN_meas_t, 
            const linearization::TrimLinearization& lin_sol, 
            const trim::TrimActuatorInputs<double>& trim_sol_input, 
            const actuators::SurfaceActuators& surface_actuators,
            const actuators::PropulsorActuators& propulsor_actuators
        );
    };

    Eigen::VectorXd surface_actuator_inputs_to_vector(const SurfaceActuatorInputs& u_surface);
    Eigen::VectorXd propulsor_actuator_inputs_to_vector(const PropulsorActuatorInputs& u_propulsor);
}
