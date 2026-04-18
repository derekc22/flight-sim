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

    struct FullStateControlSetpoint {
        dynamics::LinearVelocity vB_BI;
        dynamics::AngularVelocity wB_BI;
        dynamics::EulerAngles eulIB;
    };

    struct ControlSurfaceInputs {
        double elevator = 0.0;  // rad
        double aileron = 0.0;   // rad
        double rudder = 0.0;    // rad
        double flaps = 0.0;     // rad
        double spoilers = 0.0;  // rad
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

    // struct ControlLawInput {};

    struct AxisControlLawInput {
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
        const actuators::Actuators& actuators;
    };

    template <typename ControlLawCommand>
    using AxisControlLaw = std::function<ControlLawCommand(const AxisControlLawInput&)>;

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
        LinearQuadraticIntegrator
    };

    struct ControlProperties {

        AxisControlLaw<double> longitudinal_controller;
        AxisControlLaw<double> lateral_controller;
        AxisControlLaw<double> vertical_controller;
        FullStateControlLaw<Eigen::VectorXd> full_state_controller;

        ControlType longitudinal_control_type = ControlType::None;
        ControlType lateral_control_type = ControlType::None;
        ControlType vertical_control_type = ControlType::None;
        ControlType full_state_control_type = ControlType::None;

        AxisControlSetpoint axis_setpoint;
        FullStateControlSetpoint full_state_setpoint;
        
        AxisControlLawInput make_axis_control_input(const dynamics::RigidBodyState& xN_meas_t, const actuators::Actuator& actuator, ControlType control_type);
        FullStateControlLawInput make_full_state_control_input(const dynamics::RigidBodyState& xN_meas_t, const linearization::TrimLinearization& lin_sol, const actuators::Actuators& actuators, ControlType control_type);

        ControlSurfaceInputs step(const dynamics::RigidBodyState& xN_meas_t, const actuators::Actuators& actuators);
        ControlSurfaceInputs step(const dynamics::RigidBodyState& xN_meas_t, const linearization::TrimLinearization& lin_sol, const trim::TrimControlSurfaceInputs<double>& trim_sol_input, const actuators::Actuators& actuators);
    };

}
