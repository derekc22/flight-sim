#pragma once
#include <functional>
#include <string>
#include <Eigen/Dense>
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/actuators/actuators.hpp"
#include "simulation/autopilot/autopilot.hpp"
#include "simulation/analysis/analysis.hpp"

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
        const analysis::TrimStateJacobian& A;
        const analysis::TrimInputJacobian& B;
        autopilot::TrimStateVector_T<double> meas;
        autopilot::TrimStateVector_T<double> meas_des;
        const actuators::Actuators& actuators;
    };

    template <typename ControlLawCommand>
    using AxisControlLaw = std::function<ControlLawCommand(const AxisControlLawInput&)>;

    template <typename ControlLawCommand>
    using FullStateControlLaw = std::function<ControlLawCommand(const FullStateControlLawInput&)>;

    struct PIDController {
        double Kp = 0;
        double Kd = 0;
        double Ki = 0;

        double integral = 0.0;
        double d_filtered = 0.0;
        double prev_err = 0.0;
        double tau;

        PIDController(const ControlLawParameters& params);

        double _step(const AxisControlLawInput& ctrl_law_input);
    };

    struct RollPIDController : PIDController {
        RollPIDController(const ControlLawParameters& params);
    };

    struct PitchPIDController : PIDController {
        PitchPIDController(const ControlLawParameters& params);
    };

    struct YawPIDController : PIDController {
        YawPIDController(const ControlLawParameters& params);
    };

    struct RollDamper : PIDController {
        RollDamper(const ControlLawParameters& params);
    };

    struct PitchDamper : PIDController {
        PitchDamper(const ControlLawParameters& params);
    };

    struct YawDamper : PIDController {
        YawDamper(const ControlLawParameters& params);
    };

    struct LinearQuadraticController {
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
        std::optional<Eigen::MatrixXd> K_lqr;

        LinearQuadraticController(const ControlLawParameters& params);

        Eigen::VectorXd _step(const FullStateControlLawInput& ctrl_law_input);
    };

    struct LinearQuadraticRegulator : LinearQuadraticController {
        LinearQuadraticRegulator(const ControlLawParameters& params);
    };

    struct LinearQuadraticTracker : LinearQuadraticController {
        LinearQuadraticTracker(const ControlLawParameters& params);
    };


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
        FullStateControlLawInput make_full_state_control_input(const dynamics::RigidBodyState& xN_meas_t, const analysis::TrimLinearization& lin_sol, const actuators::Actuators& actuators, ControlType control_type);

        ControlSurfaceInputs step(const dynamics::RigidBodyState& xN_meas_t, const actuators::Actuators& actuators);
        ControlSurfaceInputs step(const dynamics::RigidBodyState& xN_meas_t, const analysis::TrimLinearization& lin_sol, const autopilot::TrimControlSurfaceInputs<double>& trim_sol_input, const actuators::Actuators& actuators);
    };

}
