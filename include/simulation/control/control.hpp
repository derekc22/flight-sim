#pragma once
#include <functional>
#include <string>
#include <Eigen/Dense>
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/actuators/actuators.hpp"

namespace control {

    struct ControlSurfaceInputs {
        // u ∈ [−umax, +umax]
        double elevator = 0.0;  // rad
        double aileron = 0.0;   // rad
        double rudder = 0.0;    // rad
        // u ∈ [0, +umax]
        double flaps = 0.0;      // rad
        double spoilers = 0.0;   // rad
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
        double meas_dot;

        double meas_des;
        double meas_dot_des;

        double limit_max;
        double limit_min;
    };

    struct FullStateControlLawInput {
        dynamics::RigidBodyState rbs_meas;

        dynamics::RigidBodyState rbs_des;

        actuators::Actuators actuators;
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
        LinearQuadraticTracker
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

        dynamics::RigidBodyState xN_des_t;
        ControlSurfaceInputs step(const dynamics::RigidBodyState& xN_meas_t, const actuators::Actuators& actuators);
        AxisControlLawInput make_axis_control_input(const dynamics::RigidBodyState& xN_meas_t, const actuators::Actuator& actuator, ControlType control_type);

    };

}
