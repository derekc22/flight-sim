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
        double flap = 0.0;      // rad
        double spoiler = 0.0;   // rad
    };

    struct ControlLawParameters {
        Eigen::VectorXd gains;
    };

    struct ControlLawInput {
        double meas;
        double meas_dot;
        dynamics::RigidBodyState rbs_meas;

        double meas_des;
        double meas_dot_des;
        dynamics::RigidBodyState rbs_des;

        double limit_max;
        double limit_min;
        actuators::ActuatorLimits limits;
    };

    typedef std::function<double(const ControlLawInput&)> ControlLaw;

    struct PIDController {
        double Kp = 0;
        double Kd = 0;
        double Ki = 0;

        double integral = 0.0;
        double d_filtered = 0.0;
        double tau = constants::eps;

        PIDController(const ControlLawParameters& params);

        double _step(const ControlLawInput& ctrl_law_input);
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


    struct ControlProperties {

        ControlLaw longitudinal_controller;
        ControlLaw lateral_controller;
        ControlLaw vertical_controller;
        ControlLaw full_state_controller;

        std::string longitudinal_control_type;
        std::string lateral_control_type;
        std::string vertical_control_type;
        bool full_state = false;

        dynamics::RigidBodyState xN_des_t;
        ControlSurfaceInputs step(const dynamics::RigidBodyState& xN_meas_t, const actuators::ActuatorLimits& limits);
    };

}
