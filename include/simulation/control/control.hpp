#pragma once
#include <functional>
#include <string>
#include <Eigen/Dense>
#include "simulation/dynamics/dynamics.hpp"

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

    struct ControlSurfaceLimits {
        // Assume symmetric limits
        double elevator_max = 0.0;  // rad
        double aileron_max = 0.0;   // rad
        double rudder_max = 0.0;    // rad
        // Assume lower limit is zero
        double flap_max = 0.0;      // rad
        double spoiler_max = 0.0;   // rad
    };

    struct ControlLawParameters {
        Eigen::VectorXd gains;
        double ctrl_surface_max;
        double ctrl_surface_min;
    };

    typedef std::function<double(double, double, double)> ControlLaw;

    struct PIDController {
        double Kp = 0;
        double Kd = 0;
        double Ki = 0;
        double u_max = 0;
        double u_min = 0;

        double integral = 0.0;
        double d_filtered = 0.0;
        double tau = constants::eps;

        PIDController(const ControlLawParameters& params);

        double _step(double meas, double meas_dot, double setpoint);
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
        ControlSurfaceLimits limits;

        ControlLaw longitudinal_controller;
        ControlLaw lateral_controller;
        ControlLaw vertical_controller;
        ControlLaw full_state_controller;

        std::string longitudinal_control_type;
        std::string lateral_control_type;
        std::string vertical_control_type;
        bool full_state = false;

        dynamics::RigidBodyState xN_des_t;
        ControlSurfaceInputs step(const dynamics::RigidBodyState& xN_meas_t);
    };

}
