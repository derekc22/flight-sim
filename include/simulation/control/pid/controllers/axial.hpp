#pragma once
#include "simulation/control/interface.hpp"
#include "simulation/control/shared.hpp"
#include "simulation/control/pid/pid.hpp"

namespace control { struct ControlOutput; struct AxialControllerInput; } // forward declare

namespace control {

    struct AxialPIDParameters : AxialControllerParameters {
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

    struct AxialPID {
        enum ControlAxis { Longitudinal, Lateral, Vertical };
        AxialPIDParameters params;
        PIDController lateral_policy;
        PIDController longitudinal_policy;
        PIDController vertical_policy;

        AxialPID(const AxialPIDParameters& params);
        ControlOutput step(const AxialControllerInput& controller_input);

        virtual PIDControllerInput make_pid_controller_input(const AxialControllerInput& controller_input, ControlAxis control_axis);
    };

}
