#pragma once
#include "simulation/control/shared/public.hpp"
#include "simulation/control/pid/public.hpp"

namespace control {

    struct AxialPIDParameters {
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
        PIDPolicy lateral_policy;
        PIDPolicy longitudinal_policy;
        PIDPolicy vertical_policy;

        AxialPID(const AxialPIDParameters& params);
        ControlOutput step(const AttitudeControllerInput& input);

        virtual PIDPolicyInput make_pid_policy_input(const AttitudeControllerInput& input, ControlAxis axis);
    };

}
