#pragma once
#include "simulation/control/shared/public.hpp"
#include "simulation/control/pid/public.hpp"

namespace control {

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

    struct AttitudePID {
        enum ControlAxis { Longitudinal, Lateral, Vertical };
        PIDPolicy lateral_policy;
        PIDPolicy longitudinal_policy;
        PIDPolicy vertical_policy;

        AttitudePID(const AttitudePIDParameters& params);
        VirtualControlOutput_T<double> step(const AttitudeControllerInput& input, double dt);

        virtual PIDPolicyInput make_pid_policy_input(const AttitudeControllerInput& input, ControlAxis axis);
    };

}
