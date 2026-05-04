#pragma once
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/actuators/actuators.hpp"
#include "simulation/control/shared.hpp"
#include "simulation/control/pid/pid.hpp"
#include "simulation/guidance/guidance.hpp"

namespace control { struct ControlOutput; struct AxialControlLawInput; } // forward declare

namespace control {

    struct AxialPIDInput : AxialControlLawInput {
        dynamics::RigidBodyState& zN_t;
        actuators::SurfaceActuators& surface_actuators;
        guidance::AxialSetpoint setpoint;
    };

    struct AxialPIDParameters : AxialControlLawParameters {
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
        PIDControlLaw lateral_policy;
        PIDControlLaw longitudinal_policy;
        PIDControlLaw vertical_policy;

        AxialPID(const AxialPIDParameters& params);
        ControlOutput step(const AxialPIDInput& ctrl_law_input);

        virtual PIDControlLawInput make_pid_control_law_input(
            const AxialPIDInput& ctrl_law_input,
            ControlAxis control_axis
        );
    };

}
