#pragma once
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/actuators/actuators.hpp"
#include "simulation/control/shared.hpp"
#include "simulation/control/pid/pid.hpp"

namespace control { struct ControlOutput; struct AxialControlLawInput; } // forward declare

namespace control {

    struct AxialPIDSetpoint {
        dynamics::EulerAngles eulIB;
        dynamics::AngularVelocity wB_BI;
    };

    struct AxialPIDInput : AxialControlLawInput {
        dynamics::RigidBodyState& zN_t;
        actuators::SurfaceActuators& surface_actuators;
        AxialPIDSetpoint setpoint;
    };

    struct AxialPIDParamters : AxialControlLawParameters {
        double Kp_lateral = 0.0;
        double Ki_lateral = 0.0;
        double Kd_lateral = 0.0;
        double tau_lateral = 0.0;

        double Kp_longitudinal = 0.0;
        double Ki_longitudinal = 0.0;
        double Kd_longitudinal = 0.0;
        double tau_longitudinal = 0.0;

        double Kp_vertical = 0.0;
        double Ki_vertical = 0.0;
        double Kd_vertical = 0.0;
        double tau_vertical = 0.0;
    };

    struct AxialPID {
        enum ControlAxis { Longitudinal, Lateral, Vertical };
        AxialPIDParamters params;
        PIDControlLaw lateral_policy;
        PIDControlLaw longitudinal_policy;
        PIDControlLaw vertical_policy;

        AxialPID(const AxialPIDParamters& params);
        ControlOutput step(const AxialPIDInput& ctrl_law_input);

        virtual PIDControlLawInput make_pid_control_law_input(
            const AxialPIDInput& ctrl_law_input,
            ControlAxis control_axis
        );
    };

}
