#pragma once
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/actuators/actuators.hpp"
#include "simulation/control/shared.hpp"
#include "simulation/control/pid/pid.hpp"
#include "simulation/guidance/guidance.hpp"

namespace control { struct ControlOutput; struct VelocityControllerInput; } // forward declare

namespace control {

    struct VelocityPIDParameters : PIDControllerParameters, VelocityControllerParameters {};

    struct VelocityPID {
        VelocityPIDParameters params;
        PIDController policy;

        VelocityPID(const VelocityPIDParameters& params);
        ControlOutput step(const VelocityControllerInput& ctrl_law_input);

        PIDControllerInput make_pid_controller_input(const VelocityControllerInput& ctrl_law_input);
        std::tuple<double, double, double> allocate_thrust(double T_tot, const actuators::PropulsorActuators& propulsor_actuators);
    };
}
