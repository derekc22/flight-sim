#pragma once
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/actuators/actuators.hpp"
#include "simulation/control/shared.hpp"
#include "simulation/control/pid/pid.hpp"
#include "simulation/guidance/guidance.hpp"

namespace control { struct ControlOutput; struct VelocityControlLawInput; } // forward declare

namespace control {

    struct VelocityPIDParameters : PIDControlLawParameters, VelocityControlLawParameters {};

    struct VelocityPID {
        VelocityPIDParameters params;
        PIDControlLaw policy;

        VelocityPID(const VelocityPIDParameters& params);
        ControlOutput step(const VelocityControlLawInput& ctrl_law_input);

        PIDControlLawInput make_pid_control_law_input(const VelocityControlLawInput& ctrl_law_input);
        std::tuple<double, double, double> allocate_thrust(double T_tot, const actuators::PropulsorActuators& propulsor_actuators);
    };
}
