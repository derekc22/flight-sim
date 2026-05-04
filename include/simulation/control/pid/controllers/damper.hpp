
#include "simulation/control/pid/controllers/axial.hpp"

namespace control { struct PIDControllerInput; struct ControlOutput; } // forward declare

namespace control {

    struct DamperPID : AxialPID {
        using AxialPID::AxialPID;

        PIDControllerInput make_pid_controller_input(
            const AxialControllerInput& ctrl_law_input,
            ControlAxis control_axis
        ) override;
    };
}