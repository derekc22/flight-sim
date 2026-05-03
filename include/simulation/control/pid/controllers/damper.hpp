
#include "simulation/control/pid/controllers/axial.hpp"

namespace control { struct PIDControlLawInput; struct ControlOutput; } // forward declare

namespace control {

    struct DamperPID : AxialPID {
        PIDControlLawInput make_pid_control_law_input(
            const AxialPIDInput& ctrl_law_input,
            ControlAxis control_axis
        ) override;
    };
}