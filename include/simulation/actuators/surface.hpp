#pragma once
#include <optional>
#include "simulation/actuators/actuator.hpp"

/** @deprecated ?? */
// namespace control { struct SurfaceActuatorInputs; } // forward declare

namespace actuators {

    struct SurfaceActuator : Actuator { using Actuator::Actuator; };

    struct Elevator : SurfaceActuator { using SurfaceActuator::SurfaceActuator; };
    struct Aileron : SurfaceActuator { using SurfaceActuator::SurfaceActuator; };
    struct Rudder : SurfaceActuator { using SurfaceActuator::SurfaceActuator; };
    struct Flap : SurfaceActuator { using SurfaceActuator::SurfaceActuator; };
    struct Spoiler : SurfaceActuator { using SurfaceActuator::SurfaceActuator; };

    struct SurfaceActuators {
        Elevator elevator;
        Aileron aileron;
        Rudder rudder;
        Flap flap;
        Spoiler spoiler;
    };

}
