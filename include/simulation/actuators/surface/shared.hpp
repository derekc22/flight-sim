#pragma once
#include "simulation/actuators/actuator/actuator.hpp"

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

    template <typename T>
    struct SurfaceActuatorInputs_T {
        T elevator_cmd = T(0);
        T aileron_cmd = T(0);
        T rudder_cmd = T(0);
        T flap_cmd = T(0);
        T spoiler_cmd = T(0);
    };

}
