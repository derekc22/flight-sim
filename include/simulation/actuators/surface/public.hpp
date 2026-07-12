#pragma once
#include <Eigen/Dense>
#include "simulation/actuators/actuator/public.hpp"
#include "simulation/constants/public.hpp"

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
        T elevator_cmd = T(0.0);
        T aileron_cmd = T(0.0);
        T rudder_cmd = T(0.0);
        T flap_cmd = T(0.0);
        T spoiler_cmd = T(0.0);
    };

    using SurfaceActuatorInputsVector = Eigen::Matrix<double, constants::full_surface_input_dim, 1>;

    SurfaceActuatorInputsVector unpack_surface_actuator_inputs(const SurfaceActuatorInputs_T<double>& u_surface);

}
