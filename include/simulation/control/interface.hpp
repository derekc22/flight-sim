#pragma once
#include "simulation/actuators/propulsor/shared.hpp"
#include "simulation/actuators/surface/shared.hpp"
#include "simulation/actuators/shared.hpp"
#include "simulation/control/shared.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/guidance/shared.hpp"
#include "simulation/linearization/shared.hpp"

namespace control {

    struct AxialControllerInput {
        const dynamics::RigidBodyState& zN_t;
        const actuators::SurfaceActuators& surface_actuators;
        const guidance::AxialSetpoint& setpoint;
    };
    struct AxialControllerParameters {};

    struct VelocityControllerInput {
        const dynamics::RigidBodyState& zN_t;
        const actuators::PropulsorActuators& propulsor_actuators;
        const guidance::VelocitySetpoint& setpoint;
    };
    struct VelocityControllerParameters {};

    struct LinearFullStateFeedbackControllerInput {
        const dynamics::RigidBodyState& zN_t;
        const actuators::ActuatorInputs_T<double>& u_sol_trim;
        const linearization::StateJacobian& A;
        const linearization::TrimInputJacobian& B;
        const guidance::LinearFullStateFeedbackSetpoint& setpoint;
    };
    struct LinearFullStateFeedbackControllerParameters {};

    struct NonlinearControllerInput {
        const dynamics::RigidBodyState& zN_t;
        const actuators::SurfaceActuators& surface_actuators;
        const actuators::PropulsorActuators& propulsor_actuators;
        const guidance::NonlinearSetpoint& setpoint;
    };
    struct NonlinearControllerParameters {};
}
