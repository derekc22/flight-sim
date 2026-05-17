#pragma once
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/guidance/public.hpp"
#include "simulation/linearization/public.hpp"

namespace control {

    struct ControlOutput {
        actuators::SurfaceActuatorInputs_T<double> surface_inputs;
        actuators::PropulsorActuatorInputs_T<double> propulsor_inputs;
    };

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
