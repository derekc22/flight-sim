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

    struct AttitudeControllerInput {
        const dynamics::RigidBodyState& zN_t;
        const actuators::SurfaceActuators& surface_actuators;
        const guidance::AxialSetpoint& setpoint;
    };

    struct VelocityControllerInput {
        const dynamics::RigidBodyState& zN_t;
        const actuators::PropulsorActuators& propulsor_actuators;
        const guidance::VelocitySetpoint& setpoint;
    };

    struct LinearQuadraticControllerInput {
        const dynamics::RigidBodyState& zN_t;
        const actuators::ActuatorInputs_T<double>& u_sol_trim;
        const actuators::SurfaceActuators& surface_actuators;
        const actuators::PropulsorActuators& propulsor_actuators;
        const linearization::StateJacobian& A;
        const linearization::InputJacobian& B;
        const guidance::LinearQuadraticSetpoint& setpoint;
    };

    struct NonlinearControllerInput {
        const dynamics::RigidBodyState& zN_t;
        const actuators::SurfaceActuators& surface_actuators;
        const actuators::PropulsorActuators& propulsor_actuators;
        const guidance::NonlinearSetpoint& setpoint;
    };

}
