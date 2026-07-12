#pragma once
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/guidance/public.hpp"
#include "simulation/linearization/public.hpp"

namespace control {

    using ControlOutput = actuators::ActuatorInputs_T<double>;

    struct AttitudeControllerInput {
        const dynamics::RigidBodyState& Zt;
        const actuators::SurfaceActuators& surface_actuators;
        const guidance::AttitudeSetpoint& setpoint;
    };

    struct VelocityControllerInput {
        const dynamics::RigidBodyState& Zt;
        const actuators::PropulsorActuators& propulsor_actuators;
        const guidance::VelocitySetpoint& setpoint;
    };

    struct LinearQuadraticControllerInput {
        const dynamics::RigidBodyState& Zt;
        const actuators::ActuatorInputs_T<double>& u_sol_trim;
        const actuators::SurfaceActuators& surface_actuators;
        const actuators::PropulsorActuators& propulsor_actuators;
        const linearization::StateJacobian& A;
        const linearization::InputJacobian& B;
        const guidance::LinearQuadraticSetpoint& setpoint;
    };

    struct NonlinearControllerInput {
        const dynamics::RigidBodyState& Zt;
        const actuators::SurfaceActuators& surface_actuators;
        const actuators::PropulsorActuators& propulsor_actuators;
        const guidance::NonlinearSetpoint& setpoint;
    };

}
