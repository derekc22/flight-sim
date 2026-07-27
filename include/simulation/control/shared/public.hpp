#pragma once
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/guidance/public.hpp"
#include "simulation/linearization/public.hpp"

namespace control {

    using ControlOutput = actuators::ActuatorInputs_T<double>;

	using VirtualControlOutput = dynamics::Wrench;

	template <typename T>
	using VirtualControlOutputVector_T = dynamics::WrenchVector_T<T>;

	template <typename T>
    using VirtualControlOutput_T = dynamics::Wrench_T<T>;

    struct AttitudeControllerInput {
        const dynamics::RigidBodyState& Zt;
        const guidance::AttitudeSetpoint& setpoint;
    };

    struct VelocityControllerInput {
        const dynamics::RigidBodyState& Zt;
        const guidance::VelocitySetpoint& setpoint;
    };

    struct LinearQuadraticControllerInput {
        const dynamics::RigidBodyState& Zt;
        const VirtualControlOutput_T<double>& mu_sol_trim;
        const actuators::SurfaceActuators& surface_actuators;
        const actuators::PropulsorActuators& propulsor_actuators;
        const linearization::VirtualLocalLinearization& virtual_linearization;
        const guidance::LinearQuadraticSetpoint& setpoint;
    };

    struct NonlinearControllerInput {
        const dynamics::RigidBodyState& Zt;
        const actuators::SurfaceActuators& surface_actuators;
        const actuators::PropulsorActuators& propulsor_actuators;
        const guidance::NonlinearSetpoint& setpoint;
    };

}
