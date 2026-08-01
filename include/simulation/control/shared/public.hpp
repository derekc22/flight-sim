#pragma once
#include <array>
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/guidance/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/constants/public.hpp"

namespace control {

    using ControlOutput = actuators::ActuatorInputs_T<double>;

    struct ControlOutputSet {
        ControlOutput u;
        dynamics::WrenchVector_T<double> delta_mu_vec_t_1;
    };

	using VirtualControlOutput = dynamics::Wrench;

	template <typename T>
	using VirtualControlOutputVector_T = dynamics::WrenchVector_T<T>;

	template <typename T>
    using VirtualControlOutput_T = dynamics::Wrench_T<T>;

    struct VirtualControlOutputSet {
        VirtualControlOutput mu;
        std::array<bool, constants::virtual_input_dim> active_mask;
    };

    struct AttitudeControllerInput {
        const dynamics::RigidBodyState& Zt;
        const guidance::AttitudeSetpoint& setpoint;
        dynamics::WrenchVector_T<double> delta_mu_vec_t_1;
    };

    struct VelocityControllerInput {
        const dynamics::RigidBodyState& Zt;
        const guidance::VelocitySetpoint& setpoint;
        dynamics::WrenchVector_T<double> delta_mu_vec_t_1;
    };

    struct LinearQuadraticControllerInput {
        const dynamics::RigidBodyState& Zt;
        const VirtualControlOutput_T<double>& mu_sol_trim;
        const actuators::SurfaceActuators& surface_actuators;
        const actuators::PropulsorActuators& propulsor_actuators;
        const linearization::VirtualLocalLinearization& virtual_linearization;
        const guidance::LinearQuadraticSetpoint& setpoint;
        dynamics::WrenchVector_T<double> delta_mu_vec_t_1;
    };

    struct NonlinearControllerInput {
        const dynamics::RigidBodyState& Zt;
        const actuators::SurfaceActuators& surface_actuators;
        const actuators::PropulsorActuators& propulsor_actuators;
        const guidance::NonlinearSetpoint& setpoint;
    };

}
