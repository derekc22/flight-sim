#pragma once
#include <optional>
#include "simulation/control/public/components/attitude.hpp"
#include "simulation/control/public/components/full_state.hpp"
#include "simulation/control/public/components/velocity.hpp"
#include "simulation/control/public/data/types.hpp"

namespace control {

    struct ControlManager {
        std::optional<AttitudeControl> attitude_control;
        std::optional<VelocityControl> velocity_control;
        std::optional<FullStateControl> full_state_control;

        ControlManagerOutput step(const ControlManagerInput& input);
        ControlManagerOutput step(const ControlComponentInputs& inputs, double dt);

        ControlComponentInputs build_component_inputs(
            const dynamics::RigidBodyState& Zt, 
            const trim::TrimSolution& trim_sol, 
            const linearization::VirtualLocalLinearization& virtual_lin_sol, 
            const actuators::SurfaceActuators& surface_actuators, 
            const actuators::PropulsorActuators& propulsor_actuators, 
            const guidance::GuidanceSetpoint& setpoint,
            dynamics::WrenchVector_T<double> delta_mu_vec_t_1
        );
    };

}
