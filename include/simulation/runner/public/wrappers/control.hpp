#pragma once
#include <array>
#include <optional>
#include "core/devices/public/joystick.hpp"
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/runner/public/data/types.hpp"

namespace runner {

    struct ControlWrapper {
        std::optional<devices::JoystickManager> joystick_manager;

        guidance::GuidanceSetpoint setpoint_t_1;
        control::VirtualControlOutput mu_cmd_t_1;
        std::array<bool, constants::virtual_input_dim> active_mask_t_1;
        std::array<bool, constants::input_dim> actuator_mask_t_1;
        control::ControlOutput u_cmd_t_1;
        control::ControlOutput u_actual_t_1;

        // initialize prior-step delta mu
        dynamics::WrenchVector_T<double> delta_mu_vec_t_1{};

        ControlWrapper(const JSONFlags& flags, const actuators::SurfaceActuators& surface_actuators, const actuators::PropulsorActuators& propulsor_actuators);
        devices::JoystickManagerOutput poll_joystick();
        ControlWrapperOutput step(const ControlWrapperInput& input);
    };

}
