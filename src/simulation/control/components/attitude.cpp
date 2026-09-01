#include <stdexcept>
#include "simulation/control/private/components/attitude/damper.hpp"
#include "simulation/control/private/components/attitude/pid.hpp"
#include "simulation/control/public/components/attitude.hpp"
#include "simulation/util/public.hpp"

namespace control {

    AttitudeControl::AttitudeControl(ControllerType controller_type, const AttitudePIDParameters& params) : controller_type(controller_type) {
        // Creates and returns a stateful lambda that owns a Controller initialized with params
        // Each call forwards the ControllerClassInput to the stored controller's step(input) method and returns the result
        // mutable is required because lambda objects treat captured values as const by default, but step(input) may modify the stored controller
        switch (controller_type) {
            case ControllerType::AttitudePID:
                implementation = [controller = AttitudePID{ params }](const AttitudeControlInput& input, double dt) mutable {
                    return controller.step(input, dt);
                };
                break;

            case ControllerType::DamperPID:
                implementation = [controller = DamperPID{ params }](const AttitudeControlInput& input, double dt) mutable {
                    return controller.step(input, dt);
                };
                break;

            default:
                throw std::runtime_error("control::AttitudeControl unknown control type");
        }
    }

    ControlComponentOutput AttitudeControl::step(const AttitudeControlInput& input, double dt) {
        ControlComponentOutput output{ .virtual_control = implementation(input, dt) };
        util::fill_arr(output.active_mask, 3, 6, true);
        util::fill_arr(output.actuator_mask, 0, 3, true);
        return output;
    }

}
