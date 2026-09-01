#include "simulation/control/private/components/velocity/pid.hpp"
#include "simulation/control/public/components/velocity.hpp"
#include "simulation/util/public.hpp"

namespace control {

    VelocityControl::VelocityControl(ControllerType controller_type, const VelocityPIDParameters& params) :
        controller_type(controller_type),
        implementation(
            [controller = VelocityPID{ params }](const VelocityControlInput& input, double dt) mutable {
                return controller.step(input, dt);
            }
        )
    {}

    ControlComponentOutput VelocityControl::step(const VelocityControlInput& input, double dt) {
        ControlComponentOutput output{ .virtual_control = implementation(input, dt) };
        util::fill_arr(output.active_mask, 0, 1, true);
        util::fill_arr(output.actuator_mask, 3, 4, true);
        return output;
    }

}
