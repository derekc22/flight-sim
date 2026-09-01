#include "simulation/control/private/components/full_state/lqi.hpp"
#include "simulation/control/private/components/full_state/lqr.hpp"
#include "simulation/control/public/components/full_state.hpp"
#include "simulation/util/public.hpp"

namespace control {

    FullStateControl::FullStateControl(ControllerType controller_type, const LinearQuadraticRegulatorParameters& params) :
        controller_type(controller_type),
        implementation(
            [controller = LinearQuadraticRegulator{ params }](const FullStateControlInput& input, double dt) mutable {
                return controller.step(input, dt);
            }
        )
    {}

    FullStateControl::FullStateControl(ControllerType controller_type, const LinearQuadraticIntegratorParameters& params) :
        controller_type(controller_type),
        implementation(
            [controller = LinearQuadraticIntegrator{ params }](const FullStateControlInput& input, double dt) mutable {
                return controller.step(input, dt);
            }
        )
    {}

    ControlComponentOutput FullStateControl::step(const FullStateControlInput& input, double dt) {
        ControlComponentOutput output{ .virtual_control = implementation(input, dt) };
        util::fill_arr(output.active_mask, 0, 6, true);
        util::fill_arr(output.actuator_mask, 0, 6, true);
        return output;
    }

}
