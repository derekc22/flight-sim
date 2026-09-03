#include "simulation/control/private/components/linear_quadratic/lqi.hpp"
#include "simulation/control/private/components/linear_quadratic/lqr.hpp"
#include "simulation/control/public/components/linear_quadratic.hpp"
#include "simulation/util/public/math.hpp"

namespace control {

    LinearQuadraticControl::LinearQuadraticControl(const LinearQuadraticRegulatorParameters& params) :
        implementation(
            [controller = LinearQuadraticRegulator{ params }](const LinearQuadraticControlInput& input, double dt) mutable {
                return controller.step(input, dt);
            }
        )
    {}

    LinearQuadraticControl::LinearQuadraticControl(const LinearQuadraticIntegratorParameters& params) :
        implementation(
            [controller = LinearQuadraticIntegrator{ params }](const LinearQuadraticControlInput& input, double dt) mutable {
                return controller.step(input, dt);
            }
        )
    {}

    ControlComponentOutput LinearQuadraticControl::step(const LinearQuadraticControlInput& input, double dt) {
        ControlComponentOutput output{ .mu = implementation(input, dt) };
        util::fill_arr(output.active_mask, 0, 6, true);
        util::fill_arr(output.actuator_mask, 0, 6, true);
        return output;
    }

}
