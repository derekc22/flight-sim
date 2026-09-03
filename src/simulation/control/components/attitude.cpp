#include "simulation/control/private/components/attitude/damper.hpp"
#include "simulation/control/private/components/attitude/pid.hpp"
#include "simulation/control/public/components/attitude.hpp"
#include "simulation/util/public.hpp"

namespace control {

    // Creates a stateful lambda that owns a controller initialized with params
    // Each call forwards input and dt to the stored controller's step() method and returns the result
    // mutable is required because captured values are const by default, but step() may modify the stored controller
    AttitudeControl::AttitudeControl(const AttitudePIDParameters& params) :
        implementation([controller = AttitudePID{ params }](const AttitudeControlInput& input, double dt) mutable {
            return controller.step(input, dt);
        })
    {}

    AttitudeControl::AttitudeControl(const DamperPIDParameters& params) :
        implementation([controller = DamperPID{ params }](const AttitudeControlInput& input, double dt) mutable {
            return controller.step(input, dt);
        })
    {}

    ControlComponentOutput AttitudeControl::step(const AttitudeControlInput& input, double dt) {
        ControlComponentOutput output{ .mu = implementation(input, dt) };
        util::fill_arr(output.active_mask, 3, 6, true);
        util::fill_arr(output.actuator_mask, 0, 3, true);
        return output;
    }

}
