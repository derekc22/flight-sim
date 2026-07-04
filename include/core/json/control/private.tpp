#pragma once

namespace json {

    template <typename Controller, typename ControllerClass, typename ControllerParameters, typename ControllerClassInput>
    ControllerClass make_stateful_controller(const ControllerParameters& params) {
        // Creates and returns a stateful lambda that owns a Controller initialized with params
        // Each call forwards the ControllerClassInput to the stored controller's step(input) method and returns the result
        // mutable is required because lambda objects treat captured values as const by default, but step(input) may modify the stored controller
        return [controller = Controller{ params }](const ControllerClassInput& input, double dt) mutable {
            return controller.step(input, dt);
        };
    }
}
