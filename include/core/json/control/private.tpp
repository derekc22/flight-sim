#pragma once

namespace json {

    template <typename Controller, typename ControllerClass, typename ControllerParameters, typename ControllerClassInput>
    ControllerClass make_stateful_controller(const ControllerParameters& params) {
        return [controller = Controller{ params }](const ControllerClassInput& input) mutable {
            return controller.step(input);
        };
    }
}
