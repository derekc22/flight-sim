#pragma once

namespace json {

    template <typename ControllerType, typename ControllerClass, typename ControllerParametersType, typename ControllerInputType>
    ControllerClass make_stateful_controller(const ControllerParametersType& params) {
        return [controller = ControllerType{ params }](const ControllerInputType& input) mutable {
            return controller.step(input);
        };
    }
}
