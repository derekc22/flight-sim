#pragma once
#include <Eigen/Dense>
#include <optional>
#include "simulation/control/control.hpp"

namespace control {

    struct LinearQuadraticController {
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
        std::optional<Eigen::MatrixXd> K;

        LinearQuadraticController(const ControlLawParameters& params);

        Eigen::VectorXd _step(const FullStateControlLawInput& ctrl_law_input);
    };

    struct LinearQuadraticRegulator : LinearQuadraticController {
        LinearQuadraticRegulator(const ControlLawParameters& params);
    };

    struct LinearQuadraticTracker : LinearQuadraticController {
        LinearQuadraticTracker(const ControlLawParameters& params);
    };

}
