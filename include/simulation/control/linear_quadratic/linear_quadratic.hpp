#pragma once
#include "simulation/control/control.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/actuators/actuators.hpp"
#include "simulation/trim/trim.hpp"
#include "simulation/linearization/linearization.hpp"

namespace control {

    struct LinearQuadraticControlLawInput {
        trim::TrimStateVector_T<double> meas;
        trim::TrimStateVector_T<double> meas_des;
        Eigen::MatrixXd A;
        Eigen::MatrixXd B;
    };

    struct LinearQuadraticControlLawParameters {
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
        std::optional<Eigen::MatrixXd> K;
        bool integrator_bool = false;
    };

    struct LinearQuadraticControlLaw {
        const LinearQuadraticControlLawParameters& params;

        LinearQuadraticControlLaw(const LinearQuadraticControlLawParameters& params);
        Eigen::VectorXd step(const LinearQuadraticControlLawInput& ctrl_law_input);
    };
}