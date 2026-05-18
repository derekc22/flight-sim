#pragma once
#include <Eigen/Dense>
#include "simulation/dynamics/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/control/shared/public.hpp"
#include "simulation/control/linear_quadratic/public.hpp"
#include "simulation/guidance/public.hpp"

namespace control {

    struct LinearQuadraticRegulatorParameters : LinearQuadraticControllerParameters, LinearFullStateFeedbackControllerParameters {};

    struct LinearQuadraticRegulator {
        LinearQuadraticRegulatorParameters params;
        LinearQuadraticController policy;

        LinearQuadraticRegulator(const LinearQuadraticRegulatorParameters& params);
        ControlOutput step(const LinearFullStateFeedbackControllerInput& controller_input);

        LinearQuadraticControllerInput make_linear_quadratic_controller_input(const LinearFullStateFeedbackControllerInput& controller_input);
        dynamics::StateVector_T<double> unpack_linear_quadratic_regulator_setpoint(const guidance::LinearFullStateFeedbackSetpoint& linear_quadratic_regulator_setpoint);
    };
}
