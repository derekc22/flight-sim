#include <algorithm>
#include "simulation/actuators/public.hpp"
#include "simulation/control/linear_quadratic/public.hpp"
#include "simulation/control/linear_quadratic/slicot_care.hpp"
#include "simulation/dynamics/public.hpp"

namespace control {

    LinearQuadraticController::LinearQuadraticController(const LinearQuadraticControllerParameters& params) : params(params) {}

    actuators::ActuatorInputsVector_T<double> LinearQuadraticController::step(const LinearQuadraticControllerInput& controller_input) {

        if (!params.K.has_value()){
            const CareSolution care_sol = control::solve_care(controller_input.A, controller_input.B, params.Q, params.R);
            params.K = lqr_gain(controller_input.B, params.R, care_sol.P);
        }

        Eigen::VectorXd zN_t_deviation = controller_input.zN_t - controller_input.zN_t_des;

        actuators::ActuatorInputsVector_T<double> u_deviation;
        u_deviation = -params.K.value() * zN_t_deviation;

        return u_deviation;
    }

}
