#include <algorithm>
#include "simulation/control/control.hpp"
#include "simulation/control/linear_quadratic/care.hpp"
#include "simulation/trim/trim.hpp"

namespace control {

    LinearQuadraticController::LinearQuadraticController(const LinearQuadraticControllerParameters& params) : params(params) {}

    trim::TrimActuatorInputsVector_T<double> LinearQuadraticController::step(const LinearQuadraticControllerInput& controller_input) {

        if (!params.K.has_value()){
            const CareSolution care_sol = control::solve_care(controller_input.A, controller_input.B, params.Q, params.R);
            params.K = lqr_gain(controller_input.B, params.R, care_sol.P);
        }

        trim::TrimStateVector_T<double> zN_t_deviation = controller_input.zN_t - controller_input.zN_t_des;

        trim::TrimActuatorInputsVector_T<double> u_deviation;
        if (!params.integrator_bool){
            u_deviation = -params.K.value() * zN_t_deviation;
        } else {
            ; // LQI logic
        }

        return u_deviation;
    }

}
