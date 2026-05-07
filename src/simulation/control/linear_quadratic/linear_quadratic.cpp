#include <algorithm>
#include "simulation/control/control.hpp"
#include "simulation/control/linear_quadratic/care.hpp"
#include "simulation/trim/trim.hpp"

namespace control {

    LinearQuadraticController::LinearQuadraticController(const LinearQuadraticControllerParameters& params) : params(params) {}

    Eigen::VectorXd LinearQuadraticController::step(const LinearQuadraticControllerInput& controller_input) {

        if (!params.K.has_value()){
            const CareSolution care_sol = control::solve_care(controller_input.A, controller_input.B, params.Q, params.R);
            params.K = lqr_gain(controller_input.B, params.R, care_sol.P);
        }

        trim::TrimStateVector_T<double> meas_deviation = controller_input.meas - controller_input.meas_des;

        Eigen::VectorXd u_deviation;
        if (!params.integrator_bool){
            u_deviation = -params.K.value() * meas_deviation;
        } else {
            ; // LQI logic
        }

        return u_deviation;
    }

}