#include <algorithm>
#include "simulation/control/control.hpp"
#include "simulation/control/lqr.hpp"
#include "simulation/control/care.hpp"
#include "simulation/trim/trim.hpp"

namespace control {

    LinearQuadraticControlLaw::LinearQuadraticControlLaw(const LinearQuadraticControlLawParamters& params) : params(params) {}

    Eigen::VectorXd LinearQuadraticControlLaw::step(const LinearQuadraticControlLawInput& ctrl_law_input) {

        if (!params.K.has_value()){
            const control::CareSolution care_sol = control::solve_care(ctrl_law_input.A, ctrl_law_input.B, params.Q, params.R);
            params.K = control::lqr_gain(ctrl_law_input.B, params.R, care_sol.P);
        }

        trim::TrimStateVector_T<double> meas_deviation = ctrl_law_input.meas - ctrl_law_input.meas_des;

        if (!params.integrator_bool){
            Eigen::VectorXd u_deviation = -K.value() * meas_deviation;
        } else {
            // LQI logic
        }

        return u_deviation;
    }

}