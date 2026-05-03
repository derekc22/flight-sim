#include "simulation/control/linear_quadratic/controllers/lqr.hpp"
#include "simulation/control/linear_quadratic/controllers/lqi.hpp"
#include "simulation/constants/constants.hpp"

namespace control {

    LinearQuadraticIntegrator::LinearQuadraticIntegrator(const LinearQuadraticIntegratorParamters& params) {
        double n = constants::state_dim;
        MatrixXd Q_aug = MatrixXd::Zero(2 * n, 2 * n);
        Q_aug.block(0, 0, n, n) = params.Q;
        Q_aug.block(n, n, n, n) = params.Qi;

        policy = LinearFullStateFeedbackControlLaw({ 
            .Q = Q_aug, 
            .R = params.R,
            .integrator_bool = true
        })
    };

    LinearFullStateFeedbackControlLawInput LinearQuadraticIntegrator::make_linear_quadratic_control_law_input(
        const LinearQuadraticRegulatorInput& ctrl_law_input
    ){
        /** @todo: Add augmentation structure for LQI */
        linearization::TrimStateJacobian A_aug;
        linearization::TrimInputJacobian B_aug;
        return {
            .meas = trim::unpack_rigid_body_state(ctrl_law_input.zN_t),
            .meas_des = unpack_linear_quadratic_regulator_setpoint(setpoint),
            .A = ctrl_law_input.A_aug,
            .B = ctrl_law_input.B_aug
        };
    }

}