#include <Eigen/Dense>
#include "simulation/control/linear_quadratic/controllers/lqr.hpp"
#include "simulation/control/linear_quadratic/controllers/lqi.hpp"
#include "simulation/constants/constants.hpp"

namespace control {

    LinearQuadraticIntegrator::LinearQuadraticIntegrator(const LinearQuadraticIntegratorParameters& params) :
        LinearQuadraticRegulator(params)
    {
        double n = constants::state_dim;
        Eigen::MatrixXd Q_aug = Eigen::MatrixXd::Zero(2 * n, 2 * n);
        Q_aug.block(0, 0, n, n) = params.Q;
        Q_aug.block(n, n, n, n) = params.Qi;

        policy = LinearQuadraticControlLaw({ 
            .Q = Q_aug, 
            .R = params.R,
            .integrator_bool = true
        });
    };

    LinearQuadraticControlLawInput LinearQuadraticIntegrator::make_linear_quadratic_control_law_input(const LinearFullStateFeedbackControlLawInput& ctrl_law_input){
        /** @todo: Add augmentation structure for LQI */
        linearization::TrimStateJacobian A_aug;
        linearization::TrimInputJacobian B_aug;
        return {
            .meas = trim::unpack_rigid_body_state(ctrl_law_input.zN_t),
            .meas_des = unpack_linear_quadratic_regulator_setpoint(ctrl_law_input.setpoint),
            .A = A_aug,
            .B = B_aug
        };
    }

}