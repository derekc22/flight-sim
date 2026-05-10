#include <Eigen/Dense>
#include "simulation/control/interface.hpp"
#include "simulation/control/linear_quadratic/controllers/lqr.hpp"
#include "simulation/control/linear_quadratic/controllers/lqi.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/linearization/shared.hpp"
#include "simulation/trim/shared.hpp"

namespace control {

    LinearQuadraticIntegrator::LinearQuadraticIntegrator(const LinearQuadraticIntegratorParameters& params) :
        LinearQuadraticRegulator(params)
    {
        double n = constants::state_dim;
        Eigen::MatrixXd Q_aug = Eigen::MatrixXd::Zero(2 * n, 2 * n);
        Q_aug.block(0, 0, n, n) = params.Q;
        Q_aug.block(n, n, n, n) = params.Qi;

        policy = LinearQuadraticController({ 
            .Q = Q_aug, 
            .R = params.R,
            .integrator_bool = true
        });
    };

    LinearQuadraticControllerInput LinearQuadraticIntegrator::make_linear_quadratic_controller_input(const LinearFullStateFeedbackControllerInput& controller_input){
        /** @todo: Add augmentation structure for LQI */
        linearization::StateJacobian A_aug;
        linearization::TrimInputJacobian B_aug;
        return {
            .zN_t = trim::unpack_rigid_body_state(controller_input.zN_t),
            .zN_t_des = unpack_linear_quadratic_regulator_setpoint(controller_input.setpoint),
            .A = A_aug,
            .B = B_aug
        };
    }

}
