#include <Eigen/Dense>
#include "simulation/control/linear_quadratic/controllers/lqr/public.hpp"
#include "simulation/control/linear_quadratic/controllers/lqi/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/trim/public.hpp"
#include "simulation/control/linear_quadratic/private.hpp"
#include "simulation/util/public.hpp"

namespace control {

    LinearQuadraticIntegrator::LinearQuadraticIntegrator(const LinearQuadraticIntegratorParameters& params) :
        LinearQuadraticRegulator(params)
    {
        size_t n = constants::state_dim;
        size_t i = integrated_state_dim;

        Eigen::MatrixXd Q_aug = Eigen::MatrixXd::Zero(n + i, n + i);
        Q_aug.block(0, 0, n, n) = params.Q;
        Q_aug.block(n, n, i, i) = params.Qi;

        policy = LinearQuadraticPolicy({ 
            .Q = Q_aug, 
            .R = params.R,
        });
    };

    IntegratedStateVector LinearQuadraticIntegrator::integrate_state_err(const dynamics::StateVector_T<double>& zN_t, const dynamics::StateVector_T<double>& zN_t_des) {
        IntegratedStateVector pqr_zN_t = zN_t.segment<integrated_state_dim>(3);  // grab p, q, r
        IntegratedStateVector pqr_zN_t_des = zN_t_des.segment<integrated_state_dim>(3);

        return integral + (pqr_zN_t_des - pqr_zN_t) * constants::dt;     // integrate
    }


    LinearQuadraticPolicyInput LinearQuadraticIntegrator::make_linear_quadratic_policy_input(const LinearQuadraticControllerInput& input, const IntegratedStateVector& integral_new) {
        size_t n = constants::state_dim;
        size_t m = constants::input_dim;
        size_t i = integrated_state_dim;

        Eigen::MatrixXd A_aug = Eigen::MatrixXd::Zero(n + i, n + i);
        A_aug.block(0, 0, n, n) = input.A;

        // Ci selects the integrated states p, q, r from the state vector for the LQI controller - it is not the system-wide output matrix C
        Eigen::MatrixXd Ci = Eigen::MatrixXd::Zero(integrated_state_dim, constants::state_dim);
        Ci.block<integrated_state_dim, integrated_state_dim>(0, 3) = Eigen::Matrix<double, integrated_state_dim, integrated_state_dim>::Identity();
        A_aug.block(n, 0, i, n) = -Ci;

        Eigen::MatrixXd B_aug = Eigen::MatrixXd::Zero(n + i, m);
        B_aug.block(0, 0, n, m) = input.B;

        dynamics::StateVector_T<double> zN_t = dynamics::unpack_rigid_body_state(input.zN_t);
        dynamics::StateVector_T<double> zN_t_des = unpack_linear_quadratic_control_setpoint(input.setpoint);

        AugmentedStateVector zN_t_aug;
        zN_t_aug << zN_t, integral_new;

        AugmentedStateVector zN_t_des_aug;
        zN_t_des_aug << zN_t_des, IntegratedStateVector::Zero();

        AugmentedStateVector zN_t_aug_deviation = zN_t_aug - zN_t_des_aug;

        return {
            .zN_t = zN_t_aug_deviation,
            .A = A_aug,
            .B = B_aug
        };
    }

    ControlOutput LinearQuadraticIntegrator::step(const LinearQuadraticControllerInput& input) {

        // integral candidate
        IntegratedStateVector integral_new = integrate_state_err(
            dynamics::unpack_rigid_body_state(input.zN_t), 
            unpack_linear_quadratic_control_setpoint(input.setpoint)
        );

        actuators::ActuatorInputsVector_T<double> u_deviation = policy.step(
            make_linear_quadratic_policy_input(input, integral_new)
        );

        // unsaturated control
        actuators::ActuatorInputsVector_T<double> u_trim = actuators::unpack_actuator_inputs_T(input.u_sol_trim);
        actuators::ActuatorInputsVector_T<double> u_unsat = u_deviation + u_trim;

        // saturate
        auto [u_min, u_max] = actuators::unpack_actuator_limits(input.surface_actuators, input.propulsor_actuators);
        actuators::ActuatorInputsVector_T<double> u = util::vec_clamp(u_unsat, u_min, u_max);

        // anti-windup
        if (util::vec_is_close(u, u_unsat)) { integral = integral_new; }

        return make_control_output(u);
    }

}
