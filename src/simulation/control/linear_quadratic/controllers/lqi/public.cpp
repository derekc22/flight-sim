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

    IntegratedStateVector LinearQuadraticIntegrator::integrate_state_err(const dynamics::StateVector_T<double>& zt, const dynamics::StateVector_T<double>& zt_des, double dt) {
        IntegratedStateVector zt_pqr = zt.segment<integrated_state_dim>(3);  // grab p, q, r
        IntegratedStateVector zt_des_pqr = zt_des.segment<integrated_state_dim>(3);

        return integral + (zt_des_pqr - zt_pqr) * dt;     // integrate
    }


    LinearQuadraticPolicyInput LinearQuadraticIntegrator::make_linear_quadratic_policy_input(const LinearQuadraticControllerInput& input, const IntegratedStateVector& integral_new) {
        size_t n = constants::state_dim;
        size_t m = constants::input_dim;
        size_t i = integrated_state_dim;

        Eigen::MatrixXd A_aug = Eigen::MatrixXd::Zero(n + i, n + i);
        A_aug.block(0, 0, n, n) = input.A;

        // Ci selects the integrated states p, q, r from the state vector for the LQI controller - it is not the system-wide output matrix C
        Eigen::MatrixXd Ci = Eigen::MatrixXd::Zero(integrated_state_dim, constants::state_dim);
        Ci.block<integrated_state_dim, integrated_state_dim>(0, 3) = constants::IX_T<double, integrated_state_dim>;
        A_aug.block(n, 0, i, n) = -Ci;

        Eigen::MatrixXd B_aug = Eigen::MatrixXd::Zero(n + i, m);
        B_aug.block(0, 0, n, m) = input.B;

        dynamics::StateVector_T<double> zt = dynamics::unpack_state(input.Zt);
        dynamics::StateVector_T<double> zt_des = unpack_state(input.setpoint);

        AugmentedStateVector zt_aug;
        zt_aug << zt, integral_new;

        AugmentedStateVector zt_des_aug;
        zt_des_aug << zt_des, IntegratedStateVector::Zero();

        AugmentedStateVector zt_aug_deviation = zt_aug - zt_des_aug;

        return {
            .zt = zt_aug_deviation,
            .A = A_aug,
            .B = B_aug
        };
    }

    ControlOutput LinearQuadraticIntegrator::step(const LinearQuadraticControllerInput& input, double dt) {

        // integral candidate
        IntegratedStateVector integral_new = integrate_state_err(
            dynamics::unpack_state(input.Zt),
            unpack_state(input.setpoint),
            dt
        );

        actuators::ActuatorInputsVector_T<double> u_deviation = policy.step(
            make_linear_quadratic_policy_input(input, integral_new)
        );

        // unsaturated control
        actuators::ActuatorInputsVector_T<double> u_trim = actuators::unpack_actuator_inputs_T(input.u_sol_trim);
        actuators::ActuatorInputsVector_T<double> u_unsat = u_deviation + u_trim;

        // saturate
        actuators::ActuatorLimitsVector limits = actuators::unpack_actuator_limits(input.surface_actuators, input.propulsor_actuators);
        actuators::ActuatorInputsVector_T<double> u = util::vec_clamp(u_unsat, limits.col(0), limits.col(1));

        // anti-windup
        if (util::vec_is_close(u, u_unsat)) { integral = integral_new; }

        return make_control_output(u);
    }

}
