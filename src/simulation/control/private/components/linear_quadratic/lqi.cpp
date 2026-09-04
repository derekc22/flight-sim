#include "simulation/control/private/components/linear_quadratic/lqi.hpp"

#include "simulation/constants/public/dimensions.hpp"
#include "simulation/constants/public/scalars.hpp"
#include "simulation/control/private/components/linear_quadratic/helpers.hpp"
#include "simulation/control/private/components/linear_quadratic/lqr.hpp"
#include "simulation/dynamics/public/data/helpers.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/linearization/public/data/types.hpp"
#include "simulation/util/public/math.hpp"

#include <Eigen/Dense>
#include <algorithm>

namespace control
{

	LinearQuadraticIntegrator::LinearQuadraticIntegrator(
		const LinearQuadraticIntegratorParameters& params)
		: LinearQuadraticRegulator(params)
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

	IntegratedStateVector LinearQuadraticIntegrator::integrate_state_err(
		const dynamics::StateVector_T<double>& zt,
		const dynamics::StateVector_T<double>& zt_des,
		double dt)
	{
		IntegratedStateVector zt_integrated; // grab phi, theta, r
		zt_integrated << zt(6), zt(7), zt(5);

		IntegratedStateVector zt_des_integrated;
		zt_des_integrated << zt_des(6), zt_des(7), zt_des(5);

		return integral + (zt_des_integrated - zt_integrated) * dt; // integrate
	}

	LinearQuadraticPolicyInput LinearQuadraticIntegrator::make_linear_quadratic_policy_input(
		const LinearQuadraticControlInput& input,
		const IntegratedStateVector& integral_new)
	{
		size_t n = constants::state_dim;
		size_t m = constants::virtual_input_dim;
		size_t i = integrated_state_dim;

		Eigen::MatrixXd A_virtual_aug = Eigen::MatrixXd::Zero(n + i, n + i);
		A_virtual_aug.block(0, 0, n, n) = input.virtual_linearization.A_virtual;

		// Ci selects the integrated states phi, theta, r from the state vector for the LQI controller - it is not the canonical output matrix C
		Eigen::MatrixXd Ci = Eigen::MatrixXd::Zero(integrated_state_dim, constants::state_dim);
		Ci(0, 6) = 1.0;
		Ci(1, 7) = 1.0;
		Ci(2, 5) = 1.0;
		A_virtual_aug.block(n, 0, i, n) = -Ci;

		Eigen::MatrixXd B_virtual_aug = Eigen::MatrixXd::Zero(n + i, m);
		B_virtual_aug.block(0, 0, n, m) = input.virtual_linearization.B_virtual;

		dynamics::StateVector_T<double> zt = dynamics::unpack_state(input.Zt);
		dynamics::StateVector_T<double> zt_trim = dynamics::unpack_state_T(input.Z_sol_trim);

		AugmentedStateVector zt_aug;
		zt_aug << zt, integral_new;

		AugmentedStateVector zt_trim_aug;
		zt_trim_aug << zt_trim, IntegratedStateVector::Zero();

		AugmentedStateVector zt_aug_deviation = zt_aug - zt_trim_aug;

		return {.zt = zt_aug_deviation, .A_virtual = A_virtual_aug, .B_virtual = B_virtual_aug};
	}

	VirtualControlOutput_T<double> LinearQuadraticIntegrator::step(
		const LinearQuadraticControlInput& input,
		double dt)
	{

		// integral candidate
		IntegratedStateVector integral_new =
			integrate_state_err(dynamics::unpack_state(input.Zt), unpack_state(input.setpoint), dt);

		VirtualControlOutputVector_T<double> mu_deviation =
			policy.step(make_linear_quadratic_policy_input(input, integral_new));

		if (input.delta_mu_vec_t_1.norm() <= constants::eps) {
			integral = integral_new;
		}

		return dynamics::pack_wrench_T(mu_deviation);
	}

} // namespace control
