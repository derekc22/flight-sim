#pragma once
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/control/private/components/controllers/linear_quadratic/lqr.hpp"
#include "simulation/dynamics/public/data/types.hpp"

#include <Eigen/Dense>

namespace control
{

	using IntegratedStateVector = constants::MatrixX_T<double, constants::nxi, 1>;
	using AugmentedStateVector = constants::MatrixX_T<double, constants::nx + constants::nxi, 1>;

	struct LinearQuadraticIntegrator : LinearQuadraticRegulator {
		IntegratedStateVector integral = Eigen::Vector3d::Zero();

		LinearQuadraticIntegrator(const LinearQuadraticIntegratorParameters& params);

		/**
		 * @brief Computes the virtual-control deviation and conditionally updates the integral state.
		 *
		 * @param[in] input Current state, setpoint, trim state, virtual linearization, and allocation residual.
		 * @param[in] dt Integration step [s].
		 * @return Virtual-control deviation with force components [N] and moment components [N m].
		 */
		VirtualControlOutput_T<double> step(const LinearQuadraticControlInput& input, double dt);

		/**
		 * @brief Builds the augmented linear-quadratic policy input.
		 *
		 * @param[in] input Current state, trim state, and virtual linearization.
		 * @param[in] integral_candidate Candidate integral state.
		 * @return Augmented state deviation and augmented virtual-system matrices.
		 */
		LinearQuadraticPolicyInput make_linear_quadratic_policy_input(const LinearQuadraticControlInput& input,
			const IntegratedStateVector& integral_candidate);

		/**
		 * @brief Integrates the roll, pitch, and yaw-rate tracking errors.
		 *
		 * @param[in] zt Current reduced state vector.
		 * @param[in] zt_des Desired reduced state vector.
		 * @param[in] dt Integration step [s].
		 * @return Candidate roll, pitch, and yaw-rate integral state.
		 */
		IntegratedStateVector integrate_state_err(const dynamics::StateVector_T<double>& zt,
			const dynamics::StateVector_T<double>& zt_des,
			double dt);
	};

} // namespace control
