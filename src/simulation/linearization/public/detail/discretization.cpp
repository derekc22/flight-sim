#include "simulation/linearization/public/detail/discretization.hpp"

#include "simulation/constants/public/dimensions.hpp"
#include "simulation/constants/public/linalg.hpp"

#include <unsupported/Eigen/MatrixFunctions>

namespace linearization
{

	DiscretizedLocalLinearization discretize(
		const LocalLinearization& lin_sol,
		double dt)
	{
		Eigen::MatrixXd M = Eigen::MatrixXd::Zero(
			constants::state_dim + constants::input_dim, constants::state_dim + constants::input_dim);

		M.block(0, 0, constants::state_dim, constants::state_dim) = lin_sol.A;
		M.block(0, constants::state_dim, constants::state_dim, constants::input_dim) = lin_sol.B;

		Eigen::MatrixXd Md = (M * dt).exp();

		Eigen::MatrixXd Ak = Md.block(0, 0, constants::state_dim, constants::state_dim);
		Eigen::MatrixXd Bk = Md.block(0, constants::state_dim, constants::state_dim, constants::input_dim);

		// C and D are pass-through
		return {.A = Ak, .B = Bk, .C = lin_sol.C, .D = lin_sol.D};
	}

	DiscretizedLocalLinearization discretize_euler(
		const LocalLinearization& lin_sol,
		double dt)
	{
		Eigen::MatrixXd Ak = constants::I_T<double, constants::state_dim> + dt * lin_sol.A;
		Eigen::MatrixXd Bk = dt * lin_sol.B;

		// C and D are pass-through
		return {.A = Ak, .B = Bk, .C = lin_sol.C, .D = lin_sol.D};
	}

} // namespace linearization
