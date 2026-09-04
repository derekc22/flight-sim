#include "simulation/linearization/public/detail/discretization.hpp"

#include <unsupported/Eigen/MatrixFunctions>

namespace linearization
{

	DiscretizedLocalLinearization discretize(
		const LocalLinearization& lin_sol,
		double dt)
	{
		int nx = lin_sol.A.rows();
		int nu = lin_sol.B.cols();

		Eigen::MatrixXd M = Eigen::MatrixXd::Zero(nx + nu, nx + nu);

		M.block(0, 0, nx, nx) = lin_sol.A;
		M.block(0, nx, nx, nu) = lin_sol.B;

		Eigen::MatrixXd Md = (M * dt).exp();

		Eigen::MatrixXd Ak = Md.block(0, 0, nx, nx);
		Eigen::MatrixXd Bk = Md.block(0, nx, nx, nu);

		// C and D are pass-through
		return {.A = Ak, .B = Bk, .C = lin_sol.C, .D = lin_sol.D};
	}

	DiscretizedLocalLinearization discretize_euler(
		const LocalLinearization& lin_sol,
		double dt)
	{
		int nx = lin_sol.A.rows();

		Eigen::MatrixXd Ak = Eigen::MatrixXd::Identity(nx, nx) + dt * lin_sol.A;
		Eigen::MatrixXd Bk = dt * lin_sol.B;

		// C and D are pass-through
		return {.A = Ak, .B = Bk, .C = lin_sol.C, .D = lin_sol.D};
	}

} // namespace linearization
