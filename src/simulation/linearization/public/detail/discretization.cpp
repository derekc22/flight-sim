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
		constants::MatrixX_T<double, constants::nxu, constants::nxu> M =
			constants::MatrixX_T<double, constants::nxu, constants::nxu>::Zero();

		M.block(0, 0, constants::nx, constants::nx) = lin_sol.A;
		M.block(0, constants::nx, constants::nx, constants::nu) = lin_sol.B;

		constants::MatrixX_T<double, constants::nxu, constants::nxu> Md = (M * dt).exp();

		StateJacobian Ak = Md.block(0, 0, constants::nx, constants::nx);
		InputJacobian Bk = Md.block(0, constants::nx, constants::nx, constants::nu);

		// C and D are pass-through
		return {.A = Ak, .B = Bk, .C = lin_sol.C, .D = lin_sol.D};
	}

	DiscretizedLocalLinearization discretize_euler(
		const LocalLinearization& lin_sol,
		double dt)
	{
		StateJacobian Ak = constants::I_T<double, constants::nx> + dt * lin_sol.A;
		InputJacobian Bk = dt * lin_sol.B;

		// C and D are pass-through
		return {.A = Ak, .B = Bk, .C = lin_sol.C, .D = lin_sol.D};
	}

} // namespace linearization
