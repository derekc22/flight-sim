#pragma once
#include "simulation/trim/private/data/helpers.hpp"

namespace trim {

	template <typename T>
	TrimResidual_T<T> pack_trim_residual_T(
	    const TrimResidualVector_T<T>& residual)
	{
		return {.vx_dot = residual(0),
		    .vy_dot = residual(1),
		    .vz_dot = residual(2),
		    .p_dot = residual(3),
		    .q_dot = residual(4),
		    .r_dot = residual(5),
		    .phi_dot = residual(6),
		    .theta_dot = residual(7),
		    .beta_err = residual(8),
		    .phi_err = residual(9),
		    .theta_err = residual(10),
		    .vx_err = residual(11),
		    .vz_err = residual(12),
		    .psi_dot_err = residual(13)};
	}

} // namespace trim
