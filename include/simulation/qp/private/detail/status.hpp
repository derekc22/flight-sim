#pragma once
#include "simulation/qp/public/data/types.hpp"

#include <proxsuite/proxqp/dense/dense.hpp>

namespace qp
{

	/**
	 * @brief Maps a ProxQP solver status to the repository's QP status type.
	 *
	 * Unrecognized values map to `Status::NotRun`.
	 *
	 * @param[in] status ProxQP solver output status.
	 * @return Corresponding repository QP status.
	 */
	Status map_status(proxsuite::proxqp::QPSolverOutput status);

} // namespace qp
