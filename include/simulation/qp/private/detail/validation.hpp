#pragma once
#include <cstddef>
#include "simulation/qp/public/data/types.hpp"

namespace qp {

	std::size_t validate_dimension(std::size_t dimension);
	void validate_problem(const Problem& problem, std::size_t dimension);

} // namespace qp
