#include <stdexcept>
#include "simulation/qp/private/detail/validation.hpp"

namespace qp {

	std::size_t validate_dimension(
	    std::size_t dimension)
	{
		if (dimension == 0)
			throw std::invalid_argument("qp::Solver dimension must be positive");
		return dimension;
	}

	void validate_problem(
	    const Problem& problem,
	    std::size_t dimension)
	{
		const Eigen::Index n = static_cast<Eigen::Index>(dimension);
		if (problem.hessian.rows() != n || problem.hessian.cols() != n)
			throw std::invalid_argument("qp::Solver::solve hessian dimensions must match solver dimension");
		if (problem.gradient.size() != n)
			throw std::invalid_argument("qp::Solver::solve gradient dimension must match solver dimension");
		if (problem.lower.size() != n || problem.upper.size() != n)
			throw std::invalid_argument("qp::Solver::solve bound dimensions must match solver dimension");
		if (!problem.hessian.allFinite() || !problem.gradient.allFinite())
			throw std::invalid_argument("qp::Solver::solve objective must be finite");
		if (problem.lower.array().isNaN().any() || problem.upper.array().isNaN().any())
			throw std::invalid_argument("qp::Solver::solve bounds cannot contain NaN");
		if ((problem.lower.array() > problem.upper.array()).any())
			throw std::invalid_argument("qp::Solver::solve lower bounds cannot exceed upper bounds");
		if (!problem.hessian.isApprox(problem.hessian.transpose()))
			throw std::invalid_argument("qp::Solver::solve hessian must be symmetric");
	}

} // namespace qp
