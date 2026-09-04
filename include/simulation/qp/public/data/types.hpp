#pragma once
#include <Eigen/Dense>

namespace qp
{

	enum class Status {
		Solved,
		MaxIterations,
		PrimalInfeasible,
		DualInfeasible,
		ClosestPrimalFeasible,
		NotRun
	};

	struct Problem {
		Eigen::MatrixXd hessian;
		Eigen::VectorXd gradient;
		Eigen::VectorXd lower;
		Eigen::VectorXd upper;
	};

	struct Solution {
		Eigen::VectorXd x;
		Status status = Status::NotRun;
	};

} // namespace qp
