#pragma once
#include <Eigen/Dense>
#include "simulation/control/private/data/types.hpp"

namespace control {

	Eigen::MatrixXd symmetrize(const Eigen::Ref<const Eigen::MatrixXd>& M);
	Eigen::LDLT<Eigen::MatrixXd> factorize_symmetric(const Eigen::Ref<const Eigen::MatrixXd>& M, const char* name);
	Eigen::MatrixXd care_residual(const Eigen::Ref<const Eigen::MatrixXd>& A,
	    const Eigen::Ref<const Eigen::MatrixXd>& G,
	    const Eigen::Ref<const Eigen::MatrixXd>& Q,
	    const Eigen::Ref<const Eigen::MatrixXd>& P);
	CareSolution solve_care_sb02md(const Eigen::Ref<const Eigen::MatrixXd>& A,
	    const Eigen::Ref<const Eigen::MatrixXd>& G,
	    const Eigen::Ref<const Eigen::MatrixXd>& Q);
	CareSolution solve_care(const Eigen::Ref<const Eigen::MatrixXd>& A,
	    const Eigen::Ref<const Eigen::MatrixXd>& B,
	    const Eigen::Ref<const Eigen::MatrixXd>& Q,
	    const Eigen::Ref<const Eigen::MatrixXd>& R);
	Eigen::MatrixXd lqr_gain(const Eigen::Ref<const Eigen::MatrixXd>& B,
	    const Eigen::Ref<const Eigen::MatrixXd>& R,
	    const Eigen::Ref<const Eigen::MatrixXd>& P);

} // namespace control
