#pragma once
#include "simulation/control/private/data/types.hpp"

#include <Eigen/Dense>

namespace control
{

	/**
	 * @brief Returns the symmetric part of a matrix.
	 *
	 * @param[in] M Matrix to symmetrize.
	 * @return Matrix given by one half of @p M plus its transpose.
	 */
	Eigen::MatrixXd symmetrize(const Eigen::Ref<const Eigen::MatrixXd>& M);

	/**
	 * @brief Computes an LDLT factorization of a matrix's symmetric part.
	 *
	 * @param[in] M Matrix to symmetrize and factorize.
	 * @param[in] name Matrix name used in failure diagnostics.
	 * @return LDLT factorization of the symmetric part of @p M.
	 */
	Eigen::LDLT<Eigen::MatrixXd> factorize_symmetric(const Eigen::Ref<const Eigen::MatrixXd>& M, const char* name);

	/**
	 * @brief Evaluates a continuous-time algebraic Riccati equation residual.
	 *
	 * @param[in] A State matrix.
	 * @param[in] G Riccati quadratic-term matrix.
	 * @param[in] Q State-weighting matrix.
	 * @param[in] P Candidate Riccati solution.
	 * @return Riccati residual matrix.
	 */
	Eigen::MatrixXd care_residual(const Eigen::Ref<const Eigen::MatrixXd>& A,
		const Eigen::Ref<const Eigen::MatrixXd>& G,
		const Eigen::Ref<const Eigen::MatrixXd>& Q,
		const Eigen::Ref<const Eigen::MatrixXd>& P);

	/**
	 * @brief Solves a continuous-time algebraic Riccati equation with SLICOT SB02MD.
	 *
	 * @param[in] A Square state matrix.
	 * @param[in] G Square Riccati quadratic-term matrix with the same order as @p A.
	 * @param[in] Q Square state-weighting matrix with the same order as @p A.
	 * @return Riccati solution, residual, and reciprocal condition estimate.
	 */
	CareSolution solve_care_sb02md(const Eigen::Ref<const Eigen::MatrixXd>& A,
		const Eigen::Ref<const Eigen::MatrixXd>& G,
		const Eigen::Ref<const Eigen::MatrixXd>& Q);

	/**
	 * @brief Solves the continuous-time algebraic Riccati equation for an input matrix.
	 *
	 * @param[in] A Square state matrix.
	 * @param[in] B Input matrix.
	 * @param[in] Q Square state-weighting matrix with the same order as @p A.
	 * @param[in] R Square input-weighting matrix with the same column count as @p B.
	 * @return Riccati solution, residual, and reciprocal condition estimate.
	 */
	CareSolution solve_care(const Eigen::Ref<const Eigen::MatrixXd>& A,
		const Eigen::Ref<const Eigen::MatrixXd>& B,
		const Eigen::Ref<const Eigen::MatrixXd>& Q,
		const Eigen::Ref<const Eigen::MatrixXd>& R);

	/**
	 * @brief Computes a continuous-time linear-quadratic feedback gain.
	 *
	 * @param[in] B Input matrix.
	 * @param[in] R Square input-weighting matrix.
	 * @param[in] P Square Riccati solution matrix.
	 * @return State-feedback gain matrix.
	 */
	Eigen::MatrixXd lqr_gain(const Eigen::Ref<const Eigen::MatrixXd>& B,
		const Eigen::Ref<const Eigen::MatrixXd>& R,
		const Eigen::Ref<const Eigen::MatrixXd>& P);

} // namespace control
