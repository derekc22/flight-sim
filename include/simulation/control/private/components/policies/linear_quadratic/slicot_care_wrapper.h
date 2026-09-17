#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Solves a continuous-time algebraic Riccati equation through SLICOT SB02MD.
 *
 * The matrices use column-major storage and have dimensions @p n by @p n.
 * The solver symmetrizes G and Q, then computes X for
 * A^T X + X A - X G X + Q = 0. On failure, X is set to zero.
 *
 * @param[in] n Order of the equation matrices.
 * @param[in] a State-matrix data.
 * @param[in] g Riccati quadratic-term matrix data.
 * @param[in] q State-weighting matrix data.
 * @param[out] x Riccati solution matrix data.
 * @param[out] rcond Reciprocal condition estimate returned by SB02MD.
 * @param[out] info SB02MD status code; zero indicates success.
 */
void slicot_solve_care_sb02md(int n, const double* a, const double* g, const double* q, double* x, double* rcond, int* info);

#ifdef __cplusplus
}
#endif
