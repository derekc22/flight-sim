#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Solves a continuous-time algebraic Riccati equation through SLICOT SB02MD.
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
