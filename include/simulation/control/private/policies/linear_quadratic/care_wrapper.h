#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void slicot_solve_care_sb02md(int n, const double* a, const double* g, const double* q, double* x, double* rcond, int* info);

#ifdef __cplusplus
}
#endif
