// #include <cmath>
// #include <iostream>
// #include <stdexcept>
// #include "simulation/control/care.hpp"

// int main() {
//     Eigen::MatrixXd A(1, 1);
//     Eigen::MatrixXd B(1, 1);
//     Eigen::MatrixXd Q(1, 1);
//     Eigen::MatrixXd R(1, 1);
//     A << 0.0;
//     B << 1.0;
//     Q << 1.0;
//     R << 1.0;

//     const control::CareSolution care = control::solve_care(A, B, Q, R);
//     const Eigen::MatrixXd K = control::lqr_gain(B, R, care.P);
//     const Eigen::MatrixXd A_cl = A - B * K;

//     if (std::abs(care.P(0, 0) - 1.0) > 1e-10) { throw std::runtime_error("control_care_smoke: unexpected P"); }
//     if (std::abs(K(0, 0) - 1.0) > 1e-10) { throw std::runtime_error("control_care_smoke: unexpected K"); }
//     if (care.residual.norm() > 1e-10) { throw std::runtime_error("control_care_smoke: CARE residual too large"); }
//     if (A_cl(0, 0) >= 0.0) { throw std::runtime_error("control_care_smoke: closed-loop system is not stable"); }

//     std::cout << "P =\n" << care.P << "\n";
//     std::cout << "K =\n" << K << "\n";
//     std::cout << "rcond = " << care.rcond << "\n";
//     std::cout << "residual_norm = " << care.residual.norm() << "\n";
//     std::cout << "A_cl =\n" << A_cl << "\n";
//     return 0;
// }
