#pragma once

#include <Eigen/Dense>

namespace control {

    struct CareSolution {
        Eigen::MatrixXd P;
        Eigen::MatrixXd residual;
        double rcond = 0.0;
    };

    CareSolution solve_care_sb02md(const Eigen::Ref<const Eigen::MatrixXd>& A, const Eigen::Ref<const Eigen::MatrixXd>& G, const Eigen::Ref<const Eigen::MatrixXd>& Q);
    CareSolution solve_care_lqr(const Eigen::Ref<const Eigen::MatrixXd>& A, const Eigen::Ref<const Eigen::MatrixXd>& B, const Eigen::Ref<const Eigen::MatrixXd>& Q, const Eigen::Ref<const Eigen::MatrixXd>& R);
    Eigen::MatrixXd lqr_gain(const Eigen::Ref<const Eigen::MatrixXd>& B, const Eigen::Ref<const Eigen::MatrixXd>& R, const Eigen::Ref<const Eigen::MatrixXd>& P);

}
