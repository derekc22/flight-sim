#include <stdexcept>
#include "simulation/control/linear_quadratic/care.hpp"
#include "simulation/control/linear_quadratic/slicot_care_wrapper.h"
#include "simulation/util/validate.hpp"

namespace control {

    static Eigen::MatrixXd _symmetrize(const Eigen::Ref<const Eigen::MatrixXd>& M) {
        return 0.5 * (M + M.transpose());
    }

    static Eigen::LDLT<Eigen::MatrixXd> _factorize_symmetric(const Eigen::Ref<const Eigen::MatrixXd>& M, const char* name) {
        Eigen::LDLT<Eigen::MatrixXd> ldlt(_symmetrize(M));
        if (ldlt.info() != Eigen::Success) { throw std::runtime_error(std::string("control::") + name + " factorization failed"); }
        return ldlt;
    }

    static Eigen::MatrixXd _care_residual(const Eigen::Ref<const Eigen::MatrixXd>& A, const Eigen::Ref<const Eigen::MatrixXd>& G, const Eigen::Ref<const Eigen::MatrixXd>& Q, const Eigen::Ref<const Eigen::MatrixXd>& P) {
        return A.transpose() * P + P * A - P * G * P + Q;
    }

    CareSolution solve_care_sb02md(const Eigen::Ref<const Eigen::MatrixXd>& A, const Eigen::Ref<const Eigen::MatrixXd>& G, const Eigen::Ref<const Eigen::MatrixXd>& Q) {
        util::validate_sqaure(A, "control::solve_care_sb02md", "A");
        util::validate_sqaure(G, "control::solve_care_sb02md", "G");
        util::validate_sqaure(Q, "control::solve_care_sb02md", "Q");
        util::validate_shape(G, A.rows(), A.cols(), "control::solve_care_sb02md", "G");
        util::validate_shape(Q, A.rows(), A.cols(), "control::solve_care_sb02md", "Q");

        const int n = static_cast<int>(A.rows());
        Eigen::MatrixXd A_work = A;
        Eigen::MatrixXd G_work = _symmetrize(G);
        Eigen::MatrixXd Q_work = _symmetrize(Q);
        Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n, n);
        double rcond = 0.0;
        int info = 0;

        slicot_solve_care_sb02md(n, A_work.data(), G_work.data(), Q_work.data(), P.data(), &rcond, &info);
        if (info != 0) { throw std::runtime_error("control::solve_care_sb02md failed with INFO = " + std::to_string(info)); }
        P = _symmetrize(P);
        return { .P = P, .residual = _care_residual(A, G_work, Q, P), .rcond = rcond };
    }

    CareSolution solve_care(const Eigen::Ref<const Eigen::MatrixXd>& A, const Eigen::Ref<const Eigen::MatrixXd>& B, const Eigen::Ref<const Eigen::MatrixXd>& Q, const Eigen::Ref<const Eigen::MatrixXd>& R) {
        util::validate_sqaure(A, "control::solve_care", "A");
        util::validate_sqaure(Q, "control::solve_care", "Q");
        util::validate_sqaure(R, "control::solve_care", "R");
        util::validate_shape(Q, A.rows(), A.cols(), "control::solve_care", "Q");
        util::validate_shape(B, A.rows(), R.rows(), "control::solve_care", "B");

        const Eigen::LDLT<Eigen::MatrixXd> R_ldlt = _factorize_symmetric(R, "R");
        const Eigen::MatrixXd G = B * R_ldlt.solve(B.transpose());
        if (R_ldlt.info() != Eigen::Success || !G.allFinite()) { throw std::runtime_error("control::solve_care failed to form G = B R^{-1} B^T"); }
        return solve_care_sb02md(A, G, Q);
    }

    Eigen::MatrixXd lqr_gain(const Eigen::Ref<const Eigen::MatrixXd>& B, const Eigen::Ref<const Eigen::MatrixXd>& R, const Eigen::Ref<const Eigen::MatrixXd>& P) {
        util::validate_sqaure(R, "control::lqr_gain", "R");
        util::validate_sqaure(P, "control::lqr_gain", "P");
        util::validate_shape(B, P.rows(), R.rows(), "control::lqr_gain", "B");

        const Eigen::LDLT<Eigen::MatrixXd> R_ldlt = _factorize_symmetric(R, "R");
        const Eigen::MatrixXd K = R_ldlt.solve(B.transpose() * P);
        if (R_ldlt.info() != Eigen::Success || !K.allFinite()) { throw std::runtime_error("control::lqr_gain failed to solve for K"); }
        return K;
    }

}
