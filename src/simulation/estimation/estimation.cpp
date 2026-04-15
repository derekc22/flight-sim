#include <Eigen/Dense>
#include <cmath>
#include <tuple>
#include <algorithm>
#include <stdexcept>
#include "simulation/estimation/estimation.hpp"
#include "simulation/util/validate.hpp"

namespace estimation {

    KalmanFilter::KalmanFilter(
        const Eigen::VectorXd& x0, 
        const Eigen::MatrixXd& A, 
        const Eigen::MatrixXd& B, 
        const Eigen::MatrixXd& C, 
        const Eigen::MatrixXd& P0,
        const Eigen::MatrixXd& Q0, 
        const Eigen::MatrixXd& R0
    ) : n(A.rows()), m(B.cols()), k(C.rows()) {

        _validate_init(x0, B, C, P0, Q0, R0);

        this->xt = x0;
        this->A = A;
        this->B = B;
        this->C = C;
        this->Pt = P0;
        this->Qt = Q0;
        this->Rt = R0;
    }

    void KalmanFilter::_validate_init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& P0, const Eigen::MatrixXd& Q0, const Eigen::MatrixXd& R0) {
        std::string err_context = "estimation::KalmanFilter";
        util::validate_shape(x0, n, 1, err_context, "x0");
        util::validate_shape(B, n, m, err_context, "B");
        util::validate_shape(C, k, n, err_context, "C");
        util::validate_shape(P0, n, n, err_context, "P0");
        util::validate_shape(Q0, k, k, err_context, "Q0");
        util::validate_shape(R0, n, n, err_context, "R0");
    }

    KalmanState KalmanFilter::_predict(const Eigen::VectorXd& u) {

        Eigen::VectorXd xt1_bar = A * xt + B * u;

        Eigen::MatrixXd Pt1_bar = A * Pt * A.transpose() + Rt;

        return { xt1_bar, Pt1_bar };
    }

    KalmanState KalmanFilter::_correct(const KalmanState& pred, const Eigen::VectorXd& zt) {

        Eigen::MatrixXd Kt = pred.P * C.transpose() * (C * pred.P * C.transpose() + Qt).inverse(); // Kalman gain

        Eigen::VectorXd y = zt - C * pred.x; // Innovation

        Eigen::VectorXd xt1 = pred.x + Kt * y;

        Eigen::MatrixXd Inxn = Eigen::MatrixXd::Identity(n, n);

        Eigen::MatrixXd Pt1 = ( Inxn - Kt * C ) * pred.P * ( Inxn - Kt * C ).transpose() + Kt * Qt * Kt.transpose();

        xt = xt1;
        Pt = Pt1;

        return { xt1, Pt1 };
    }

    KalmanState KalmanFilter::step(const Eigen::VectorXd& zt, const Eigen::VectorXd& u) {
        std::string err_context = "estimation::KalmanFilter::step";
        util::validate_shape(zt, k, 1, err_context, "zt");
        util::validate_shape(u, m, 1, err_context, "u");
        return _correct(_predict(u), zt);
    }




}
