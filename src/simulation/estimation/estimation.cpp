#include <Eigen/Dense>
#include <cmath>
#include <tuple>
#include <algorithm>
#include <string>
#include <format>
#include <stdexcept>
#include "simulation/estimation/estimation.hpp"

namespace estimation {


    KalmanFilter::KalmanFilter(
        const Eigen::VectorXd& x0, 
        const Eigen::MatrixXd& A, 
        const Eigen::MatrixXd& B, 
        const Eigen::MatrixXd& C, 
        const Eigen::MatrixXd& R0, 
        const Eigen::MatrixXd& Q0, 
        const Eigen::MatrixXd& P0 
    ) : n(A.rows()), m(B.cols()), k(C.rows()) {

        std::string x0_err_msg = std::format("estimation::KalmanFilter: Incorrect shape for state, x0. Must be ({} x 1)", n);
        if (!(x0.rows() == n && x0.cols() == 1)) throw std::invalid_argument(x0_err_msg);

        std::string B_err_msg = std::format("estimation::KalmanFilter: Incorrect shape for input matrix, B. Must be ({} x {})", n, m);
        if (!(B.rows() == n)) throw std::invalid_argument(B_err_msg);

        std::string C_err_msg = std::format("estimation::KalmanFilter: Incorrect shape for measurement matrix, C. Must be ({} x {})", k, n);
        if (!(C.cols() == n)) throw std::invalid_argument(C_err_msg);

        std::string R0_err_msg = std::format("estimation::KalmanFilter: Incorrect shape for process noise covariance matrix, R0. Must be ({} x {})", n, n);
        if (!(R0.rows() == n && R0.cols() == n)) throw std::invalid_argument(R0_err_msg);

        std::string Q0_err_msg = std::format("estimation::KalmanFilter: Incorrect shape for measurement noise covariance matrix, Q0. Must be ({} x {})", k, k);
        if (!(Q0.rows() == k && Q0.cols() == k)) throw std::invalid_argument(Q0_err_msg);

        std::string P0_err_msg = std::format("estimation::KalmanFilter: Incorrect shape for state estimate error covariance matrix, P0. Must be ({} x {})", n, n);
        if (!(P0.rows() == n && P0.cols() == n)) throw std::invalid_argument(P0_err_msg);

        this->xt = x0;
        this->A = A;
        this->B = B;
        this->C = C;
        this->Pt = P0;
        this->Rt = R0;
        this->Qt = Q0;

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

        std::string zt_err_msg = std::format("estimation::KalmanFilter::step: Incorrect shape for measurement, zt. Must be ({} x 1)", k);
        if (!(zt.rows() == k && zt.cols() == 1)) throw std::invalid_argument(zt_err_msg);

        std::string u_err_msg = std::format("estimation::KalmanFilter::step: Incorrect shape for control input, u. Must be ({} x 1)", m);
        if (!(u.rows() == m && u.cols() == 1)) throw std::invalid_argument(u_err_msg);

        return _correct(_predict(u), zt);
    }




}