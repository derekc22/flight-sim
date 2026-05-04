// #include "simulation/estimation/lkf.hpp"
// #include "simulation/linearization/linearization.hpp"
// #include "simulation/constants/constants.hpp"
// #include "simulation/constants/constants.hpp"

// namespace estimation {

//     LinearKalmanFilter::LinearKalmanFilter(
//         const Eigen::VectorXd& x0, 
//         const Eigen::MatrixXd& A, 
//         const Eigen::MatrixXd& B, 
//         const Eigen::MatrixXd& C, 
//         const Eigen::MatrixXd& P0,
//         const Eigen::MatrixXd& Q0, 
//         const Eigen::MatrixXd& R0
//     ) : n(A.rows()), m(B.cols()), k(C.rows()) {

//         validate_init(x0, B, C, P0, Q0, R0);

//         this->xt_1 = x0;
//         this->A = A;
//         this->B = B;
//         this->C = C;
//         this->Pt_1 = P0;
//         this->Qt = Q0;
//         this->Rt = R0;
//     }

//     void LinearKalmanFilter::validate_init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& P0, const Eigen::MatrixXd& Q0, const Eigen::MatrixXd& R0) {
//         std::string err_scope = "estimation::LinearKalmanFilter";

//         util::validate_value(n, constants::state_dim, err_scope, "n", "state_dim");
//         util::validate_value(m, constants::input_dim, err_scope, "m", "input_dim");
//         util::validate_value(k, constants::state_dim, err_scope, "k", "state_dim");

//         util::validate_shape(x0, n, 1, err_scope, "x0");
//         util::validate_shape(B, n, m, err_scope, "B");
//         util::validate_shape(C, k, n, err_scope, "C");
//         util::validate_shape(P0, n, n, err_scope, "P0");
//         util::validate_shape(Q0, k, k, err_scope, "Q0");
//         util::validate_shape(R0, n, n, err_scope, "R0");
//     }

//     KalmanState LinearKalmanFilter::predict(const Eigen::VectorXd& ut_1) {

//         Eigen::VectorXd xt_bar = A * xt_1 + B * ut_1;

//         Eigen::MatrixXd Pt_bar = A * Pt_1 * A.transpose() + Rt;

//         return { xt_bar, Pt_bar };
//     }

//     KalmanState LinearKalmanFilter::correct(const KalmanState& pred, const Eigen::VectorXd& zt) {

//         Eigen::MatrixXd Kt = pred.P * C.transpose() * (C * pred.P * C.transpose() + Qt).inverse(); // Kalman gain

//         Eigen::VectorXd yt = zt - C * pred.x; // Innovation

//         Eigen::VectorXd xt = pred.x + Kt * yt;

//         Eigen::MatrixXd Inxn = Eigen::MatrixXd::Identity(n, n);

//         Eigen::MatrixXd Pt = ( Inxn - Kt * C ) * pred.P * ( Inxn - Kt * C ).transpose() + Kt * Qt * Kt.transpose();

//         xt_1 = xt;
//         Pt_1 = Pt;

//         return { xt, Pt };
//     }

//     KalmanState LinearKalmanFilter::step(const Eigen::VectorXd& zt, const Eigen::VectorXd& ut_1) {
//         std::string err_scope = "estimation::LinearKalmanFilter::step";
//         util::validate_shape(zt, k, 1, err_scope, "zt");
//         util::validate_shape(ut_1, m, 1, err_scope, "ut_1");
//         return correct(predict(ut_1), zt);
//     }





// }