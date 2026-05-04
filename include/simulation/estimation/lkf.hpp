// #pragma once

// #include <Eigen/Dense>

// namespace estimation {

//     struct LinearKalmanFilter {

//         Eigen::VectorXd xt_1;   // state estimate
//         Eigen::MatrixXd A;      // state transition matrix
//         Eigen::MatrixXd B;      // input matrix
//         Eigen::MatrixXd C;      // measurement matrix
//         Eigen::MatrixXd Pt_1;   // state estimate error covariance matrix
//         Eigen::MatrixXd Rt;     // process noise covariance matrix
//         Eigen::MatrixXd Qt;     // measurement noise covariance matrix

//         int n;
//         int m;
//         int k;

//         LinearKalmanFilter(
//             const Eigen::VectorXd& x0, 
//             const Eigen::MatrixXd& A, 
//             const Eigen::MatrixXd& B, 
//             const Eigen::MatrixXd& C, 
//             const Eigen::MatrixXd& P0, 
//             const Eigen::MatrixXd& Q0, 
//             const Eigen::MatrixXd& R0 
//         );

//         void validate_init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& P0, const Eigen::MatrixXd& Q0, const Eigen::MatrixXd& R0);

//         KalmanState predict(const Eigen::VectorXd& u);

//         KalmanState correct(const KalmanState& pred, const Eigen::VectorXd& zt);

//         KalmanState _step(const Eigen::VectorXd& zt, const Eigen::VectorXd& ut_1);


//     };
// }
