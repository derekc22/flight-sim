#include <Eigen/Dense>
#include <cmath>
#include <tuple>
#include <algorithm>

namespace estimation {

    struct KalmanState {
        Eigen::VectorXd x;
        Eigen::MatrixXd P;
    };


    struct KalmanFilter {

        Eigen::VectorXd xt; // state estimate
        Eigen::MatrixXd A;  // state transition matrix
        Eigen::MatrixXd B;  // input matrix
        Eigen::MatrixXd C;  // measurement matrix
        Eigen::MatrixXd Pt; // state estimate error covariance matrix
        Eigen::MatrixXd Rt; // process noise covariance matrix
        Eigen::MatrixXd Qt; // measurement noise covariance matrix

        int n;
        int m;
        int k;

        KalmanFilter(
            const Eigen::VectorXd& x0, 
            const Eigen::MatrixXd& A, 
            const Eigen::MatrixXd& B, 
            const Eigen::MatrixXd& C, 
            const Eigen::MatrixXd& R0, 
            const Eigen::MatrixXd& Q0, 
            const Eigen::MatrixXd& P0 
        );

        KalmanState _predict(const Eigen::VectorXd& u);

        KalmanState _correct(const KalmanState& pred, const Eigen::VectorXd& zt);

        KalmanState step(const Eigen::VectorXd& zt, const Eigen::VectorXd& u);

        

    };




}