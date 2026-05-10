#include <unsupported/Eigen/MatrixFunctions>
#include "simulation/constants/constants.hpp"
#include "simulation/linearization/shared.hpp"

namespace linearization {

    DiscretizedTrimLinearization discretize(const linearization::TrimLinearization& lin_sol){
        int nx = lin_sol.A.rows();
        int nu = lin_sol.B.cols();

        Eigen::MatrixXd M = Eigen::MatrixXd::Zero(nx + nu, nx + nu);

        M.block(0, 0, nx, nx) = lin_sol.A;
        M.block(0, nx, nx, nu) = lin_sol.B;

        Eigen::MatrixXd Md = (M * constants::dt).exp();

        Eigen::MatrixXd Ak = Md.block(0, 0, nx, nx);
        Eigen::MatrixXd Bk = Md.block(0, nx, nx, nu);

        return { .A = Ak, .B = Bk };
    }

}
