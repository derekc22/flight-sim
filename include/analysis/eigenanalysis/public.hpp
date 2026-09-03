#pragma once
#include <Eigen/Dense>
#include <complex>
#include <string>
#include "simulation/constants/public/linalg.hpp"
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/linearization/public/data/types.hpp"

namespace analysis {
    using Eigenvalues = constants::MatrixX_T<std::complex<double>, constants::state_dim, 1>;
    using Eigenvectors = constants::MatrixX_T<std::complex<double>, constants::state_dim, constants::state_dim>;

    struct EigenAnalysis {
        Eigenvalues eigenvalues = Eigenvalues::Zero();
        Eigenvectors eigenvectors = Eigenvectors::Zero();
    };

    EigenAnalysis linearization_eigen_analysis(const linearization::LocalLinearization& lin_sol);

    std::string print_eigen_analysis(const EigenAnalysis& eig);

}
