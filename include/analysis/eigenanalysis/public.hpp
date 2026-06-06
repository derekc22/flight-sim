#pragma once

#include <Eigen/Dense>
#include <complex>
#include <string>
#include "simulation/constants/public.hpp"
#include "simulation/linearization/public.hpp"

namespace analysis {
    using Eigenvalues = Eigen::Matrix<std::complex<double>, constants::state_dim, 1>;
    using Eigenvectors = Eigen::Matrix<std::complex<double>, constants::state_dim, constants::state_dim>;

    struct EigenAnalysis {
        Eigenvalues eigenvalues = Eigenvalues::Zero();
        Eigenvectors eigenvectors = Eigenvectors::Zero();
    };

    EigenAnalysis linearization_eigen_analysis(const linearization::LocalLinearization& lin);

    std::string print_eigen_analysis(const EigenAnalysis& eig);

}
