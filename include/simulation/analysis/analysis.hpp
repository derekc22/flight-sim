#pragma once

#include <Eigen/Dense>
#include <complex>
#include <string>
#include "simulation/constants/constants.hpp"
#include "simulation/linearization/linearization.hpp"

namespace analysis {
    using TrimEigenvalues = Eigen::Matrix<std::complex<double>, constants::state_dim, 1>;
    using TrimEigenvectors = Eigen::Matrix<std::complex<double>, constants::state_dim, constants::state_dim>;

    struct TrimEigenAnalysis {
        TrimEigenvalues eigenvalues = TrimEigenvalues::Zero();
        TrimEigenvectors eigenvectors = TrimEigenvectors::Zero();
    };

    TrimEigenAnalysis trim_linearization_eigen_analysis(const linearization::TrimLinearization& lin);

    std::string print_eigen_analysis(const TrimEigenAnalysis& eig);

}
