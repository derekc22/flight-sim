#pragma once

#include <Eigen/Dense>
#include <complex>
#include "simulation/trim/trim.hpp"
#include "simulation/linearization/linearization.hpp"

namespace analysis {
    using TrimEigenvalues = Eigen::Matrix<std::complex<double>, trim::trim_state_dofs, 1>;
    using TrimEigenvectors = Eigen::Matrix<std::complex<double>, trim::trim_state_dofs, trim::trim_state_dofs>;

    struct TrimEigenAnalysis {
        TrimEigenvalues eigenvalues = TrimEigenvalues::Zero();
        TrimEigenvectors eigenvectors = TrimEigenvectors::Zero();
    };

    TrimEigenAnalysis trim_linearization_eigen_analysis(const linearization::TrimLinearization& lin);

    std::string print_eigen_analysis(const TrimEigenAnalysis& eig);

}
