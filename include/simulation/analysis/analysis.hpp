#pragma once

#include <Eigen/Dense>
#include <complex>
#include "simulation/autopilot/autopilot.hpp"

namespace analysis {
    using TrimStateJacobian = Eigen::Matrix<double, autopilot::trim_state_dofs, autopilot::trim_state_dofs>;
    using TrimInputJacobian = Eigen::Matrix<double, autopilot::trim_state_dofs, autopilot::trim_input_dofs>;
    using TrimEigenvalues = Eigen::Matrix<std::complex<double>, autopilot::trim_state_dofs, 1>;
    using TrimEigenvectors = Eigen::Matrix<std::complex<double>, autopilot::trim_state_dofs, autopilot::trim_state_dofs>;

    struct TrimLinearization {
        TrimStateJacobian A = TrimStateJacobian::Zero();
        TrimInputJacobian B = TrimInputJacobian::Zero();
    };

    struct TrimEigenAnalysis {
        TrimEigenvalues eigenvalues = TrimEigenvalues::Zero();
        TrimEigenvectors eigenvectors = TrimEigenvectors::Zero();
    };

    TrimLinearization linearize_trim_dynamics(const autopilot::TrimState<double>& x, const autopilot::TrimInput<double>& u, const autopilot::TrimModel& model, const autopilot::TrimConditions& conditions);

    TrimLinearization linearize_trim_solution(vehicles::Aircraft& aircraft, const autopilot::TrimSolution& trim);

    TrimEigenAnalysis trim_linearization_eigen_analysis(const TrimLinearization& lin);

    std::string print_linearization_solution(const TrimLinearization& lin);

    std::string print_eigen_analysis(const TrimEigenAnalysis& eig);

}
