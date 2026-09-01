#include <cppad/example/cppad_eigen.hpp>
#include <cppad/cppad.hpp>
#include "simulation/autodiff/public/detail/cppad.hpp"
#include "simulation/trim/private/detail/residual.hpp"

namespace trim {

    double residual_norm_inf(const TrimResidualVector_T<double>& residual) {
        return residual.cwiseAbs().maxCoeff();
    }

    TrimResidualJacobian compute_trim_residual_jac(const operating::StateInputVector_T<double>& xu, autodiff::AutoDiffModel& model, const TrimTarget& target, const operating::OperatingConditions& conditions) {

        CppAD::eigen_vector<CppAD::AD<double>> xu_tracked = autodiff::start_autodiff_tracking(xu);  // start of autodiff tracking

        const operating::StateInputVector_T<CppAD::AD<double>> xu_eigen = autodiff::eigen_vector_from_cppad_vector<CppAD::AD<double>, constants::state_input_dim>(xu_tracked);

        const TrimResidualVector_T<CppAD::AD<double>> residual_tracked = compute_trim_residual_vector_T<CppAD::AD<double>>(xu_eigen, model, target, conditions);
        const CppAD::eigen_vector<CppAD::AD<double>> residual_tracked_cppad = autodiff::cppad_vector_from_eigen_vector(residual_tracked);

        CppAD::ADFun<double> f(xu_tracked, residual_tracked_cppad); // end of autodiff tracking

        return autodiff::compute_jac<trim_residual_dim, constants::state_input_dim>(f, xu);
    }

}
