#include <cppad/example/cppad_eigen.hpp>
#include <cppad/cppad.hpp>
#include "simulation/autodiff/public/detail/cppad.hpp"
#include "simulation/autodiff/public/detail/evaluation.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public/data/helpers.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/linearization/public/detail/jacobian.hpp"
#include "simulation/operating/public/data/helpers.hpp"

namespace linearization {

    LocalLinearization linearize_operating_point(const autodiff::AutoDiffModel& model, const operating::OperatingPoint_T<double>& operating_point, const operating::OperatingConditions& conditions) {

        const operating::StateInputVector_T<double> xu = operating::unpack_state_input_T<double>(operating_point.state, operating_point.input);

        CppAD::eigen_vector<CppAD::AD<double>> xu_tracked = autodiff::start_autodiff_tracking(xu);  // start of autodiff tracking

        const operating::StateInputVector_T<CppAD::AD<double>> xu_eigen = autodiff::eigen_vector_from_cppad_vector<CppAD::AD<double>, constants::state_input_dim>(xu_tracked);

        const operating::OperatingPoint_T<CppAD::AD<double>> operating_point_cppad = operating::pack_state_input_T(xu_eigen);

        const dynamics::StateDot_T<CppAD::AD<double>> x_dot = autodiff::compute_state_dot_T<CppAD::AD<double>>(operating_point_cppad, model, conditions, constants::dt);
        const dynamics::StateDotVector_T<CppAD::AD<double>> x_dot_vec = dynamics::unpack_state_dot_T(x_dot);

        const CppAD::eigen_vector<CppAD::AD<double>> x_dot_cppad = autodiff::cppad_vector_from_eigen_vector(x_dot_vec);
        
        CppAD::ADFun<double> f(xu_tracked, x_dot_cppad);    // end of autodiff tracking

        const constants::MatrixX_T<double, constants::state_dim, constants::state_input_dim> jac_full = autodiff::compute_jac<constants::state_dim, constants::state_input_dim>(f, xu);

        LocalLinearization out;
        out.A = jac_full.leftCols<constants::state_dim>();
        out.B = jac_full.rightCols<constants::input_dim>();

        return out;
    }


    VirtualLocalLinearization linearize_virtual_operating_point(const autodiff::AutoDiffModel& model, const operating::VirtualOperatingPoint_T<double>& virtual_operating_point) {
        
        const operating::VirtualStateInputVector_T<double> xu_virtual = operating::unpack_virtual_state_input_T<double>(virtual_operating_point.state, virtual_operating_point.input);

        CppAD::eigen_vector<CppAD::AD<double>> xu_virtual_tracked = autodiff::start_autodiff_tracking(xu_virtual);  // start of autodiff tracking

        const operating::VirtualStateInputVector_T<CppAD::AD<double>> xu_virtual_eigen = autodiff::eigen_vector_from_cppad_vector<CppAD::AD<double>, constants::virtual_state_input_dim>(xu_virtual_tracked);

        const operating::VirtualOperatingPoint_T<CppAD::AD<double>> virtual_operating_point_cppad = operating::pack_virtual_state_input_T(xu_virtual_eigen);

        const dynamics::StateDot_T<CppAD::AD<double>> x_dot = autodiff::compute_state_dot_T<CppAD::AD<double>>(virtual_operating_point_cppad, model);
        const dynamics::StateDotVector_T<CppAD::AD<double>> x_dot_vec = dynamics::unpack_state_dot_T(x_dot);

        const CppAD::eigen_vector<CppAD::AD<double>> x_dot_cppad = autodiff::cppad_vector_from_eigen_vector(x_dot_vec);
        
        CppAD::ADFun<double> f(xu_virtual_tracked, x_dot_cppad);    // end of autodiff tracking

        const constants::MatrixX_T<double, constants::state_dim, constants::virtual_state_input_dim> jac_full = autodiff::compute_jac<constants::state_dim, constants::virtual_state_input_dim>(f, xu_virtual);

        VirtualLocalLinearization out;
        out.A_virtual = jac_full.leftCols<constants::state_dim>();
        out.B_virtual = jac_full.rightCols<constants::virtual_input_dim>();

        return out;

    }

}
