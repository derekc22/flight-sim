#include <Eigen/Eigenvalues>
#include <array>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include "simulation/vehicles/vehicles.hpp"
#include "simulation/linearization/linearization.hpp"

namespace linearization {

    TrimLinearization linearize_trim_solution(vehicles::Aircraft& aircraft, const trim::TrimSolution& trim_sol) {
        const trim::TrimModel model{
            .structural = aircraft.structural_properties,
            .aerodynamic = aircraft.aerodynamic_properties,
            .actuator = aircraft.actuator_properties,
            .fixed_controls = trim::TrimFixedControls{
                .flaps = 0.0,
                .spoilers = 0.0,
            },
        };

        const trim::TrimVariableVector_T<double> z = trim::pack_trim_variables_T<double>(trim_sol.state, trim_sol.input);
        CppAD::eigen_vector<CppAD::AD<double>> z_t = util::start_autodiff_tracking(z);
        const trim::TrimVariableVector_T<CppAD::AD<double>> z_vec = util::eigen_vector_from_cppad_vector<CppAD::AD<double>, trim::trim_variable_dofs>(z_t);
        const trim::TrimState<CppAD::AD<double>> x_t = trim::unpack_trim_state_T<CppAD::AD<double>>(z_vec);
        const trim::TrimControlSurfaceInputs<CppAD::AD<double>> u_t = trim::unpack_trim_input_T<CppAD::AD<double>>(z_vec);
        const trim::TrimStateDot<CppAD::AD<double>> trim_state_dot = trim::compute_trim_state_dot_T<CppAD::AD<double>>(x_t, u_t, model, trim_sol.conditions);
        const trim::TrimStateDotVector_T<CppAD::AD<double>> x_dot_vec = trim::pack_trim_state_dot_T(trim_state_dot);
        const CppAD::eigen_vector<CppAD::AD<double>> x_dot_t = util::cppad_vector_from_eigen_vector(x_dot_vec);
        CppAD::ADFun<double> f(z_t, x_dot_t);
        const Eigen::Matrix<double, trim::trim_state_dofs, trim::trim_variable_dofs> jac_map = util::compute_jac<trim::trim_state_dofs, trim::trim_variable_dofs>(f, z);

        TrimLinearization out;
        out.A = jac_map.leftCols<trim::trim_state_dofs>();
        out.B = jac_map.rightCols<trim::trim_input_dofs>();
        return out;
    }

    std::string print_linearization_solution(const TrimLinearization& lin_sol) {
        std::ostringstream out;
        out << "A:\n" << lin_sol.A << "\n";
        out << "B:\n" << lin_sol.B << "\n";
        return out.str();
    }

}
