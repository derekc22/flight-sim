#include <Eigen/Eigenvalues>
#include <array>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include "simulation/vehicles/vehicles.hpp"
#include "simulation/linearization/linearization.hpp"

namespace linearization {

    TrimLinearization linearize_trim_solution(vehicles::Aircraft& aircraft, const autopilot::TrimSolution& trim_sol) {
        const autopilot::TrimModel model{
            .structural = aircraft.structural_properties,
            .aerodynamic = aircraft.aerodynamic_properties,
            .actuator = aircraft.actuator_properties,
            .fixed_controls = autopilot::TrimFixedControls{
                .flaps = 0.0,
                .spoilers = 0.0,
            },
        };

        const autopilot::TrimVariableVector_T<double> z = autopilot::pack_trim_variables_T<double>(trim_sol.state, trim_sol.input);
        CppAD::eigen_vector<CppAD::AD<double>> z_t = util::start_autodiff_tracking(z);
        const autopilot::TrimVariableVector_T<CppAD::AD<double>> z_vec = util::eigen_vector_from_cppad_vector<CppAD::AD<double>, autopilot::trim_variable_dofs>(z_t);
        const autopilot::TrimState<CppAD::AD<double>> x_t = autopilot::unpack_trim_state_T<CppAD::AD<double>>(z_vec);
        const autopilot::TrimControlSurfaceInputs<CppAD::AD<double>> u_t = autopilot::unpack_trim_input_T<CppAD::AD<double>>(z_vec);
        const autopilot::TrimStateDot<CppAD::AD<double>> trim_state_dot = autopilot::compute_trim_state_dot_T<CppAD::AD<double>>(x_t, u_t, model, trim_sol.conditions);
        const autopilot::TrimStateDotVector_T<CppAD::AD<double>> x_dot_vec = autopilot::pack_trim_state_dot_T(trim_state_dot);
        const CppAD::eigen_vector<CppAD::AD<double>> x_dot_t = util::cppad_vector_from_eigen_vector(x_dot_vec);
        CppAD::ADFun<double> f(z_t, x_dot_t);
        const Eigen::Matrix<double, autopilot::trim_state_dofs, autopilot::trim_variable_dofs> jac_map = util::compute_jac<autopilot::trim_state_dofs, autopilot::trim_variable_dofs>(f, z);

        TrimLinearization out;
        out.A = jac_map.leftCols<autopilot::trim_state_dofs>();
        out.B = jac_map.rightCols<autopilot::trim_input_dofs>();
        return out;
    }

    std::string print_linearization_solution(const TrimLinearization& lin_sol) {
        std::ostringstream out;
        out << "A:\n" << lin_sol.A << "\n";
        out << "B:\n" << lin_sol.B << "\n";
        return out.str();
    }

}
