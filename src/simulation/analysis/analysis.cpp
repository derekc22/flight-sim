#include "simulation/analysis/analysis.hpp"
#include <sstream>

namespace analysis {

    TrimLinearization linearize_trim_dynamics(const autopilot::TrimState<double>& x, const autopilot::TrimInput<double>& u, const autopilot::TrimModel& model, const autopilot::TrimConditions& conditions) {
        const autopilot::TrimVariableVector_T<double> z = autopilot::pack_trim_variables_T<double>(x, u);
        CppAD::eigen_vector<CppAD::AD<double>> z_t = global::start_autodiff_tracking(z);
        const autopilot::TrimVariableVector_T<CppAD::AD<double>> z_vec = global::eigen_vector_from_cppad_vector<CppAD::AD<double>, autopilot::trim_variable_dofs>(z_t);
        const autopilot::TrimState<CppAD::AD<double>> x_t = autopilot::unpack_trim_state_T<CppAD::AD<double>>(z_vec);
        const autopilot::TrimInput<CppAD::AD<double>> u_t = autopilot::unpack_trim_input_T<CppAD::AD<double>>(z_vec);
        const autopilot::TrimDynamics<CppAD::AD<double>> trim_dynamics = autopilot::compute_trim_dynamics_T<CppAD::AD<double>>(x_t, u_t, model, conditions);
        autopilot::TrimDynamicsVector_T<CppAD::AD<double>> x_dot_vec;
        x_dot_vec << trim_dynamics.vx_dot,
                     trim_dynamics.vy_dot,
                     trim_dynamics.vz_dot,
                     trim_dynamics.p_dot,
                     trim_dynamics.q_dot,
                     trim_dynamics.r_dot,
                     trim_dynamics.phi_dot,
                     trim_dynamics.theta_dot;
        const CppAD::eigen_vector<CppAD::AD<double>> x_dot_t = global::cppad_vector_from_eigen_vector(x_dot_vec);
        CppAD::ADFun<double> f(z_t, x_dot_t);
        const Eigen::Matrix<double, autopilot::trim_state_dofs, autopilot::trim_variable_dofs> jac_map = global::compute_jac<autopilot::trim_state_dofs, autopilot::trim_variable_dofs>(f, z);

        TrimLinearization out;
        out.A = jac_map.leftCols<autopilot::trim_state_dofs>();
        out.B = jac_map.rightCols<autopilot::trim_input_dofs>();
        return out;
    }

    TrimLinearization linearize_trim_solution(vehicles::Aircraft& aircraft, const autopilot::TrimSolution& trim) {
        const autopilot::TrimModel model{
            .structural = aircraft.structural_properties,
            .aerodynamic = aircraft.aerodynamic_properties,
            .control = aircraft.control_properties,
            .fixed_controls = autopilot::TrimFixedControls{
                .flap = 0.0,
                .spoiler = 0.0,
            },
        };

        return linearize_trim_dynamics(trim.state, trim.input, model, trim.conditions);
    }

    std::string print_linerization_solution(const TrimLinearization& lin) {
        std::ostringstream out;
        out << "A:\n" << lin.A << "\n";
        out << "B:\n" << lin.B << "\n";
        return out.str();
    }
}
