#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>
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
            .surface_actuators = aircraft.actuator_properties.surface_actuators,
            .propulsor_actuators = aircraft.actuator_properties.propulsor_actuators,
            .fixed_controls = trim::TrimFixedActuatorInputs{
                .flap = aircraft.operating_properties.fixed_controls.flap,
                .spoiler = aircraft.operating_properties.fixed_controls.spoiler,
            },
        };

        const trim::TrimVariablesVector_T<double> z = trim::unpack_trim_variables_T<double>(trim_sol.state, trim_sol.input);
        CppAD::eigen_vector<CppAD::AD<double>> z_t = util::start_autodiff_tracking(z);

        const trim::TrimVariablesVector_T<CppAD::AD<double>> z_vec = util::eigen_vector_from_cppad_vector<CppAD::AD<double>, trim::trim_variable_dim>(z_t);
        const trim::TrimState<CppAD::AD<double>> x_t = trim::pack_trim_state_T<CppAD::AD<double>>(z_vec);
        const trim::TrimActuatorInputs<CppAD::AD<double>> u_t = trim::pack_trim_actuator_inputs_T<CppAD::AD<double>>(z_vec);
        
        const trim::TrimStateDot<CppAD::AD<double>> trim_state_dot = trim::compute_trim_state_dot_T<CppAD::AD<double>>(x_t, u_t, model, trim_sol.conditions);
        const trim::TrimStateDotVector_T<CppAD::AD<double>> x_dot_vec = trim::unpack_trim_state_dot_T(trim_state_dot);
        const CppAD::eigen_vector<CppAD::AD<double>> x_dot_t = util::cppad_vector_from_eigen_vector(x_dot_vec);
        
        CppAD::ADFun<double> f(z_t, x_dot_t);
        const Eigen::Matrix<double, constants::state_dim, trim::trim_variable_dim> jac_map = util::compute_jac<constants::state_dim, trim::trim_variable_dim>(f, z);

        TrimLinearization out;
        out.A = jac_map.leftCols<constants::state_dim>();
        out.B = jac_map.rightCols<constants::input_dim>();
        return out;
    }

    std::string print_linearization_solution(const TrimLinearization& lin_sol) {
        std::ostringstream out;
        out << "A:\n" << lin_sol.A << "\n";
        out << "B:\n" << lin_sol.B << "\n";
        return out.str();
    }

    DiscretizedTrimLinearization discretize(const linearization::TrimLinearization& lin_sol){
        int nx = lin_sol.A.rows();
        int nu = lin_sol.B.cols();

        Eigen::MatrixXd M = Eigen::MatrixXd::Zero(nx + nu, nx + nu);

        M.block(0, 0, nx, nx) = lin_sol.A;  // continuous, Ac
        M.block(0, nx, nx, nu) = lin_sol.B; // continuous, Bc

        Eigen::MatrixXd Md = (M * constants::dt).exp();

        Eigen::MatrixXd Ak = Md.block(0, 0, nx, nx);
        Eigen::MatrixXd Bk = Md.block(0, nx, nx, nu);

        return { .A = Ak, .B = Bk };
    }

}
